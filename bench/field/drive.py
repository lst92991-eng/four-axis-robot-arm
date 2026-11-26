"""USB-CAN v3 executor for the live Windows bench. All angles are driver degrees."""

from datetime import datetime
from pathlib import Path
import json
import math
import os
import sys
import time

ROOT = Path(__file__).resolve().parents[2]
WORK = ROOT / 'output/bench'
sys.path.insert(0, str(ROOT / 'temp/transport'))
sys.path.insert(1, str(ROOT / 'manual_control'))
from stepfoc_candlelight import Candlelight, Frame, is_candlelight_status_report
from core import CLEAR, MOVE, OFF, PACKET, PING, STOP, ZERO, packet

PROFILE = {'saved_travel_deg': -18420.27,
 'acceleration_deg_s2': 5050,
 'brake_deg_s2': 4400,
 'running_bus_max_mv': 40000,
 'speed_kp_milli': 1480,
 'speed_ki_milli': 100000,
 'rotary_arrival_deg': {'3': 0.65, '2': 0.15},
 'position_hold_start': True,
 'game_speed_deg_s': 4800,
 'voltage_limit_mv': 40000,
 'descent_brake_deg_s2': 4800,
 'descent_brake_ki_permille': 950,
 'release_acceleration_deg_s2': 5050,
 'synchronize_xy': True,
 'j3_speed_gains': [1000, 14000],
 'rotary_ramps': {'2': {'acceleration_deg_s2': 110, 'brake_deg_s2': 75},
                  '3': {'acceleration_deg_s2': 180, 'brake_deg_s2': 125}}}
Z_ORIGIN = PROFILE['saved_travel_deg']
SET_SPEED_GAINS = 10
SET_PROFILE = 13
NODES = (1, 2, 3)
SERIAL = os.environ.get('SCARA_CAN_SERIAL', '')


# ---------运动参数区---------
# 横移参数同时给出 J2 和 J3 的速度、加速斜率和制动斜率。
def apply_rotary_tuning(settings):
    kp, ki, v2, v3, a2, a3, b2, b3, sync, *extras = settings
    PROFILE['j3_speed_gains'] = (round(kp * 1000), round(ki * 1000))
    PROFILE['rotary_ramps'] = {'2': dict(acceleration_deg_s2=a2, brake_deg_s2=b2),
                              '3': dict(acceleration_deg_s2=a3, brake_deg_s2=b3)}
    PROFILE['synchronize_xy'] = sync
    PROFILE['j3_shape_delay_ms'] = extras[0] if extras else 0
    PROFILE['j3_brake_lead_ms'] = extras[1] if len(extras) > 1 else 0
    PROFILE['j3_settle_gain_milli'] = round(extras[2] * 1000) if len(extras) > 2 else 0
    return v2, v3


def j3_peak_speed(distance, requested, acceleration, brake):
    # 与板端 speedRamp 的位移相同。短段的峰速在主机求，减少受理 MOVE 时的板端计算。
    def ramp(v, a, entry):
        exit_time = .060
        transition = .5*a*(entry+exit_time)
        if v < transition:
            return v*(entry+2*exit_time)/3*math.sqrt(v/transition)
        return v*v/(2*a) + v*exit_time/2 + a*(entry*entry-exit_time*exit_time)/24

    def travel(v):
        return ramp(v, acceleration, .100) + ramp(v, brake, .240)

    # 如果这段角行程容不下请求的峰速，用二分法求能在段内加速和制动的峰速。
    if travel(requested) <= distance:
        return requested
    low, high = 0., requested
    for _ in range(22):
        mid = .5*(low+high)
        if travel(mid) <= distance:
            low = mid
        else:
            high = mid
    return max(.001, math.floor(low*1000)/1000)


# ---------CAN FD 命令与状态区---------
class Drive:
    def __init__(self, *, record=True):
        owner = None
        if os.name == 'nt':
            from windows_owner import UsbOwner as owner
        self.can = Candlelight(serial=SERIAL, fd=True, owner_factory=owner)
        self.states = {}
        self.wrist_status = None
        self.sequences = {n: int(time.time() * 1000) % 0x3fffffff + 20 for n in NODES}
        self.acks = {}
        self.last_sent = {n: 0 for n in NODES}
        self.last_update = {}
        self.on_poll = None
        self.profile_set = False
        self.z_ramps = None
        self.rotary_ramps = {}
        self.sync_ping = None
        run_dir = os.environ.get('SCARA_RUN_DIR') if record else None
        self.trace = (Path(run_dir) / 'can.jsonl').open('a', encoding='utf-8') if run_dir else None

    def note(self, event, **fields):
        if self.trace is None:
            return
        self.trace.write(json.dumps(dict(time=datetime.now().astimezone().isoformat(), event=event, **fields),
                                    ensure_ascii=False) + '\n')
        self.trace.flush()

    # 发送时写入节点和递增序号。J1 的绝对位置在上总线前扣除固定原点偏移。
    def send(self, n, kind, a=0, b=0, *, c=0, d=0):
        seq = (self.sequences[n] + 1) & 0xFFFFFFFF
        self.sequences[n] = seq
        wire_angle = a - round(Z_ORIGIN * 1000) if n == 1 and kind in (MOVE, ZERO) else a
        self.can.send(Frame(0x400 + n, packet(kind, n, seq, wire_angle, b, c, d), fd=True, brs=True))
        self.last_sent[n] = time.monotonic()
        if kind != PING:
            self.note('tx', node=n, kind=kind, sequence=seq, a=a, b=b, c=c, d=d)
        return seq

    # 处理设备心跳和接收队列，每个节点保留最近一份有效状态。
    def poll(self, active=True):
        if self.on_poll is not None:
            self.on_poll()
        now = time.monotonic()
        if active:
            for n in NODES:
                a = self.states.get(n, {})
                if a.get('state') in (1, 2, 3, 5) and now - self.last_sent[n] > .10:
                    self.send(n, PING)
        f = self.can.recv(.028)
        # 一次 USB 批量读取可能包含多帧，先处理完缓冲中的整帧。
        while f is not None:
            self.receive_frame(f)
            f = self.can.recv(0)

    def receive_frame(self, f):
        if f.echo:
            return
        if f.error:
            if not is_candlelight_status_report(f):
                self.note('can_controller_report', identifier=f.identifier, data=f.data.hex())
            if f.identifier & 0x40:
                raise RuntimeError('USB-CAN bus-off')
            return
        if f.extended or not f.fd or not f.brs or len(f.data) != PACKET.size:
            return
        k, n, result, version, seq, *v = PACKET.unpack(f.data)
        if n not in (1, 2, 3, 4) or version != 3:
            return
        # 应答按节点、命令和序号关联到发送请求。周期状态另行更新各轴读数。
        if f.identifier == 0x480 + n:
            if k != (PING | 0x80) or (n, seq) == self.sync_ping:
                self.acks[n, k, seq] = result, v
            return
        if f.identifier != 0x200 + n or k != 0x40 or result != 0:
            return
        # 把反馈中的千分之一度还原为度，J1 的位置和目标加回原点偏移。
        flags = (v[9] & 0xffffffff) >> 16
        origin = Z_ORIGIN if n == 1 else 0
        item = dict(position=origin + v[1] / 1000, target=origin + v[2] / 1000, velocity=v[4] / 1000,
                    counts=v[5], uptime=v[0] & 0xffffffff, build=f'{v[10] & 0xffffffff:08x}',
                    state=v[9] & 255, fault=v[9] >> 8 & 255, detail=v[3] >> 24 & 255,
                    bus_mv=v[12], referenced=bool(flags & 2),
                    interlocked=bool(flags & 8), configured=bool(flags & 16), encoder_valid=bool(flags & 32),
                    power=bool(flags & 64), stationary=bool(flags & 256))
        if n == 4:
            self.wrist_status = item
            return
        self.states[n] = item
        self.last_update[n] = time.monotonic()

    def healthy(self):
        now = time.monotonic()
        for n in NODES:
            a = self.states.get(n)
            if a is None or now - self.last_update[n] > .25:
                raise RuntimeError(f'J{n} status expired')
            if a['fault'] == 5 and a['detail'] == 3:
                raise RuntimeError(f"J{n} 驱动轨迹超时，目标 {a['target']:.3f}°，实到 {a['position']:.3f}°")
            # 旋转轴出力时，板端电流环按实际采样锁存母线故障；使能前核对供电。
            check_supply = n == 1 or not a['power']
            bus_max = PROFILE['running_bus_max_mv']
            if a['fault'] or a['interlocked'] or not a['configured'] \
                    or not a['referenced'] or not a['encoder_valid'] or (check_supply and not 10000 <= a['bus_mv'] <= bus_max):
                raise RuntimeError(f'J{n} state unavailable: {a}')

    def wait(self, condition, seconds, check=True, elapsed_ok=False):
        until = time.monotonic() + seconds
        while time.monotonic() < until:
            self.poll()
            if check:
                self.healthy()
            if condition():
                return
        if not elapsed_ok:
            raise TimeoutError('CAN state or ACK timeout')

    # ---------启动检查与参数回读区---------
    def read_initial(self):
        self.wait(lambda: len(self.states) == 3 and
                  all(time.monotonic() - self.last_update[n] < .18 for n in NODES), .75, check=False)
        # 停在目标处的通信联锁只需重新握手，保留机械参考和保持力。
        for n in NODES:
            axis = self.states[n]
            if (axis['interlocked'] and axis['state'] == 1 and axis['power']
                    and axis['configured'] and axis['referenced']
                    and axis['encoder_valid'] and not axis['fault'] and (n != 1 or PROFILE.get('position_hold_start'))):
                self.sync_sequence(n, check=False)
                self.wait(lambda: self.states[n]['stationary'], 1.2, check=False)
                self.command(n, CLEAR, check=False)
                self.wait(lambda: not self.states[n]['interlocked'], .6, check=False)
        self.healthy()
        self.wait(lambda: all(a['state'] in (0, 1) and a['stationary'] for a in self.states.values()), 2)
        # 首次取得稳定状态后，同步三轴序号，再把各轴的加减速参数写入驱动。
        if not self.profile_set:
            for n in NODES:
                self.sync_sequence(n)
            self.command(1, SET_PROFILE, round(PROFILE['brake_deg_s2'] * 1000),
                         round(PROFILE['acceleration_deg_s2'] * 1000))
            for n in (2, 3):
                ramps = PROFILE.get('rotary_ramps', {}).get(str(n))
                if ramps:
                    self.command(n, SET_PROFILE, round(ramps['brake_deg_s2'] * 1000),
                                 round(ramps['acceleration_deg_s2'] * 1000),
                                 c=PROFILE.get('j3_shape_delay_ms', 0) if n == 3 else 0,
                                 d=PROFILE.get('j3_brake_lead_ms', 0) if n == 3 else 0)
            # J3 处于失能状态时写入实测速度环增益。
            if self.states[3]['state'] == 0 and not self.states[3]['power']:
                self.command(3, SET_SPEED_GAINS, *PROFILE.get('j3_speed_gains', (1000, 35000)),
                             d=PROFILE.get('j3_settle_gain_milli', 0))
            self.profile_set = True
        self.note('initial', axes=self.states)
        return self.states

    def sync_sequence(self, n, check=True):
        try:
            for _ in range(4):
                self.sync_ping = n, (self.sequences[n] + 1) & 0xFFFFFFFF
                seq = self.send(n, PING)
                if self.ack(n, PING, seq, check=check, allow_reject=True):
                    return
                self.sequences[n] = (seq + 0x3FFFFFFF) & 0xFFFFFFFF
            raise RuntimeError(f'J{n} sequence sync rejected')
        finally:
            self.sync_ping = None

    # 发出命令后等同序号的应答。斜率和增益命令还要对照驱动返回的参数值。
    def command(self, n, kind, a=0, b=0, check=True, *, c=0, d=0):
        settings = kind in (SET_SPEED_GAINS, SET_PROFILE)
        deadline = time.monotonic() + 1.6
        while True:
            seq = self.send(n, kind, a, b, c=c, d=d)
            values = self.ack(n, kind, seq, check, allow_reject=kind == SET_PROFILE, read_values=settings)
            if values is not False:
                break
            stamp = self.states[n]['uptime']
            remaining = deadline - time.monotonic()
            if remaining <= 0 or self.states[n]['state'] not in (0, 1):
                raise RuntimeError(f'J{n} rejected ramp configuration')
            self.wait(lambda: self.states[n]['uptime'] != stamp and self.states[n]['stationary'], remaining, check=check)
        if settings:
            if kind == SET_SPEED_GAINS:
                matches = values[:2] == [a, b] and (c == 0 or values[3] == c) and (d == 0 or values[4] == d)
            else:
                matches = values[0] == a and (b == 0 or values[1] == b) and values[2:4] == [c, d]
            reply = values[:5] if kind == SET_SPEED_GAINS and d else values[:4]
            self.note('parameter_readback', node=n, kind=kind, sequence=seq, values=reply)
            if not matches:
                raise RuntimeError(f'J{n} parameter readback {values[:4]} differs from {(a, b, c, d)}')
            if n == 1 and kind == SET_PROFILE:
                self.z_ramps = tuple(values[:3])
            if n in (2, 3) and kind == SET_PROFILE:
                self.rotary_ramps[n] = tuple(values[:2])
            return reply

    def ack(self, n, kind, seq, check=True, allow_reject=False, *, read_values=False):
        key = n, kind | 0x80, seq
        self.wait(lambda: key in self.acks, .45, check=check)
        result, values = self.acks.pop(key)
        if result != 0:
            self.note('rejected', node=n, kind=kind, sequence=seq, actual=self.states[n])
            if allow_reject:
                return False
            raise RuntimeError(f'J{n} rejected CAN action {kind}: {self.states[n]}')
        if kind == MOVE and values[0] > 0:
            self.note('ack', node=n, kind=kind, sequence=seq, applied_speed_deg_s=values[0]/1000)
        else:
            self.note('ack', node=n, kind=kind, sequence=seq)
        return values if read_values else True

    # ---------单轴定位区---------
    def move(self, n, goal, speed, tolerance=.12, *, on_rise=None):
        self.wait(lambda: self.states[n]['state'] in (0, 1) and self.states[n]['stationary'], 2)
        initial = self.states[n]['position']
        distance = abs(goal - initial)
        if distance <= tolerance:
            return
        held_start = n == 1 and PROFILE.get('position_hold_start', False) and self.states[n]['state'] == 1
        if self.states[n]['power'] and not held_start:
            self.idle(n)
        print(f'J{n} MOVE {initial:.3f} -> {goal:.3f} degrees', flush=True)
        seen = self.states[n]['uptime']
        for attempt in range(4):
            seq = self.send(n, MOVE, round(goal * 1000), round(speed * 1000))
            if self.ack(n, MOVE, seq, allow_reject=n == 1):
                break
            stamp = self.states[n]['uptime']
            if held_start:
                self.wait(lambda: self.states[n]['uptime'] != stamp and self.states[n]['state'] == 1
                          and self.states[n]['stationary'] and abs(self.states[n]['target'] - self.states[n]['position']) <= 2.0, .75)
            else:
                self.wait(lambda: self._off_ready(n, stamp), .75)
        else:
            raise RuntimeError('J1 position request repeatedly rejected')

        # 用新反馈核对当前位置、驱动目标和保持状态，决定这一段能否交给下一步。
        def arrived():
            nonlocal seen
            a = self.states[n]
            if on_rise and a['state'] == 2 and a['power'] and a['velocity'] > .05:
                if 0 < ((a['uptime'] - seen) & 0xFFFFFFFF) < 0x80000000:
                    seen = a['uptime']
                    on_rise(a.copy())
                    self.healthy()
                    a = self.states[n]
            target_matches = abs(a['target'] - goal) <= .01 and abs(a['position'] - goal) <= tolerance
            settled = a['stationary'] and a['state'] == 1
            slow_near_goal = n == 3 and a['state'] == 2 and abs(a['velocity']) <= .20
            return target_matches and (settled or slow_near_goal)
        if n == 1 and self.z_ramps:
            ramps = speed * 1000 / self.z_ramps[1] + speed * 1000 / self.z_ramps[0]
        else:
            ramps = 2 * speed / 30
        self.wait(arrived, distance / speed + ramps + 7)
        if self.states[n]['state'] == 2:
            self.command(n, STOP)
            self.wait(lambda: self.states[n]['stationary'] and self.states[n]['state'] == 1, 3)
            if abs(self.states[n]['position'] - goal) > .20:
                raise RuntimeError(f'J{n} stopped away from the requested position')
        self.note('arrived', node=n, goal=goal, actual=self.states[n])

    def idle(self, n):
        self.command(n, OFF)
        acknowledged_at = self.states[n]['uptime']
        self.wait(lambda: self._off_ready(n, acknowledged_at), 2)

    def _off_ready(self, n, after_uptime):
        a = self.states[n]
        fresh = 0 < ((a['uptime'] - after_uptime) & 0xFFFFFFFF) < 0x80000000
        return fresh and a['state'] == 0 and not a['power'] and a['stationary'] and abs(a['velocity']) <= .05

    # ---------J2 与 J3 横移区---------
    def xy(self, shoulder_goal, elbow_goal, shoulder_speed, elbow_speed=30.0):
        """Give both rotary drives an absolute target, then leave them in HOLD."""
        nodes = (2, 3)
        goals = {2: round(shoulder_goal * 1000), 3: round(elbow_goal * 1000)}
        speeds = {2: shoulder_speed, 3: elbow_speed}
        self.wait(lambda: all(self.states[n]['state'] in (0, 1) and self.states[n]['stationary']
                              for n in nodes), 2)
        initial = {n: self.states[n].copy() for n in nodes}
        # 已在同一目标保持的轴继续闭环出力，只对目标发生改变的轴下发 MOVE。
        tolerance = {n:PROFILE.get('rotary_arrival_deg', {}).get(str(n), .10) + .001 for n in nodes}
        at_previous_target = {n:initial[n]['state'] == 1 and initial[n]['power']
                              and abs(initial[n]['target'] - initial[n]['position']) <= tolerance[n] for n in nodes}
        changed = [n for n in nodes if not (at_previous_target[n] and round(initial[n]['target'] * 1000) == goals[n])]
        synchronize = PROFILE.get('synchronize_xy')
        if synchronize:
            # 让两轴按同一进度起步和制动，较短的一轴按行程比例取速度、斜率。
            distance = {n: abs(goals[n] / 1000 - initial[n]['target' if at_previous_target[n] else 'position']) for n in nodes}
            moving = [n for n in nodes if distance[n] > .02]
            if moving:
                ramps = PROFILE['rotary_ramps']
                velocity = min(speeds[n] / distance[n] for n in moving)
                synced_elbow_speed = velocity * distance[3]
                if (len(changed) == 2 and len(moving) == 2 and at_previous_target[3]
                        and synced_elbow_speed < min(3., elbow_speed)):
                    # 肘轴行程很短时用各自的斜率，同时发两个目标，肘轴先到就保持。
                    synchronize = False
                    self.note('xy_independent_short_elbow', shoulder_goal=shoulder_goal,
                              elbow_goal=elbow_goal,
                              synced_elbow_speed=synced_elbow_speed)
                else:
                    acceleration = min(ramps[str(n)]['acceleration_deg_s2'] / distance[n] for n in moving)
                    brake = min(ramps[str(n)]['brake_deg_s2'] / distance[n] for n in moving)
                    for n in moving:
                        speeds[n] = max(.1, velocity * distance[n])
                        self.command(n, SET_PROFILE, round(1000 * max(10, brake * distance[n])),
                                     round(1000 * max(10, acceleration * distance[n])),
                                     c=PROFILE.get('j3_shape_delay_ms', 0) if n == 3 else 0,
                                     d=PROFILE.get('j3_brake_lead_ms', 0) if n == 3 else 0)
        if not synchronize:
            for n in changed:
                ramps = PROFILE.get('rotary_ramps', {}).get(str(n))
                if ramps:
                    self.command(n, SET_PROFILE, round(1000 * ramps['brake_deg_s2']),
                                 round(1000 * ramps['acceleration_deg_s2']),
                                 c=PROFILE.get('j3_shape_delay_ms', 0) if n == 3 else 0,
                                 d=PROFILE.get('j3_brake_lead_ms', 0) if n == 3 else 0)
        if 3 in changed and 3 in self.rotary_ramps:
            goal = goals[3]/1000
            distances = [abs(goal-initial[3]['position'])]
            if at_previous_target[3]:
                distances.append(abs(goal-initial[3]['target']))
            distance = max(0., min(distances)-.002)
            braking, acceleration = self.rotary_ramps[3]
            peak = j3_peak_speed(distance, speeds[3], acceleration/1000, braking/1000)
            if peak < speeds[3]:
                self.note('j3_profile_peak', distance_deg=distance, requested=speeds[3], peak=peak)
                if synchronize:
                    speeds[2] = max(.001, speeds[2]*peak/speeds[3])
                speeds[3] = peak
        started = time.monotonic()
        # 先向需要转动的轴发完绝对目标，再逐轴等待应答，使两轴起动尽量接近。
        sequences = {n: self.send(n, MOVE, goals[n], round(speeds[n] * 1000)) for n in changed}
        self.note('xy_position_batch', goals=goals, speeds=speeds,
                  transmit_ms=(time.monotonic() - started) * 1000, initial=initial)
        for n in changed:
            seq = sequences[n]
            for attempt in range(4):
                if self.ack(n, MOVE, seq, allow_reject=True):
                    break
                if attempt == 3:
                    raise RuntimeError(f'J{n} repeatedly rejected the target')
                rejected_at = self.states[n]['uptime']
                self.wait(lambda: self.states[n]['uptime'] != rejected_at and
                          self.states[n]['state'] in (0, 1) and self.states[n]['stationary'], .3)
                seq = self.send(n, MOVE, goals[n], round(speeds[n] * 1000))

        # 收到起步之后的新状态帧，再核对 HOLD、出力、静止标志和实际角误差。
        def held_at_target(n):
            a = self.states[n]
            return (a['uptime'] != initial[n]['uptime'] and a['state'] == 1 and a['power']
                    and a['stationary'] and abs(a['target'] - goals[n] / 1000) <= .001
                    and abs(a['position'] - goals[n] / 1000) <= tolerance[n])

        rotary_ramps = PROFILE.get('rotary_ramps', {})
        timeout = max(abs(goals[n] / 1000 - initial[n]['position']) / speeds[n]
                      + speeds[n] / rotary_ramps.get(str(n), {}).get('acceleration_deg_s2', 30)
                      + speeds[n] / rotary_ramps.get(str(n), {}).get('brake_deg_s2', 30)
                      for n in nodes) + 7
        try:
            self.wait(lambda: all(held_at_target(n) for n in nodes), timeout)
        except TimeoutError as error:
            self.note('xy_not_settled', goals=goals, actual={n:self.states[n] for n in nodes})
            angles = '，'.join(f"J{n} 目标 {goals[n]/1000:.3f}°、实际 {self.states[n]['position']:.3f}°" for n in nodes)
            raise TimeoutError(f"驱动有反馈，横移到位超时。{angles}") from error
        self.note('xy_position_held', goals=goals, actual={n:self.states[n] for n in nodes},
                  elapsed_s=time.monotonic() - started)

    # ---------J1 升降与整组停车区---------
    def z(self, goal, high=-19353.34, precision=4.5, speed_limit=90.0, *, on_rise=None, release=False):
        if not self.profile_set:
            raise RuntimeError('J1 fast profile was not applied')
        if 'descent_brake_deg_s2' in PROFILE and abs(goal - self.states[1]['position']) > precision:
            up = goal > self.states[1]['position']
            direction_limit = 'z_ascend_speed_deg_s' if up else 'z_descend_speed_deg_s'
            speed_limit = min(speed_limit, PROFILE.get(direction_limit, speed_limit))
            accel = PROFILE['release_acceleration_deg_s2'] if release or on_rise else PROFILE['acceleration_deg_s2']
            brake = PROFILE['brake_deg_s2'] if up else PROFILE['descent_brake_deg_s2']
            scale = 0 if up else PROFILE['descent_brake_ki_permille']
            # 按当前运动方向选择起动和制动参数，配置有变化才写进 J1。
            ramps = (round(brake*1000), round(accel*1000), scale)
            if self.z_ramps != ramps:
                self.command(1, SET_PROFILE, ramps[0], ramps[1], c=scale)
        self.move(1, goal, speed_limit, precision, on_rise=on_rise)

    # 向参与运动的节点请求停车，随后用新状态确认驱动已停稳。
    def stop(self):
        failed = False
        before = {n: a['uptime'] for n, a in self.states.items()}
        # Issue STOP independently per moving axis, even if another axis has a fault.
        for n in NODES:
            try:
                self.send(n, STOP)
            except Exception as e:
                self.note('stop_send_failed', node=n, error=repr(e))
                failed = True
        until = time.monotonic() + 3
        while time.monotonic() < until:
            try:
                self.poll()
            except Exception:
                failed = True
            if len(self.states) == 3 and all(time.monotonic() - self.last_update[n] < .25 and
                    self.states[n]['uptime'] != before.get(n) and
                    a['stationary'] and (a['state'] in (0, 4) and not a['power'] or a['state'] == 1)
                    for n, a in self.states.items()):
                self.note('stopped', axes=self.states)
                return not failed
        self.note('stop_not_confirmed', axes=self.states)
        return False

    def close(self):
        self.can.close()
        if self.trace is not None:
            self.trace.close()

    # 回到观察位并已发停车命令后，核对三轴状态，再完成失能和可选的记录。
    def finish_park(self, save=False):
        # Park STOP already ran. Finish the normal disarm using fresh feedback.
        def stopped_without_fault():
            return len(self.states) == 3 and all(
                a['state'] in (0, 1) and a['stationary'] and not a['fault'] and a['referenced']
                and a['encoder_valid'] for a in self.states.values()
            )
        self.wait(stopped_without_fault, 3, check=False)
        for n in NODES:
            a = self.states[n]
            if a['interlocked'] and n != 1:
                if a['state'] != 1:
                    raise RuntimeError(f'J{n} cannot resume the hold watchdog handshake')
                self.command(n, CLEAR, check=False)
                self.wait(lambda: not self.states[n]['interlocked'], .6, check=False)
            if self.states[n]['power']:
                self.command(n, OFF, check=False)
                self.wait(lambda: self.states[n]['state'] == 0 and not self.states[n]['power'], .6, check=False)
            # The older J1 image requires OFF and a same-angle ZERO to clear a hold lock.
            # Retain the multi-turn reference and verify the raw count has not moved.
            if n == 1 and self.states[n]['interlocked']:
                before = self.states[n].copy()
                self.wait(lambda: self.states[n]['uptime'] != before['uptime'], .5, check=False)
                after = self.states[n]
                if not after['stationary'] or abs(after['counts'] - before['counts']) > 2 or after['state'] != 0:
                    raise RuntimeError('J1 position changed while releasing stationary lock')
                self.command(1, ZERO, round(after['position'] * 1000), check=False)
                self.wait(lambda: self.states[1]['referenced'] and not self.states[1]['interlocked'] and
                          abs(self.states[1]['position'] - after['position']) < .10, .6, check=False)
        self.read_initial()
        if save:
            WORK.mkdir(parents=True, exist_ok=True)
            (WORK / 'usb-game-park-readback.json').write_text(
                json.dumps(self.states, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
            print(json.dumps(self.states, ensure_ascii=False), flush=True)
