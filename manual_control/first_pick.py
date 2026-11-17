"""通过本机点动面板执行一次已示教的取放。"""

import argparse
from datetime import datetime
import json
import math
from pathlib import Path
import re
import subprocess
import time
from urllib.error import HTTPError
from urllib.request import Request, build_opener, ProxyHandler
import uuid

URL = 'http://127.0.0.1:8090'
GPIO = '/sys/class/gpio/gpio40/value'
RV1106_SERIAL = 'abc8f01242c327ec'
ROOT = Path(__file__).resolve().parents[1]
CALIBRATION = ROOT / 'output/xy-registration-20261111'
PICK_PLAN = ROOT / 'output/first-pick-c3-c4-20261111/trial-plan.json'


class Trial:
    #---------初始化区---------
    # 读入已示教的角度、释放脉冲和机械参考，并取得本机页面控制凭据
    def __init__(self, plan, adb, log, nodes=(1, 2, 3), *, calibration=CALIBRATION):
        self.plan = plan
        self.adb = [adb, '-s', RV1106_SERIAL, 'shell']
        self.log = log
        self.release_pulse_ms = plan.get('release_pulse_ms', 1000)
        if type(self.release_pulse_ms) is not int or self.release_pulse_ms <= 0:
            raise ValueError('release_pulse_ms 必须为正整数')
        self.opener = build_opener(ProxyHandler({}))
        with self.opener.open(URL + '/', timeout=2) as response:
            self.key = re.search(r"const\s+key\s*=\s*['\"]([^'\"]+)['\"]", response.read().decode())[1]
        self.client = str(uuid.uuid4())
        self.order = 0
        self.press = None
        self.last_ping = 0.0
        self.owns_panel = False
        self.nodes = nodes
        self.pulse = None
        self.pick = float(plan['z']['pickup_deg'])
        self.z_speed = float(plan['z'].get('speed_deg_s', 180))
        self.high = float(plan['z']['travel_deg'])
        self.span = abs(self.high - self.pick)
        self.degrees_per_mm = self.span / float(plan['z'].get('operator_estimated_rise_mm', 20))
        self.z_tolerance = max(0.10, self.degrees_per_mm * 0.12)
        self.up = 1 if self.high > self.pick else -1
        release_offset = float(plan['z'].get('release_offset_mm', 0))
        self.release_height = self.pick + self.up * self.degrees_per_mm * release_offset
        if not math.isfinite(self.release_height) or (self.release_height - self.high) * self.up > 0:
            raise ValueError('放棋高度必须有效且不高于搬运高度')
        arm_file = calibration / 'arm-registration.json'
        self.arm = json.loads(arm_file.read_text(encoding='utf-8'))
        if not self.span > 0 or not all(math.isfinite(x) for x in (self.pick, self.high)):
            raise ValueError('需要两个实测的 Z 高度')

    #---------取放流程区---------
    def run(self, place_held=False):
        self.acquire()
        if place_held:
            self.note('resume_with_held_piece')
        else:
            self.pick_piece(self.plan['source'])
        self.place_piece(self.plan['destination'])
        axes = self.sample()['axes']
        self.note('motion_complete', axes={n: a['position'] for n, a in axes.items()})

    def pick_piece(self, target, height=None):
        self.xy(target)
        self.z(self.pick if height is None else height)
        self.note('pickup_height', grid=target['grid_id'])
        self.z(self.high)

    def place_piece(self, target, height=None):
        self.xy(target)
        self.z(self.release_height if height is None else height)
        self.release_piece(target['grid_id'])

    #---------控制权与反馈区---------
    def acquire(self, require_raised=True):
        state = self.request('/api/state')
        if state['simulated'] or state['active'] or state['stopping']:
            raise RuntimeError('手动操作尚未停稳')
        if require_raised and (state['axes'][1]['position'] - self.high) * self.up < -self.z_tolerance:
            raise RuntimeError('当前 Z 已离开示教抬升位置')
        self.action('stop', disarm=True)
        for node in self.nodes:
            state = self.request('/api/state')
            axis = state['axes'][node]
            if not (axis['fault'] or axis['interlocked'] or not axis['referenced'] or axis['recoverable']):
                continue
            end = time.monotonic() + 1.5
            while not axis['recoverable'] and time.monotonic() < end:
                time.sleep(0.03)
                state = self.request('/api/state')
                axis = state['axes'][node]
            if not axis['recoverable']:
                raise RuntimeError(axis['reason'])
            self.action('recover', node=node)
            end = time.monotonic() + 1.5
            while time.monotonic() < end:
                state = self.request('/api/state')
                axis = state['axes'][node]
                if not (axis['interlocked'] or axis['fault'] or state['recovering']):
                    break
                time.sleep(0.03)
            else:
                raise RuntimeError(f'J{node} 恢复未确认')
        self.wait(
            lambda s: not s['armed'] and not s['stopping'] and all(s['axes'][n]['stationary'] for n in self.nodes),
            1.5,
        )
        self.action('arm')
        self.wait(lambda s: s['armed'] and s['owner'] == self.client, 0.25)
        self.owns_panel = True
        self.wait(lambda s: all(s['axes'][n]['ready'] for n in self.nodes), 2)
        self.gpio(0)

    # HTTP读取或提交，轴列表在这里转换成按节点索引的数据
    def request(self, path, body=None):
        data = None if body is None else json.dumps(body).encode()
        request = Request(URL + path, data, headers={'Content-Type': 'application/json'})
        try:
            with self.opener.open(request, timeout=0.7) as response:
                result = json.load(response)
            # 状态列表在接收处转换一次，后续直接按节点读取反馈
            if path == '/api/state':
                result['axes'] = {axis['node']: axis for axis in result['axes']}
            return result
        except HTTPError as exc:
            try:
                message = json.load(exc).get('error')
            except ValueError:
                message = None
            raise RuntimeError(message or str(exc)) from exc

    def action(self, action, **args):
        self.order += 1
        return self.request(
            '/api/action', dict(key=self.key, client=self.client, order=self.order, action=action, **args)
        )

    # 在持有控制权时续租，取得新状态并核对参与轴的实际反馈
    def sample(self):
        now = time.monotonic()
        if self.owns_panel and now - self.last_ping >= 0.075:
            self.action('heartbeat', press=self.press)
            self.last_ping = now
        state = self.request('/api/state')
        if self.owns_panel and (
            not state['armed'] or state['owner'] != self.client or state['message'] == '已请求停止并保持位置'
        ):
            raise RuntimeError(f'试走已停止（{state["message"]}）')
        axes = state['axes']
        for n in self.nodes:
            a = axes[n]
            if not a['referenced'] or a['fault'] or a['interlocked'] or a['age_ms'] is None or a['age_ms'] > 250:
                raise RuntimeError(f'J{n} {a["reason"] or "驱动状态异常"}')
        return state

    # 等待期间持续收反馈和续租，满足条件才交付状态
    def wait(self, predicate, seconds, watch_press=False):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            s = self.sample()
            if watch_press and (s['active'] or {}).get('press') != self.press:
                raise RuntimeError('运动指令已取消')
            if predicate(s):
                return s
            time.sleep(0.022)
        raise TimeoutError('等待轴到位或停稳超时')

    #---------轴指令区---------
    # 等待停稳时可重试申请，动作被接受后通过反馈确认本次按压标识
    def begin(self, action, node, **args):
        self.press = str(uuid.uuid4())
        deadline = time.monotonic() + 1.5
        while True:
            try:
                self.action(action, node=node, press=self.press, **args)
                break
            except RuntimeError as exc:
                if str(exc) != '等待轴停稳' or time.monotonic() >= deadline:
                    raise
                self.sample()
                time.sleep(0.022)
        self.last_ping = time.monotonic()
        self.wait(lambda s: (s['active'] or {}).get('press') == self.press, 0.6)

    def end(self, node):
        self.action('release', press=self.press)
        self.press = None
        return self.wait(lambda s: not s['active'] and not s['stopping'] and s['axes'][node]['stationary'], 4)

    def at_rest(self, node):
        seen = None

        def settled(state):
            nonlocal seen
            axis = state['axes'][node]
            if axis['state'] not in (0, 1) or not axis['stationary']:
                seen = None
                return False
            if seen is None:
                seen = axis['uptime']
            return axis['uptime'] != seen

        return self.wait(settled, 2)['axes'][node]

    def position(self, node, goal, speed):
        axis = self.at_rest(node)
        distance = abs(goal - axis['position'])
        if distance <= 0.10 and axis['power'] and axis['state'] == 1:
            return False
        self.begin('position', node, target=goal, speed=speed)

        def arrived(s):
            a = s['axes'][node]
            return (
                a['state'] in (1, 2)
                and a['stationary']
                and abs(a['target'] - goal) <= 0.002
                and abs(a['position'] - goal) <= 0.12
            )

        self.wait(arrived, distance / speed + 2 * speed / 30 + 5, True)
        s = self.end(node)
        self.note('rotation', axis=node, goal=goal, position=s['axes'][node]['position'])
        return True

    #---------平面与升降区---------
    # 历史实测流程仍先J2再J3，结束后用正解核对吸头的毫米位置
    def xy(self, target):
        z = self.sample()['axes'][1]
        if (z['position'] - self.high) * self.up < -self.z_tolerance:
            raise RuntimeError('Z 轴还没到搬运高度')
        if self.position(2, target['j2_deg'], 8):
            # The belt drives J3 briefly while J2 finishes braking.
            settled_at = time.monotonic() + 0.25
            self.wait(lambda _: time.monotonic() >= settled_at, 0.6)
        self.approach_j3(target['j3_motor_deg'])
        for _ in range(2):
            axes = self.sample()['axes']
            if self.tip_error(target, axes) <= 2.5:
                break
            self.trim_j3(target['j3_motor_deg'])
        axes = self.sample()['axes']
        error = self.tip_error(target, axes)
        if error > 2.5:
            raise RuntimeError('两旋转轴停稳后偏离取放点')
        self.note('xy', grid=target['grid_id'], angles=[axes[n]['position'] for n in (2, 3)], tip_error_mm=error)

    def trim_j3(self, goal):
        initial = self.at_rest(3)['position']
        delta = goal - initial
        if abs(delta) < 0.1:
            return
        direction = 1 if delta > 0 else -1
        speed = min(8, max(3, abs(delta) * 1.5))
        self.begin('jog', 3, direction=direction, speed=speed)

        def close_enough(s):
            a = s['axes'][3]
            v = max(0.0, direction * a['velocity'])
            return direction * (goal - a['position']) <= v * v / 60 + 0.05 * v

        self.wait(close_enough, abs(delta) / speed + 4, True)
        self.end(3)

    def tip_error(self, target, axes):
        arm = self.arm
        q2 = arm['shoulder_ref_rad'] + math.radians(
            (axes[2]['position'] - arm['j2_reference_deg']) * arm['j2_scale'] * arm['j2_sign']
        )
        q3 = arm['elbow_ref_rad'] + math.radians(
            (axes[3]['position'] - arm['j3_reference_motor_deg']) * arm['j3_sign'] / arm['j3_ratio']
        )
        l1, l2 = arm['arm_lengths_mm']
        xy = (l1 * math.cos(q2) + l2 * math.cos(q2 + q3), l1 * math.sin(q2) + l2 * math.sin(q2 + q3))
        return math.dist(xy, target['base_xy_mm'])

    def approach_j3(self, goal, brake_lead=0.55):
        initial = self.at_rest(3)['position']
        delta = goal - initial
        if abs(delta) <= 9:
            return
        direction = 1 if delta > 0 else -1
        speed = 30
        self.begin('jog', 3, direction=direction, speed=speed)

        def close_enough(s):
            a = s['axes'][3]
            speed_toward = max(0.0, direction * a['velocity'])
            if direction * (a['position'] - initial) < -5:
                raise RuntimeError('J3 反馈位置离目标越来越远')
            return direction * (goal - a['position']) <= max(5.0, brake_lead * speed_toward)

        self.wait(close_enough, abs(delta) / 5 + 4, True)
        settled = self.end(3)
        self.note('j3_approach', goal=goal, position=settled['axes'][3]['position'])

    def z(self, target, speed_limit=None):
        speed_limit = self.z_speed if speed_limit is None else speed_limit
        for attempt in range(3):
            initial = self.at_rest(1)
            delta = target - initial['position']
            if abs(delta) <= self.z_tolerance or (target == self.high and delta * self.up <= 0):
                self.note('z', goal=target, position=initial['position'])
                return
            if attempt == 0 and initial['power']:
                # 丝杆换向前退出保持，清除上一段运动留下的保持电流。
                self.action('idle_z')
                self.wait(lambda s: s['axes'][1]['state'] == 0 and not s['axes'][1]['power'], 1)
                initial = self.at_rest(1)
                delta = target - initial['position']
            direction = 1 if delta > 0 else -1
            # 下降到取子高度前预留低速接近的距离。
            brake_goal = target
            if attempt == 0 and delta * self.up < -2 * self.degrees_per_mm:
                brake_goal += self.up * 0.8 * self.degrees_per_mm
            speed = min(speed_limit if attempt == 0 else min(25, speed_limit), max(1, abs(delta) * 1.5))
            self.begin('jog', 1, direction=direction * initial.get('manual_direction', 1), speed=speed)
            start = time.monotonic()

            def brake_now(state):
                a = state['axes'][1]
                v = min(speed, max(0.0, direction * a['velocity'], (time.monotonic() - start) * 60))
                return direction * (brake_goal - a['target']) <= v * v / (2 * 360) + 0.033 * v

            self.wait(brake_now, abs(delta) / speed + speed / 60 + 4, True)
            self.end(1)
        final = self.sample()['axes'][1]['position']
        if abs(final - target) > self.z_tolerance and not (target == self.high and (final - target) * self.up >= 0):
            raise RuntimeError(f'Z 停在 {final:.3f}°，目标 {target:.3f}°')
        self.note('z', goal=target, position=final)

    #---------吸放区---------
    def gpio(self, value, write=True):
        command = f"echo {value} > {GPIO} && cat {GPIO}" if write else f"cat {GPIO}"
        with subprocess.Popen(
            self.adb + [command],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        ) as process:
            end = time.monotonic() + 2
            while process.poll() is None and time.monotonic() < end:
                self.sample()
                time.sleep(0.02)
            if process.poll() is None:
                process.kill()
                raise TimeoutError('磁铁输出回读超时')
            stdout, _ = process.communicate()
            if process.returncode or stdout.strip() != str(value).encode():
                raise RuntimeError('磁铁 GPIO 输出未确认')
        self.note('gpio', value=value)

    def release_piece(self, grid):
        # RV1106 确认放棋脉冲归零后再抬升。
        seconds = self.release_pulse_ms / 1000
        remote = (
            f"set -e; trap 'echo 0 > {GPIO}' EXIT; trap 'exit 1' HUP INT TERM; "
            f"echo 1 > {GPIO}; cat {GPIO}; busybox sleep {seconds:.3f}; "
            f"echo 0 > {GPIO}; cat {GPIO}"
        )
        self.pulse = subprocess.Popen(
            self.adb + [remote],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )
        try:
            deadline = time.monotonic() + seconds + 2
            while self.pulse.poll() is None:
                if time.monotonic() >= deadline:
                    raise TimeoutError('释放脉冲回读超时')
                self.sample()
                time.sleep(0.02)
            stdout, _ = self.pulse.communicate()
            if self.pulse.returncode or stdout.split() != [b'1', b'0']:
                raise RuntimeError('释放脉冲高低电平未确认')
        except BaseException:
            self.cancel_pulse()
            raise
        self.pulse = None
        self.note('release_pulse', grid=grid, duration_ms=self.release_pulse_ms, levels=[1, 0])
        self.z(self.high)

    def cancel_pulse(self):
        if self.pulse is None:
            return
        try:
            if self.pulse.poll() is None:
                self.pulse.terminate()
                self.pulse.wait(timeout=1)
        finally:
            subprocess.run(self.adb + [f'echo 0 > {GPIO}'], timeout=2, check=True)
            self.pulse = None

    #---------停止与记录区---------
    def stop(self):
        try:
            if self.owns_panel:
                self.action('stop', disarm=True)
        finally:
            self.cancel_pulse()
        self.owns_panel = False

    def note(self, step, **data):
        row = dict(step=step, time=datetime.now().astimezone().isoformat(), **data)
        line = json.dumps(row, ensure_ascii=False)
        print(line, flush=True)
        with self.log.open('a', encoding='utf-8') as f:
            f.write(line + '\n')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('plan', type=Path)
    parser.add_argument('--adb', required=True)
    parser.add_argument('--calibration', type=Path, default=CALIBRATION)
    parser.add_argument('--run', action='store_true')
    parser.add_argument('--place-held', action='store_true', help='Finish placement of a piece already held')
    args = parser.parse_args()
    if not args.run:
        parser.error('需要 --run 执行本次取放')
    plan = json.loads(args.plan.read_text(encoding='utf-8-sig'))
    trial = Trial(plan, args.adb, args.plan.with_name('trial-events.jsonl'), calibration=args.calibration)
    try:
        trial.run(args.place_held)
    except Exception as exc:
        trial.note('stopped', error=str(exc))
        raise
    finally:
        trial.stop()


if __name__ == '__main__':
    main()
