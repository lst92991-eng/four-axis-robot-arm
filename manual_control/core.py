"""处理 v3 点动、停稳和故障恢复，由上层传入 CAN 发送函数。"""

from collections import deque
import json
import math
from pathlib import Path
import struct
import time

PACKET = struct.Struct('<4BI14i')
ZERO, MOVE, STOP, OFF, CLEAR, PING = 2, 3, 4, 5, 6, 7
VELOCITY = 12
COMMAND_NAMES = {
    ZERO: 'ZERO',
    MOVE: 'MOVE',
    STOP: 'STOP',
    OFF: 'OFF',
    CLEAR: 'CLEAR',
    PING: 'PING',
    VELOCITY: 'VELOCITY',
}
LEASE_S, STATUS_S, ACK_S = 0.30, 0.25, 0.25
STATES = ['未出力', '保持位置', '运动中', '减速中', '故障', '速度运行']
FAULTS = {1: '编码器反馈异常', 2: '电流或采样保护', 3: '母线电压异常', 4: '控制时序异常', 5: '运动跟随或行程保护'}


def packet(kind, node, seq, a=0, b=0, c=0, d=0):
    return PACKET.pack(kind, node, 0, 3, seq, a, b, c, d, *([0] * 10))


class Controller:
    #---------初始化区---------
    # 配置决定每轴能力；status保存反馈，pending保存待应答请求，active保存当前按压
    def __init__(self, config, send, clock=time.monotonic, audit=None, reference_path=None):
        self.config, self.send, self.clock = config, send, clock
        self.audit = audit or (lambda event: None)
        self.axes = {int(n): dict(c) for n, c in config['axes'].items()}
        for c in self.axes.values():
            low, high = c.get('firmware_minimum', -180), c.get('firmware_maximum', 180)
            if not (-10000 <= low <= c['minimum'] < c['maximum'] <= high <= 10000):
                raise ValueError('主机行程必须在该构建已审阅的固件范围内')
            if c.get('manual_direction', 1) not in (-1, 1):
                raise ValueError('手动方向映射必须为+1或-1')
            if c['ratio'] <= 0 or not c['builds']:
                raise ValueError('缺少减速比或已核对的固件构建 ID')
            if 'diagnostic_step_deg' in c and not 0.1 <= c['diagnostic_step_deg'] <= 5:
                raise ValueError('诊断步长必须为0.1～5度')
            if 'default_speed_deg_s' in c:
                axis_limit = min(config['max_speed_deg_s'], c.get('max_speed_deg_s', 30))
                if not 0.1 <= c['default_speed_deg_s'] <= axis_limit:
                    raise ValueError('轴默认速度超出可用范围')
        if not (0.1 <= config['default_speed_deg_s'] <= config['max_speed_deg_s'] <= 500):
            raise ValueError('速度配置无效')
        self.status, self.pending, self.synced = {}, {}, set()
        self.seq = {n: 0 for n in self.axes}
        self.sync_tries = {}
        self.recovering = None
        self.reference_path = Path(reference_path) if reference_path else None
        self.reference_restore = None
        if self.reference_path and self.reference_path.exists():
            self.reference_restore = json.loads(self.reference_path.read_text(encoding='utf-8'))
            self.reference_restore['phase'] = 'paused'
        self.reference_deadline = 0.0
        self.owner, self.armed, self.active = None, False, None
        self.lease = 0
        self.last_ping = 0
        self.last_order = {}
        self.stopping = set()
        self.stop_ack = set()
        self.stop_deadline = None
        self.message = '只接收状态；尚未开启手动控制'
        self.events = deque(maxlen=100)
        self.connected = False
        self.tx_count = 0

    #---------指令与出力区---------
    def action(self, action, client='', order=0, **args):
        if action == 'stop':
            # 停止请求优先处理，更新已接收的操作序号。
            self.last_order[client] = max(order, self.last_order.get(client, 0))
            self.recovering = None
            self.stop('已请求停止并保持位置', disarm=bool(args.get('disarm', False)))
            return
        if not isinstance(order, int) or order <= self.last_order.get(client, 0):
            raise ValueError('过期的操作已丢弃')
        self.last_order[client] = order
        now = self.clock()
        if action == 'recover':
            return self._request_recover(args.get('node'), now)
        if action == 'arm':
            return self._request_arm(client, now)
        if not self.armed or client != self.owner:
            raise ValueError('请先点击「开启手动控制」')
        self.lease = now
        if action == 'heartbeat':
            press = args.get('press')
            if self.active and press != self.active['press']:
                self.stop('按压状态丢失，已请求停止', disarm=True)
            return
        if action == 'release':
            if self.active and args.get('press') == self.active['press']:
                self.stop('已松开，减速并保持位置')
            return
        if action == 'idle_z':
            if self.active or self.stopping or self.reason(1):
                raise ValueError('Z 轴需要先停稳')
            if self.status[1]['state'] == 1:
                self.transmit(1, OFF)
            return
        if action not in ('jog', 'position'):
            raise ValueError('不支持的操作')
        self._request_motion(action, args)

    def _request_arm(self, client, now):
        if self.active or self.stopping or self.recovering is not None:
            raise ValueError('请先等待停稳')
        if self.owner and self.owner != client and self.armed:
            raise ValueError('另一个页面正在控制')
        # 先做序号同步，发送运动前再核对停稳反馈。
        ready = [
            n
            for n in self.axes
            if self.status.get(n, {}).get('state') in (0, 1) and not self.reason(n, require_still=False)
        ]
        if not ready:
            raise ValueError('当前没有满足运动条件的轴，详情见各轴卡片')
        self.owner, self.armed, self.lease = client, True, now
        self.synced.clear()
        self.pending.clear()
        self.sync_tries = {n: 0 for n in ready}
        for n in ready:
            self.transmit(n, PING, syncing=True)
        self.message = '手动控制已开启；按住按钮运动，松开减速保持'
        self.log(self.message)

    def _request_motion(self, action, args):
        if self.active or self.stopping:
            raise ValueError('一次只能操作一个轴，请先松开并等它停稳')
        n = args.get('node')
        speed = args.get('speed')
        if type(n) is not int or n not in self.axes:
            raise ValueError('轴号无效')
        axis = self.axes[n]
        axis_max = min(self.config['max_speed_deg_s'], axis.get('max_speed_deg_s', 30))
        if type(speed) not in (int, float) or not math.isfinite(speed) or not 0.1 <= speed <= axis_max:
            raise ValueError('速度超出允许范围')
        if speed * axis['ratio'] > 1440:
            raise ValueError('电机速度超限')
        reason = self.reason(n)
        if reason:
            raise ValueError(reason)
        if n not in self.synced:
            raise ValueError('等待 PING 序号同步')
        press = args.get('press')
        if not isinstance(press, str) or not 1 <= len(press) <= 80:
            raise ValueError('缺少本次按压标识')
        if action == 'position':
            target = args.get('target')
            if (
                n == 1
                or not self.commissioning_firmware(self.status[n])
                or type(target) not in (int, float)
                or not -2000000 <= target <= 2000000
            ):
                raise ValueError('旋转轴位置无效')
            self.active = dict(node=n, press=press, speed=speed, target=target)
            self.transmit(n, MOVE, round(target * 1000), round(speed * 1000))
            self.message = f'J{n} 移至 {target:.3f}°'
            return
        direction = args.get('direction')
        if direction not in (-1, 1):
            raise ValueError('方向无效')
        motor_direction = direction * axis.get('manual_direction', 1)
        if self.velocity_mode(n):
            command = VELOCITY
            value = round(motor_direction * speed * 1000)  # 毫度/秒。
            speed_mdeg_s = 0
            message = f'J{n} 手动速度运行 {speed:g}°/s；松开快速制动'
        else:
            # 位置式点动沿指定方向走到行程边界，诊断时只走一小段。
            position = self.status[n]['position']
            step = axis.get('diagnostic_step_deg')
            target = axis['maximum' if motor_direction == 1 else 'minimum']
            message = f'J{n} 连续运行 {"+" if direction > 0 else "−"} {speed:g}°/s'
            if step is not None:
                target = max(axis['minimum'], min(axis['maximum'], position + motor_direction * step))
                message += f'，本次诊断最多 {step:g}°'
            if (target - position) * motor_direction <= 0.1:
                raise ValueError('已到该方向的软件行程边界')
            command = MOVE
            value = round(target * 1000)  # 毫度。
            speed_mdeg_s = round(speed * 1000)
        self.active = dict(node=n, press=press, speed=speed, direction=direction, motor_direction=motor_direction)
        self.transmit(n, command, value, speed_mdeg_s)
        self.message = message

    #---------运动条件区---------
    # 汇总该轴当前不能运动的原因，页面显示与动作入口共用同一判断
    def reason(self, n, require_still=True):
        s = self.status.get(n)
        if not self.connected or not s or self.clock() - s['received'] > STATUS_S:
            return '未收到实时状态，请检查驱动板供电和 CAN'
        if s['build'] not in self.axes[n]['builds'] and not self.commissioning_firmware(s):
            return '固件版本未核对，需先确认该版本行程与协议'
        if n == 1 and self.reference_restore is not None:
            return 'J1 原高度参考恢复尚未确认，可继续清故障恢复'
        if s['fault'] or s['state'] == 4:
            detail = {1: '实际位置越界', 2: '轨迹参考越界', 3: '轨迹超时', 4: '位置跟随误差超限', 5: '停车终点越界'}
            label = detail.get(s.get('motion_fault_detail')) if s['fault'] == 5 else None
            return f"驱动故障 {s['fault']}：{label or FAULTS.get(s['fault'], '未知故障')}"
        if not s['configured']:
            if s.get('calibration_extension'):
                return '尚未完成电角标定'
            return '电角配置未就绪（已烧录版本的电角零点为 -1）'
        if not s['referenced']:
            return '驱动板未报告有效的机械参考'
        if not s['encoder_valid']:
            return '编码器反馈无效'
        if s['interlocked']:
            return '驱动已通信失联锁定，需要另行处理'
        c = self.axes[n]
        if require_still and not self.velocity_mode(n) and not c['minimum'] <= s['position'] <= c['maximum']:
            return '当前位置超出主机行程'
        if require_still and (s['state'] not in (0, 1) or not s['stationary']):
            return '等待轴停稳'
        return ''

    # 调试构建与既有实测构建仍有能力差别，集中在这里识别
    def commissioning_firmware(self, status):
        build = self.config.get('commissioning_build')
        return bool(build and status.get('build') == build)

    def velocity_mode(self, n):
        return self.axes[n].get('manual_mode') == 'velocity' or self.commissioning_firmware(self.status.get(n, {}))

    #---------CAN收发区---------
    def transmit(self, n, kind, a=0, b=0, syncing=False):
        self.seq[n] = (self.seq[n] + 1) & 0xFFFFFFFF
        seq = self.seq[n]
        self.pending[n, seq] = dict(kind=kind, sent=self.clock(), syncing=syncing)
        self.send(n, packet(kind, n, seq, a, b))
        self.tx_count += 1
        if kind != PING:
            self.log(
                f'J{n} 发送 {COMMAND_NAMES.get(kind,str(kind))}',
                node=n,
                kind=kind,
                sequence=seq,
                target_mdeg=a,
                speed_mdeg_s=b,
            )

    def receive(self, identifier, data, received_at=None):
        n = identifier - 0x200
        ack_node = identifier - 0x480
        if n not in self.axes and ack_node not in self.axes:
            return
        if len(data) != 64:
            self.fail('收到不完整的节点报文')
            return
        kind, node, result, version, seq, *v = PACKET.unpack(data)
        if version != 3 or node != (n if n in self.axes else ack_node):
            self.fail('节点或协议版本不匹配')
            return
        now = self.clock() if received_at is None else received_at
        if n in self.axes:
            self._receive_status(n, kind, result, v, now)
        else:
            self._receive_ack(node, kind, result, seq)

    def _receive_status(self, n, kind, result, v, now):
        # 状态字段转换完，再处理重启、参考接续和停稳确认。
        if kind != 0x40 or result != 0:
            self.fail('状态帧格式不正确')
            return
        flags = (v[9] & 0xFFFFFFFF) >> 16
        old = self.status.get(n)
        uptime = v[0] & 0xFFFFFFFF
        s = dict(
            received=now,
            uptime=uptime,
            position=v[1] / 1000,
            target=v[2] / 1000,
            velocity=v[4] / 1000,
            state=v[9] & 255,
            fault=(v[9] >> 8) & 255,
            build=f'{v[10] & 0xffffffff:08x}',
        )
        if v[13] == 0x43414C31:
            s.update(
                calibration_extension=True,
                cal_state=v[3] & 255,
                motion_fault_detail=(v[3] >> 24) & 255,
                cal_error=(v[3] >> 8) & 255,
                electrical_zero=v[6],
                encoder_direction=v[7],
                encoder_counts=v[5],
                calibration_saved=bool(v[3] & (1 << 17)),
                calibration_progress=v[11],
                bus_mv=v[12],
            )
        for name, bit in [
            ('referenced', 2),
            ('interlocked', 8),
            ('configured', 16),
            ('encoder_valid', 32),
            ('power', 64),
            ('stationary', 256),
        ]:
            s[name] = bool(flags & bit)
        self.status[n] = s
        if s['fault'] and (not old or old['fault'] != s['fault']):
            self.log(
                f"J{n} 故障 {s['fault']}：{FAULTS.get(s['fault'], '未知故障')}",
                node=n,
                fault=s['fault'],
                position=s['position'],
                target=s['target'],
                velocity=s['velocity'],
                power=s['power'],
            )
        if old and ((uptime - old['uptime']) & 0xFFFFFFFF) >= 0x80000000:
            self.synced.discard(n)
            self.fail(f'J{n} 重启，禁止自动恢复运动')
        elif old and s['build'] != old['build']:
            self.synced.discard(n)
            self.fail(f'J{n} 固件发生变化')
        if n == 1:
            self.recovery_feedback(s)
        if n in self.stopping and s['state'] == 4 and not s['power']:
            self.stopping.remove(n)
            self.stop_ack.discard(n)
            self.log(f'J{n} 故障已关断出力；未据此判定机械停稳')
        elif (
            n in self.stopping
            and n in self.stop_ack
            and (s['state'] == 0 and not s['power'] or s['state'] == 1 and s['stationary'])
        ):
            self.stopping.remove(n)
            self.stop_ack.discard(n)
            self.log(f'J{n} 反馈确认停止', position=s['position'], power=s['power'])

    def _receive_ack(self, node, kind, result, seq):
        # 应答只完成对应序号的操作，不把ACK当作已经到位。
        pending = self.pending.pop((node, seq), None)
        if pending is None:
            return
        if kind != pending['kind'] | 0x80 or result not in (0, 1):
            if self.recovering == node:
                self.recovering = None
            self.fail('命令应答格式不正确')
            return
        if pending['syncing']:
            recovering = self.recovering == node
            if not self.armed and not recovering:
                return
            if result:
                # 用 PING 试探序号，每次跨约四分之一的 32 位空间，最多四次。
                self.sync_tries[node] += 1
                if self.sync_tries[node] >= 4:
                    if recovering:
                        self.recovering = None
                    self.fail(f'J{node} PING 序号同步失败')
                    return
                self.seq[node] = (self.seq[node] + 0x3FFFFFFF) & 0xFFFFFFFF
                self.transmit(node, PING, syncing=True)
            elif recovering:
                if node == 1 and self.reference_restore is not None and not self.status[node]['fault']:
                    self.restore_legacy_reference(self.status[node])
                else:
                    self.transmit(node, CLEAR)
            else:
                self.synced.add(node)
                self.log(f'J{node} 序号同步完成（仅 PING）')
            return
        if pending['kind'] == CLEAR:
            if node == 1 and self.reference_restore is not None and not result and self.recovering == 1:
                self.reference_restore.update(phase='clear_feedback', feedback_after=self.status[node]['uptime'])
                return
            self.recovering = None
            if not result:
                self.synced.add(node)
            self.message = f'J{node} ' + ('恢复被驱动拒绝，请看当前状态' if result else '恢复命令已接受')
            self.log(self.message)
            return
        if pending['kind'] == ZERO and node == 1 and self.reference_restore is not None:
            if result or self.recovering != 1:
                self.recovering = None
                self.fail('J1 原高度参考写入未确认，可继续恢复')
            else:
                self.reference_restore.update(phase='zero_feedback', feedback_after=self.status[node]['uptime'])
            return
        if result:
            if pending['kind'] == STOP:
                self.stop_ack.add(node)
                self.log(f'J{node} STOP 被拒绝，必须看停稳反馈；不再续发心跳')
                self.armed = False
            else:
                self.fail(f'J{node} 命令被驱动拒绝，不重发运动')
        elif pending['kind'] != PING:
            if pending['kind'] == STOP:
                self.stop_ack.add(node)
            self.log(f'J{node} {COMMAND_NAMES.get(pending["kind"],str(pending["kind"]))} 已接受')

    #---------停稳与通信时限区---------
    # 控制线程周期调用，检查租约、应答期限、保活和停稳反馈
    def tick(self):
        now = self.clock()
        if self.recovering == 1 and self.reference_restore is not None and now > self.reference_deadline:
            self.recovering = None
            self.fail('J1 高度参考恢复超时，可继续恢复')
        if self.armed and now - self.lease > LEASE_S:
            self.stop('页面失联，已请求停止；需重新允许点动', disarm=True)
        if self.armed:
            for n in self.synced:
                reason = self.reason(n, require_still=False)
                if reason:
                    self.fail(f'J{n}: {reason}')
                    break
        expired = [k for k, p in self.pending.items() if now - p['sent'] > ACK_S]
        for key in expired:
            # Controller由控制线程串行调用，列表中的待应答项在本轮仍然存在
            p = self.pending.pop(key)
            status = self.status.get(key[0], {})
            if (
                p['kind'] == PING
                and not p['syncing']
                and now - status.get('received', 0) <= STATUS_S
                and not status.get('interlocked')
            ):
                continue
            if key[0] == self.recovering:
                self.recovering = None
                self.log(f'J{key[0]} 恢复应答超时，请看当前状态')
            self.fail(f'J{key[0]} 应答超时；不重发运动命令')
        if now - self.last_ping >= 0.10:
            self.last_ping = now
            # 页面失联时先停运动轴。已经停稳的保持轴继续收 CAN 心跳。
            keepalive = (
                self.synced
                if self.armed
                else {
                    n
                    for n in self.synced
                    if n not in self.stopping
                    and self.status.get(n, {}).get('state') == 1
                    and not self.reason(n, require_still=False)
                }
            )
            for n in sorted(keepalive):
                if not any(key[0] == n and (p['kind'] != PING or p['syncing']) for key, p in self.pending.items()):
                    self.transmit(n, PING)
        if self.stopping and self.stop_deadline is not None and now > self.stop_deadline:
            self.armed = False
            self.message = '停止尚未获反馈确认，请观察实物；主机已停止 PING'
            self.log(self.message)
            self.stop_deadline = None

    # 先撤销当前动作，再给仍在运行的轴发STOP；停止结果由新反馈确认
    def stop(self, reason, disarm=False):
        active_node = self.active['node'] if self.active else None
        active_speed = self.active['speed'] if self.active else 0
        self.active = None
        nodes = {n for n in self.synced if n in self.status and self.status[n]['state'] in (2, 3, 5)}
        if active_node is not None:
            s = self.status.get(active_node, {})
            fault_off = s.get('state') == 4 and not s.get('power') and self.clock() - s.get('received', 0) <= STATUS_S
            if not fault_off:
                nodes.add(active_node)  # 首帧运动反馈回来前，也要停止已发出的 MOVE。
            else:
                self.log(f'J{active_node} 驱动已故障关断出力，不重复发 STOP')
        if disarm:
            self.armed = False
        self.message = reason
        self.log(reason)
        for n in nodes - self.stopping:
            self.stopping.add(n)
            self.stop_ack.discard(n)
            try:
                self.transmit(n, STOP)
            except Exception as exc:
                self.armed = False
                self.log(f'J{n} STOP 发送失败；停止续发 PING，等待驱动看门狗', error=str(exc))
        if self.stopping:
            stop_time = self.clock()
            brake_time = max(
                max(abs(self.status.get(n, {}).get('velocity', 0)), active_speed if n == active_node else 0)
                / self.axes[n].get('brake_deceleration_deg_s2', self.axes[n].get('acceleration_deg_s2', 30))
                for n in self.stopping
            )
            # 期限包含预计制动时间与停稳反馈的等待时间。
            self.stop_deadline = max(self.stop_deadline or 0, stop_time + brake_time + 2 + STATUS_S)

    def fail(self, reason):
        if self.armed or self.active:
            self.stop(reason, disarm=True)
        else:
            self.message = reason

    #---------故障恢复区---------
    def _request_recover(self, n, now):
        if type(n) is not int or n not in self.axes or not self.can_recover(n):
            raise ValueError('当前轴无法恢复，请查看状态')
        if n == 1 and self.axes[n].get('restore_reference_after_clear'):
            if self.reference_restore is None:
                self.reference_restore = dict(before=dict(self.status[n]), phase='sync')
                self.save_reference_restore()
            self.reference_deadline = now + 3
        self.recovering = n
        self.sync_tries[n] = 0
        self.transmit(n, PING, syncing=True)
        self.message = f'正在恢复 J{n}'

    def can_recover(self, n):
        s = self.status.get(n, {})
        legacy_reference = (
            n == 1
            and self.axes[n].get('restore_reference_after_clear')
            and s.get('build') in self.axes[n]['builds']
            and s.get('calibration_extension')
            and s.get('stationary')
            and not s.get('power')
            and (s.get('referenced') or self.reference_restore is not None)
        )
        return (
            self.connected
            and self.recovering is None
            and not self.armed
            and not self.active
            and not self.stopping
            and (self.commissioning_firmware(s) or legacy_reference)
            and self.clock() - s['received'] <= STATUS_S
            and s['encoder_valid']
            and (
                (legacy_reference and self.reference_restore is not None and s['state'] in (0, 4))
                or (s['fault'] and not s['power'])
                or (s['interlocked'] and s['state'] == 1 and s['stationary'])
            )
        )

    def recovery_feedback(self, status):
        restore = self.reference_restore
        if self.recovering != 1 or restore is None:
            return
        if status['uptime'] == restore.get('feedback_after'):
            return
        if status['state'] != 0 or status['fault'] or status['power'] or not status['stationary']:
            return
        if restore['phase'] == 'clear_feedback':
            self.restore_legacy_reference(status)
        elif (
            restore['phase'] == 'zero_feedback'
            and status['referenced']
            and abs(status['position'] - restore['target_mdeg'] / 1000) <= 0.05
        ):
            self.reference_restore = None
            self.save_reference_restore()
            self.recovering = None
            self.synced.add(1)
            self.message = 'J1 故障已清除，原 Z 参考已接续'
            self.log(self.message, position=status['position'])

    def restore_legacy_reference(self, status):
        restore = self.reference_restore
        before = restore['before']
        if (
            status['build'] != before['build']
            or status['electrical_zero'] != before['electrical_zero']
            or status['encoder_direction'] != before['encoder_direction']
            or ((status['uptime'] - before['uptime']) & 0xFFFFFFFF) >= 0x80000000
        ):
            self.recovering = None
            self.fail('J1 重启或电角参考发生变化，需重新确认机械参考')
            return
        delta = (status['encoder_counts'] - before['encoder_counts'] + 8192) % 16384 - 8192
        if abs(delta) > 46 or status['power'] or not status['stationary'] or not status['encoder_valid']:
            self.recovering = None
            self.fail('J1 恢复期间位置发生变化，需重新确认机械参考')
            return
        restore['target_mdeg'] = round((before['position'] + delta * 360 / 16384) * 1000)
        restore['phase'] = 'zero_ack'
        self.save_reference_restore()
        self.transmit(1, ZERO, restore['target_mdeg'])

    def save_reference_restore(self):
        if self.reference_path:
            if self.reference_restore is None:
                self.reference_path.unlink(missing_ok=True)
            else:
                temporary = self.reference_path.with_suffix('.tmp')
                temporary.write_text(json.dumps(self.reference_restore, indent=2), encoding='utf-8')
                temporary.replace(self.reference_path)

    #---------状态与记录区---------
    # 把内部状态整理成页面字段，读取快照不会发送运动指令
    def snapshot(self):
        rows = []
        for n, axis in self.axes.items():
            status = self.status.get(n, {})
            age = self.clock() - status['received'] if status else None
            direction = axis.get('manual_direction', 1)
            reason = self.reason(n)
            row = dict(axis)
            row.update(status)
            row.update(
                node=n,
                manual_mode='velocity' if self.velocity_mode(n) else 'position',
                age_ms=round(age * 1000) if age is not None else None,
                display_position=status.get('position', 0) * direction,
                display_velocity=status.get('velocity', 0) * direction,
                state_text=STATES[status['state']] if status.get('state', 99) < len(STATES) else '未知',
                reason=reason,
                recoverable=self.can_recover(n),
                ready=self.armed and n in self.synced and not reason,
            )
            rows.append(row)
        return dict(
            connected=self.connected,
            armed=self.armed,
            active=self.active,
            recovering=self.recovering,
            owner=self.owner,
            stopping=sorted(self.stopping),
            message=self.message,
            axes=rows,
            tx_count=self.tx_count,
            events=list(self.events),
            default_speed=self.config['default_speed_deg_s'],
            max_speed=self.config['max_speed_deg_s'],
        )

    def log(self, message, **extra):
        event = dict(time=time.strftime('%H:%M:%S'), message=message, **extra)
        self.events.append(event)
        self.audit(event)
