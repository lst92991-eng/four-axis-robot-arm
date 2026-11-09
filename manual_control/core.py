"""v3 point-to-point jog supervisor. No USB, HTTP or GPIO dependencies.

MOVE is sent once, toward the reviewed firmware limit. PING sustains the
firmware's 500 ms watchdog only while the browser lease remains alive.
STOP brakes and holds; OFF/ZERO/CLEAR are intentionally not exposed.
"""
from collections import deque
import math
import struct
import time

PACKET = struct.Struct('<4BI14i')
MOVE, STOP, PING = 3, 4, 7
VELOCITY=12
COMMAND_NAMES={MOVE:'MOVE',STOP:'STOP',PING:'PING',VELOCITY:'VELOCITY'}
LEASE_S, STATUS_S, ACK_S = .30, .25, .25
STATES = ['未出力', '保持位置', '运动中', '减速中', '故障', '速度运行']
FAULTS = {1:'编码器反馈异常', 2:'电流或采样保护', 3:'母线电压异常',
          4:'控制时序异常', 5:'运动跟随或行程保护'}


def packet(kind, node, seq, a=0, b=0):
    return PACKET.pack(kind, node, 0, 3, seq, a, b, *([0] * 12))


class Controller:
    def __init__(self, config, send, clock=time.monotonic, audit=None):
        self.config, self.send, self.clock = config, send, clock
        self.audit = audit or (lambda event: None)
        self.axes = {int(n): dict(c) for n, c in config['axes'].items()}
        for c in self.axes.values():
            low,high=c.get('firmware_minimum',-180),c.get('firmware_maximum',180)
            if not (-10000<=low<=c['minimum']<c['maximum']<=high<=10000):
                raise ValueError('主机行程必须在该构建已审阅的固件范围内')
            if c.get('manual_direction',1) not in (-1,1):
                raise ValueError('手动方向映射必须为+1或-1')
            if c['ratio'] <= 0 or not c['builds']:
                raise ValueError('缺少减速比或已核对的固件构建 ID')
            if 'diagnostic_step_deg' in c and not .1 <= c['diagnostic_step_deg'] <= 5:
                raise ValueError('诊断步长必须为0.1～5度')
        if not (.1 <= config['default_speed_deg_s'] <= config['max_speed_deg_s'] <= 500):
            raise ValueError('速度配置无效')
        self.status, self.pending, self.synced = {}, {}, set()
        self.seq = {n: 0 for n in self.axes}
        self.sync_tries = {}
        self.owner, self.armed, self.active = None, False, None
        self.lease = 0
        self.last_ping = 0
        self.last_order = {}
        self.stopping = set()
        self.stop_ack = set()
        self.stop_started = None
        self.stop_deadline = None
        self.stop_timeout_reported = False
        self.message = '只接收状态；尚未开启手动控制'
        self.events = deque(maxlen=100)
        self.connected = False
        self.tx_count = 0

    def log(self, message, **extra):
        event = dict(time=time.strftime('%H:%M:%S'), message=message, **extra)
        self.events.append(event)
        self.audit(event)

    def reason(self, n, require_still=True):
        s = self.status.get(n)
        if not self.connected or not s or self.clock() - s['received'] > STATUS_S:
            return '未收到实时状态，请检查驱动板供电和 CAN'
        if s['build'] not in self.axes[n]['builds']:
            return '固件版本未核对，需先确认该版本行程与协议'
        if s['fault'] or s['state'] == 4:
            detail={1:'实际位置越界',2:'轨迹参考越界',3:'轨迹超时',4:'位置跟随误差超限',5:'停车终点越界'}
            label=detail.get(s.get('motion_fault_detail')) if s['fault']==5 else None
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
        if c.get('manual_mode')!='velocity' and not c['minimum'] <= s['position'] <= c['maximum']:
            return '当前位置超出主机行程'
        if require_still and (s['state'] not in (0, 1) or not s['stationary']):
            return '等待轴停稳'
        return ''

    def transmit(self, n, kind, a=0, b=0, syncing=False):
        self.seq[n] = (self.seq[n] + 1) & 0xffffffff
        seq = self.seq[n]
        self.pending[n, seq] = dict(kind=kind, sent=self.clock(), syncing=syncing)
        self.send(n, packet(kind, n, seq, a, b))
        self.tx_count += 1
        if kind != PING:
            self.log(f'J{n} 发送 {COMMAND_NAMES.get(kind,str(kind))}',
                     node=n, kind=kind, sequence=seq, target_mdeg=a, speed_mdeg_s=b)

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
            if kind != 0x40 or result != 0:
                self.fail('状态帧格式不正确')
                return
            flags = (v[9] & 0xffffffff) >> 16
            old = self.status.get(n)
            uptime = v[0] & 0xffffffff
            s = dict(received=now, uptime=uptime, position=v[1] / 1000,
                     target=v[2] / 1000, velocity=v[4] / 1000,
                     state=v[9] & 255, fault=(v[9] >> 8) & 255,
                     build=f'{v[10] & 0xffffffff:08x}')
            if v[13] == 0x43414c31:
                s.update(calibration_extension=True, cal_state=v[3]&255,
                         motion_fault_detail=(v[3]>>24)&255,
                         cal_error=(v[3]>>8)&255, electrical_zero=v[6],
                         encoder_direction=v[7], calibration_saved=bool(v[3]&(1<<17)),
                         calibration_progress=v[11], bus_mv=v[12])
            for name, bit in [('referenced', 2), ('interlocked', 8), ('configured', 16),
                              ('encoder_valid', 32), ('power', 64), ('stationary', 256)]:
                s[name] = bool(flags & bit)
            self.status[n] = s
            if s['fault'] and (not old or old['fault'] != s['fault']):
                self.log(f"J{n} 故障 {s['fault']}：{FAULTS.get(s['fault'], '未知故障')}",
                         node=n, fault=s['fault'], position=s['position'],
                         target=s['target'], velocity=s['velocity'], power=s['power'])
            if old and ((uptime - old['uptime']) & 0xffffffff) >= 0x80000000:
                self.synced.discard(n)
                self.fail(f'J{n} 重启，禁止自动恢复运动')
            elif old and s['build'] != old['build']:
                self.synced.discard(n)
                self.fail(f'J{n} 固件发生变化')
            if n in self.stopping and s['state'] == 4 and not s['power']:
                self.stopping.remove(n)
                self.stop_ack.discard(n)
                self.log(f'J{n} 故障已关断出力；未据此判定机械停稳')
            elif n in self.stopping and n in self.stop_ack and (s['state'] == 0 and not s['power'] or
                                      s['state'] == 1 and s['stationary']):
                self.stopping.remove(n)
                self.stop_ack.discard(n)
                self.log(f'J{n} 反馈确认停止', position=s['position'], power=s['power'])
            return
        pending = self.pending.pop((node, seq), None)
        if pending is None:
            return
        if kind != pending['kind'] | 0x80 or result not in (0, 1):
            self.fail('命令应答格式不正确')
            return
        if pending['syncing']:
            if not self.armed:
                return
            if result:
                # STATUS has no last sequence. Probe with PING only, advancing
                # by a quarter of the uint32 serial space, at most four times.
                self.sync_tries[node] += 1
                if self.sync_tries[node] >= 4:
                    self.fail(f'J{node} PING 序号同步失败')
                    return
                self.seq[node] = (self.seq[node] + 0x3fffffff) & 0xffffffff
                self.transmit(node, PING, syncing=True)
            else:
                self.synced.add(node)
                self.log(f'J{node} 序号同步完成（仅 PING）')
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

    def action(self, action, client='', order=0, **args):
        if action == 'stop':
            # STOP is never suppressed by out-of-order browser requests.
            self.last_order[client] = max(order, self.last_order.get(client, 0))
            self.stop('已请求停止并保持位置', disarm=bool(args.get('disarm', False)))
            return
        if not isinstance(order, int) or order <= self.last_order.get(client, 0):
            raise ValueError('过期的操作已丢弃')
        self.last_order[client] = order
        now = self.clock()
        if action == 'arm':
            if self.active or self.stopping:
                raise ValueError('请先等待停稳')
            if self.owner and self.owner != client and self.armed:
                raise ValueError('另一个页面正在控制')
            ready = [n for n in self.axes if not self.reason(n)]
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
            return
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
        if action != 'jog':
            raise ValueError('不支持的操作')
        if self.active or self.stopping:
            raise ValueError('一次只能操作一个轴，请先松开并等它停稳')
        n, direction, speed = args.get('node'), args.get('direction'), args.get('speed')
        if type(n) is not int or n not in self.axes or direction not in (-1, 1):
            raise ValueError('轴号或方向无效')
        axis_max=min(self.config['max_speed_deg_s'],self.axes[n].get('max_speed_deg_s',30))
        if type(speed) not in (int, float) or not math.isfinite(speed) or not .1 <= speed <= axis_max:
            raise ValueError('速度超出允许范围')
        if speed * self.axes[n]['ratio'] > 1440:
            raise ValueError('电机速度超限')
        reason = self.reason(n)
        if reason:
            raise ValueError(reason)
        if n not in self.synced:
            raise ValueError('等待 PING 序号同步')
        if any(key[0] == n and p['syncing'] for key, p in self.pending.items()):
            raise ValueError('等待序号同步')
        press = args.get('press')
        if not isinstance(press, str) or not 1 <= len(press) <= 80:
            raise ValueError('缺少本次按压标识')
        motor_direction=direction*self.axes[n].get('manual_direction',1)
        if self.axes[n].get('manual_mode')=='velocity':
            self.active=dict(node=n,press=press,speed=speed,direction=direction,motor_direction=motor_direction)
            self.transmit(n,VELOCITY,round(motor_direction*speed*1000))
            self.message=f'J{n} 手动速度运行 {speed:g}°/s；松开快速制动'
            return
        target = self.axes[n]['maximum' if motor_direction == 1 else 'minimum']
        if 'diagnostic_step_deg' in self.axes[n]:
            target = max(self.axes[n]['minimum'], min(self.axes[n]['maximum'],
                         self.status[n]['position']+motor_direction*self.axes[n]['diagnostic_step_deg']))
        if (target - self.status[n]['position']) * motor_direction <= .1:
            raise ValueError('已到该方向的软件行程边界')
        self.active = dict(node=n, press=press, speed=speed, direction=direction,motor_direction=motor_direction)
        self.transmit(n, MOVE, round(target * 1000), round(speed * 1000))
        self.message = f'J{n} 连续运行 {"+" if direction > 0 else "−"} {speed:g}°/s'
        if 'diagnostic_step_deg' in self.axes[n]:
            self.message += f"，本次诊断最多 {self.axes[n]['diagnostic_step_deg']:g}°"

    def stop(self, reason, disarm=False):
        active_node = self.active['node'] if self.active else None
        active_speed = self.active['speed'] if self.active else 0
        self.active = None
        nodes = {n for n in self.synced if n in self.status and self.status[n]['state'] in (1, 2, 3, 5)}
        if active_node is not None:
            s = self.status.get(active_node, {})
            fault_off = (s.get('state') == 4 and not s.get('power') and
                         self.clock()-s.get('received',0) <= STATUS_S)
            if not fault_off:
                nodes.add(active_node)  # MOVE may still be in flight before its first STATUS.
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
            self.stop_started = self.clock()
            brake_time=max(max(abs(self.status.get(n,{}).get('velocity',0)),
                               active_speed if n==active_node else 0)/
                           self.axes[n].get('brake_deceleration_deg_s2',self.axes[n].get('acceleration_deg_s2',30))
                           for n in self.stopping)
            self.stop_deadline=max(self.stop_deadline or 0,self.stop_started+brake_time+2)
            self.stop_timeout_reported = False

    def fail(self, reason):
        if self.armed or self.active:
            self.stop(reason, disarm=True)
        else:
            self.message = reason

    def tick(self):
        now = self.clock()
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
            p = self.pending.pop(key, None)
            if p is not None:
                self.fail(f'J{key[0]} 应答超时；不重发运动命令')
        if self.armed and now - self.last_ping >= .10:
            self.last_ping = now
            for n in sorted(self.synced):
                if not any(key[0] == n for key in self.pending):
                    self.transmit(n, PING)
        if self.stopping and self.stop_deadline is not None and now > self.stop_deadline and not self.stop_timeout_reported:
            self.armed = False
            self.message = '停止尚未获反馈确认，请观察实物；主机已停止 PING'
            self.log(self.message)
            self.stop_timeout_reported = True

    def snapshot(self):
        rows = []
        for n, c in self.axes.items():
            s = self.status.get(n, {})
            age = self.clock() - s['received'] if s else None
            rows.append(dict(node=n, **c, **s, age_ms=round(age * 1000) if age is not None else None,
                             display_position=s.get('position',0)*c.get('manual_direction',1),
                             display_velocity=s.get('velocity',0)*c.get('manual_direction',1),
                             state_text=STATES[s['state']] if s.get('state', 99) < len(STATES) else '未知',
                             reason=self.reason(n), ready=self.armed and n in self.synced and not self.reason(n)))
        return dict(connected=self.connected, armed=self.armed, active=self.active,
                    owner=self.owner,
                    stopping=sorted(self.stopping), message=self.message, axes=rows,
                    tx_count=self.tx_count, events=list(self.events),
                    default_speed=self.config['default_speed_deg_s'], max_speed=self.config['max_speed_deg_s'])
