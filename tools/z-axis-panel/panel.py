"""节点1实机测试面板；手动F6恒速，临时启用失联停机，不自动使能。"""
import ctypes
import json
import os
import queue
import struct
import sys
import threading
import time
import tkinter as tk
from pathlib import Path
from can_diagnostics import LOG_PATH, append_diagnostic, describe_can_error

RPM, HOLD_SECONDS, LEASE_SECONDS = 5, 13.0, 0.2
PREFLIGHT_SECONDS = 0
AUTO_MODE = True
MANUAL_HOLD_SECONDS = 34.0
WATCHDOG_MS = 500
PULSES, MAX_PHASE_MA = 3200, 2800  # 已核对1.8°/16细分；低于已读取的3000mA配置上限。
TRAVEL_DEG = PULSES * 1.8 / 16
SERIAL = "004A00404346570B20363833"
ROOT = Path(__file__).resolve().parents[2]


def parse_rpm(value):
    text = str(value).strip()
    if not text.isascii() or not text.isdigit() or not 1 <= int(text) <= 3000:
        raise ValueError("请输入1～3000之间的整数RPM（Emm指令范围）")
    return int(text)


def motion_command(direction):
    if direction not in (0, 1):
        raise ValueError("方向错误")
    rpm = parse_rpm(RPM)
    return [0xFD, direction, rpm >> 8, rpm & 255, 1, *PULSES.to_bytes(4, "big"), 2, 0, 0x6B]


def speed_command(direction):
    if direction not in (0, 1):
        raise ValueError('方向错误')
    rpm = parse_rpm(RPM)
    return [0xF6, direction, rpm >> 8, rpm & 255, 1, 0, 0x6B]


def watchdog_command():
    return [0x68, 0x38, 0, *WATCHDOG_MS.to_bytes(4, 'big'), 0x6B]


class Intent:
    def __init__(self):
        self.lock = threading.Lock()
        self.direction, self.token, self.lease = None, 0, 0

    def press(self, direction):
        with self.lock:
            self.token += 1
            self.direction, self.lease = direction, time.monotonic() + LEASE_SECONDS

    def release(self):
        with self.lock:
            self.direction = None

    def renew(self):
        with self.lock:
            self.lease = time.monotonic() + LEASE_SECONDS

    def snapshot(self):
        with self.lock:
            return self.direction, self.token, self.lease


class HoldGate:
    """一次按下只启动一次；超时/失焦后持续按住不能自动重启。"""
    def __init__(self):
        self.active, self.used_token, self.started = None, -1, 0

    def step(self, now, snapshot):
        direction, token, lease = snapshot
        live = direction is not None and now < lease
        if self.active is not None:
            if (not live or token != self.used_token or direction != self.active
                    or now - self.started >= HOLD_SECONDS):
                self.active, self.used_token = None, token
                return "stop"
        elif live and token != self.used_token:
            self.active, self.used_token, self.started = direction, token, now
            return "start"
        return "idle"


class AutoSequence:
    """自动测试仍通过同一按住意图和停止路径，取消后永不续跑。"""
    def __init__(self, enabled=False, legs=6):
        self.legs = legs
        self.enabled, self.waiting, self.index, self.due = enabled, False, 0, float("inf")

    def next(self, now):
        if self.enabled and not self.waiting and now >= self.due and self.index < self.legs:
            self.waiting = True
            return self.index % 2
        return None

    def complete(self, now):
        if self.enabled and self.waiting:
            self.index += 1
            self.waiting, self.due = False, now + 1
            if self.index == self.legs:
                self.enabled = False

    def cancel(self):
        self.enabled, self.waiting = False, False


class Drive:
    def __init__(self):
        self.last_command = None
        self.watchdog_ready = False
        self.pending_replies = {}
        # 延迟导入：离线停止逻辑测试不打开USB。
        import usb.core
        import usb.backend.libusb1
        import libusb_package
        from gs_usb.gs_usb import GsUsb
        from gs_usb.gs_usb_frame import GsUsbFrame
        self.Frame = GsUsbFrame
        backend = usb.backend.libusb1.get_backend(find_library=libusb_package.find_library)
        devices = list(usb.core.find(find_all=True, idVendor=0x1D50, idProduct=0x606F, backend=backend))
        if len(devices) != 1 or devices[0].serial_number != SERIAL:
            raise RuntimeError("未找到本次已验证的唯一USB-CAN")
        self.usb, self.can = devices[0], GsUsb(devices[0])
        folder = ROOT / "output/hardware-validation"
        folder.mkdir(parents=True, exist_ok=True)
        self.log = (folder / ("z-panel-" + time.strftime("%Y%m%d-%H%M%S") + ".jsonl")).open("a", encoding="utf-8")
        try:
            self.usb.reset()
            self.can.stop()
            self.usb.ctrl_transfer(0x41, 0, 0, 0, struct.pack("<I", 0xBEEF))
            if not self.can.set_bitrate(500000):
                raise RuntimeError("CAN速率设置失败")
            self.can.device_flags = 0
            self.usb.ctrl_transfer(0x41, 2, 0, 0, struct.pack("<II", 1, 0))
            # 新连接先等待接收静默，旧应答不用于本轮校验；错误帧仍拒绝。
            deadline, quiet = time.monotonic() + 1, time.monotonic()
            while time.monotonic() < deadline:
                frame = self.Frame()
                if self.can.read(frame, 10):
                    quiet = time.monotonic()
                    self.record({"startup_rx": True, "id": frame.can_id, "data": frame.data[:frame.can_dlc]})
                    if frame.can_id & 0x20000000:
                        self.check_error_frame(frame, 'startup_before_new_requests', fatal=False)
                elif time.monotonic() - quiet >= .2:
                    break
            else:
                raise RuntimeError("启动接收持续繁忙，不能确认应答新鲜度")
        except Exception as error:
            append_diagnostic('ERROR', str(error), phase='connect', raw_log=self.log.name)
            self.close()
            raise

    def record(self, item):
        self.log.write(json.dumps({"time": time.time(), **item}, ensure_ascii=False) + "\n")
        self.log.flush()

    def send(self, data):
        reads = [[code, 107] for code in (0x16, 0x1A, 0x3B, 0x3A, 0x24, 0x36, 0x35, 0x27)]
        moves = [motion_command(0), motion_command(1)] if AUTO_MODE else [speed_command(0), speed_command(1)]
        allowed = reads + [[0x42, 0x6C, 107], [0xFE, 0x98, 0, 107], watchdog_command()] + moves
        if data not in allowed:
            raise RuntimeError("指令不在本面板白名单中")
        if data[0] == 0x42:
            self.pending_replies.clear()
        for packet, offset in enumerate(range(1, len(data), 7)):
            payload = [data[0]] + data[offset:offset + 7]
            self.last_command = {'id': '0x%08X' % (0x80000100 | packet), 'data': payload}
            self.can.send(self.Frame(can_id=0x80000100 | packet, data=payload))
            self.record({"tx": True, "id": 0x100 | packet, "data": payload})

    def check_error_frame(self, frame, phase, fatal=True):
        level, message = describe_can_error(frame.can_id, frame.data[:frame.can_dlc])
        details = {'phase': phase, 'last_command': self.last_command, 'raw_log': self.log.name}
        append_diagnostic(level, message, **details)
        self.record({'event': 'can_' + level.lower(), 'value': message, **details})
        if level == 'ERROR' and fatal:
            raise RuntimeError(message)

    def reply(self, code, timeout=0.1, packet=0):
        if (code, packet) in self.pending_replies:
            return self.pending_replies.pop((code, packet))
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            frame = self.Frame()
            if not self.can.read(frame, 5) or frame.echo_id != 0xFFFFFFFF:
                continue
            data = frame.data[:frame.can_dlc]
            self.record({"rx": True, "id": frame.can_id, "data": data})
            if frame.can_id & 0x20000000:
                self.check_error_frame(frame, 'waiting_reply_%02X' % code)
                continue
            if frame.can_id == (0x80000100 | packet) and data and data[0] == code:
                return data
            if code == 0x42 and 0x80000100 <= frame.can_id <= 0x80000104 and data and data[0] == code:
                key = (code, frame.can_id & 255)
                if key in self.pending_replies and self.pending_replies[key] != data:
                    raise RuntimeError('配置回包重复且内容冲突')
                self.pending_replies[key] = data
        message = "等待节点1功能码0x%02X第%d包应答超时（%dms）" % (code, packet, timeout * 1000)
        append_diagnostic('ERROR', message, phase='reply_timeout', last_command=self.last_command, raw_log=self.log.name)
        raise RuntimeError(message)

    def query(self, code, length):
        self.send([code, 107])
        data = self.reply(code)
        if len(data) != length or data[-1] != 107:
            raise RuntimeError("驱动反馈格式错误：%02X" % code)
        return data

    def validate(self):
        if self.query(0x1A, 4) != [26, 0, 6, 107]:
            raise RuntimeError("固件/单位配置已改变；需要重新核对")
        if self.query(0x3B, 3)[1] & 0x33 != 3:
            raise RuntimeError("编码器/校准/温度或电流保护异常")
        if self.query(0x3A, 3)[1] & 13 != 1:
            raise RuntimeError("电机未使能或已堵转；不自动使能/清故障")
        voltage = int.from_bytes(bytes(self.query(0x24, 4)[1:3]), "big") / 1000
        if not 20 < voltage < 27:
            raise RuntimeError("母线电压异常：%.2fV" % voltage)
        self.send([0x42, 0x6C, 107])
        config = [0x42]
        for packet in range(5):
            config += self.reply(0x42, packet=packet)[1:]
        expected = [66, 33, 21, 25, 2, 3, 2, 0, 16, 1, 0, 4, 176, 11, 184,
                    19, 136, 5, 7, 1, 0, 1, 1, 0, 8, 8, 152, 7, 208, 0, 8, 107]
        if config != expected:
            raise RuntimeError("驱动配置与刚才实测不一致；停止并重新核对")
        sample = self.sample()
        if sample[1] != 0:
            raise RuntimeError("电机尚在运动，不能接管点动")
        if not AUTO_MODE:
            self.ensure_watchdog()
        return voltage

    def ensure_watchdog(self):
        self.watchdog_ready = False
        before = int.from_bytes(bytes(self.query(0x16, 6)[1:5]), 'big')
        if before != WATCHDOG_MS:
            self.send(watchdog_command())
            if self.reply(0x68) != [0x68, 2, 107]:
                raise RuntimeError('驱动未确认临时失联停机设置，拒绝F6启动')
            time.sleep(.05)  # 配置命令处理完成后再回读，避免紧接写入争用通讯。
        after = int.from_bytes(bytes(self.query(0x16, 6)[1:5]), 'big')
        self.record({'event': 'watchdog', 'before_ms': before, 'after_ms': after, 'save_to_flash': False})
        if after != WATCHDOG_MS:
            raise RuntimeError('失联停机参数回读不符，拒绝F6启动')
        self.watchdog_ready = True

    def sample(self):
        p, v = self.query(0x36, 7), self.query(0x35, 5)
        if p[1] not in (0, 1) or v[1] not in (0, 1):
            raise RuntimeError("位置/速度符号异常")
        angle = (-1 if p[1] else 1) * int.from_bytes(bytes(p[2:6]), "big") * 360 / 65536
        rpm = (-1 if v[1] else 1) * int.from_bytes(bytes(v[2:4]), "big")
        current = int.from_bytes(bytes(self.query(0x27, 4)[1:3]), "big")
        status = self.query(0x3A, 3)[1]
        return angle, rpm, current, status

    def move(self, direction):
        if not AUTO_MODE and not self.watchdog_ready:
            raise RuntimeError('失联停机尚未核验，拒绝F6启动')
        command = motion_command(direction) if AUTO_MODE else speed_command(direction)
        self.send(command)
        if self.reply(command[0]) != [command[0], 2, 107]:
            raise RuntimeError("运动命令未获确认")

    def stop(self):
        self.send([0xFE, 0x98, 0, 107])
        if self.reply(0xFE) != [0xFE, 2, 107]:
            raise RuntimeError("停止未获确认")
        deadline, stable = time.monotonic() + .75, 0
        while time.monotonic() < deadline:
            stable = stable + 1 if self.sample()[1] == 0 else 0
            if stable >= 3:
                return
            time.sleep(.02)
        raise RuntimeError("停止后未确认连续零转速")

    def close(self):
        import usb.util
        try:
            self.can.stop()  # 只关闭适配器，保持驱动使能，不松开Z轴。
        finally:
            usb.util.dispose_resources(self.usb)
            self.log.close()


def monitor(sample, baseline, direction, elapsed, travel_limit=TRAVEL_DEG, manual=False):
    angle, rpm, current, status = sample
    delta = (angle - baseline) * (1 if direction == 0 else -1)
    if status & 12 or not status & 1:
        raise RuntimeError("堵转/失能，停止测试")
    if current > MAX_PHASE_MA:
        raise RuntimeError("相电流超过本轮2800mA停止阈值，已请求停止")
    if delta < -1.5 or delta > travel_limit + 3:
        raise RuntimeError("方向或位移超出本次范围")
    if manual:
        return  # 展示模式只记录实际转速，不按转速偏差或理想轨迹中止。
    if abs(rpm) > RPM + 3:
        raise RuntimeError("自动轨迹速度超出本次范围")
    # Emm的acc=1：每255*50us增减1RPM；跟随检查包含实际梯形加减速。
    ramp = RPM * 255 * 50e-6
    velocity = RPM * 6
    duration = travel_limit / velocity + ramp
    if elapsed < ramp:
        expected = velocity * elapsed * elapsed / (2 * ramp)
    elif elapsed < duration - ramp:
        expected = velocity * (elapsed - ramp / 2)
    elif elapsed < duration:
        expected = travel_limit - velocity * (duration - elapsed) ** 2 / (2 * ramp)
    else:
        expected = travel_limit
    if travel_limit == TRAVEL_DEG and elapsed > 0.65 and delta < expected - 6:
        raise RuntimeError("转动跟随不足，可能遇到限位或负载阻力")


def worker(intent, closing, events, settings=None, recovery=False):
    global RPM
    drive, moving = None, False
    gate = HoldGate()
    def emit(kind, value):
        events.put((kind, value))
        if kind in ('fault', 'ready', 'stopped', 'status', 'settings_applied'):
            append_diagnostic('ERROR' if kind == 'fault' else 'INFO', value, event=kind, node=1, target_rpm=RPM)
        if drive:
            drive.record({"event": kind, "value": value})
    try:
        drive = Drive()
        if recovery:
            drive.stop()
            emit('status', '重新连接后已确认停止，正在复核驱动配置')
        voltage = drive.validate()
        deadline = time.monotonic() + PREFLIGHT_SECONDS
        while time.monotonic() < deadline and not closing.is_set():
            sample = drive.sample()
            if sample[1] or sample[3] & 13 != 1:
                raise RuntimeError("启动前状态异常")
            closing.wait(.1)
        if closing.is_set():
            return
        emit("ready", "已连接节点1 · %.2fV · Emm闭环" % voltage)
        last_check = time.monotonic()
        while not closing.is_set():
            if settings is not None and not settings.empty():
                proposed = settings.get_nowait()
                if moving or intent.snapshot()[0] is not None or AUTO_MODE:
                    emit("settings_rejected", "先停止运动，再应用转速；自动序列运行中不改参数")
                else:
                    new_rpm = parse_rpm(proposed)
                    drive.validate()
                    RPM = new_rpm
                    emit("settings_applied", RPM)
            action = gate.step(time.monotonic(), intent.snapshot())
            if action == "start":
                drive.validate()
                baseline = drive.sample()[0]
                # 校验过程可能耗时；松开按钮后不能迟到启动。
                direction, token, lease = intent.snapshot()
                if closing.is_set() or direction != gate.active or token != gate.used_token or time.monotonic() >= lease:
                    gate.active = None
                    continue
                moving = True  # 分包/ACK异常也必须走停止路径。
                drive.move(direction)
                gate.started = time.monotonic()
                settled = 0
                emit("leg_start", {"direction": direction, "baseline_deg": baseline, "target_rpm": RPM,
                                   "mode": 'FD_position' if AUTO_MODE else 'F6_speed'})
                emit("status", "方向%s运行中" % ("A" if direction == 0 else "B"))
            elif action == "stop":
                drive.stop()
                moving = False
                emit("stopped", "已停止并保持使能；再次运行需松开后重新按")
            if moving:
                sample = drive.sample()
                emit("sample", sample)
                travel_limit = TRAVEL_DEG if AUTO_MODE else RPM * 6 * HOLD_SECONDS + TRAVEL_DEG
                monitor(sample, baseline, gate.active, time.monotonic() - gate.started,
                        travel_limit, manual=not AUTO_MODE)
                # F6只在按下时发送一次；这里的查询不重建运动轨迹，同时维持驱动心跳。
                settled = settled + 1 if abs(abs(sample[0] - baseline) - TRAVEL_DEG) < 1 and sample[1] == 0 and sample[3] & 2 else 0
                if AUTO_MODE and settled >= 3:
                    drive.stop()
                    moving, gate.active = False, None
                    emit("leg_complete", {"direction": direction, "delta_deg": sample[0] - baseline})
                    emit("stopped", "有限行程完成，已停止并保持使能")
            elif time.monotonic() - last_check > 0.5:
                sample = drive.sample()
                if sample[1] or sample[3] & 13 != 1:
                    raise RuntimeError("待机状态变化，请检查驱动")
                emit("sample", sample)
                last_check = time.monotonic()
            closing.wait(0.1)
    except Exception as error:
        emit("fault", str(error))
    finally:
        intent.release()
        if drive:
            try:
                if moving:
                    drive.stop()
                    emit("status", "故障后已确认停止，保持使能")
            except Exception as error:
                boundary = '已有位置指令最多1圈' if AUTO_MODE else '已核对500ms驱动失联停机'
                emit("fault", "停止未确认：%s。请现场核对；%s。" % (error, boundary))
            finally:
                drive.close()
        events.put(("closed", None))


def main():
    global RPM, HOLD_SECONDS, PREFLIGHT_SECONDS, AUTO_MODE
    requested_rpm = parse_rpm(next((a.split('=', 1)[1] for a in sys.argv if a.startswith('--rpm=')), '50'))
    cycles = int(next((a.split('=', 1)[1] for a in sys.argv if a.startswith('--cycles=')), '3'))
    if not 1 <= cycles <= 30:
        raise ValueError("自动测试轮数必须为1至30")
    RPM = requested_rpm
    AUTO_MODE = '--auto-test' in sys.argv or '--direction-check' in sys.argv
    if AUTO_MODE and RPM > 50:
        raise ValueError("现有自动测试轨迹只验证到50RPM；更高目标请在手动面板分档测试")
    HOLD_SECONDS = (TRAVEL_DEG / (RPM * 6) + RPM * 255 * 50e-6 + 3) if AUTO_MODE else MANUAL_HOLD_SECONDS
    PREFLIGHT_SECONDS = 5
    # 本面板只允许一个进程占用已核对的USB-CAN。
    kernel = ctypes.WinDLL("kernel32", use_last_error=True)
    kernel.CreateMutexW.restype = ctypes.c_void_p
    kernel.CreateMutexW.argtypes = [ctypes.c_void_p, ctypes.c_bool, ctypes.c_wchar_p]
    mutex = kernel.CreateMutexW(None, False, "Local\\SCARA_Z_TEST_PANEL")
    if not mutex or ctypes.get_last_error() == 183:
        raise RuntimeError("Z轴面板已经打开，请使用已有窗口")
    root = tk.Tk()
    direction_check = "--direction-check" in sys.argv
    auto = AutoSequence("--auto-test" in sys.argv or direction_check, legs=1 if direction_check else cycles * 2)
    auto_requested = auto.enabled
    auto_report = ROOT / "output/hardware-validation/z-auto-latest.json"
    def report(outcome):
        if auto_requested:
            data = {"time": time.time(), "outcome": outcome, "completed_legs": auto.index,
                    "planned_legs": auto.legs, "rpm": RPM, "leg_motor_degrees": TRAVEL_DEG}
            temp = auto_report.with_suffix(".tmp")
            temp.write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding="utf-8")
            temp.replace(auto_report)
    report("connecting")
    root.title("SCARA · Joint1 Z轴恒速测试")
    root.geometry("740x790")
    root.resizable(False, False)
    root.configure(bg="#f3f5f7")
    intent, closing, events, settings = Intent(), threading.Event(), queue.Queue(), queue.Queue()
    state = {"ready": False, "held": False, "fault": False, "thread": None,
             "running": False, "last_speed": None, "peak": 0}
    status = tk.StringVar(value="正在连接并核对驱动……")
    feedback = tk.StringVar(value="仅读取状态；按住方向按钮才会运动")
    previous_fault = tk.StringVar(value="故障详情会保留在下方及故障日志中")
    tk.Label(root, text="Z轴 · 自动往返 / 随时停止" if auto_requested else "Z轴 · 恒速性能测试", font=("Microsoft YaHei UI", 22, "bold"), bg="#f3f5f7").pack(pady=(22, 4))
    target_label = tk.StringVar(value="已应用目标 %d RPM  |  按住最多34秒" % RPM)
    tk.Label(root, textvariable=target_label, font=("Microsoft YaHei UI", 12), bg="#f3f5f7").pack()
    tk.Label(root, text="已确认A上升 / B下降；尚未建立Z软限位", bg="#f3f5f7", fg="#6c4a00").pack(pady=8)
    speed_row = tk.Frame(root, bg="#f3f5f7")
    speed_row.pack(pady=7)
    tk.Label(speed_row, text="目标转速 RPM", bg="#f3f5f7", font=("Microsoft YaHei UI", 12)).pack(side="left", padx=8)
    speed_input = tk.StringVar(value=str(RPM))
    speed_entry = tk.Spinbox(speed_row, from_=1, to=3000, textvariable=speed_input, width=8, font=("Microsoft YaHei UI", 16))
    speed_entry.pack(side="left", padx=8)

    def apply_speed():
        try:
            proposed = parse_rpm(speed_input.get())
        except ValueError as error:
            status.set(str(error))
            return
        if not state["ready"] or state["held"] or state["running"] or state["last_speed"] != 0 or auto.enabled:
            status.set("请先停止并确认连接正常，再应用转速")
            return
        state["ready"] = False
        settings.put(proposed)
        status.set("正在复核静止状态；应用参数不会启动电机")

    apply_button = tk.Button(speed_row, text="应用转速", command=apply_speed, font=("Microsoft YaHei UI", 12), state="disabled")
    apply_button.pack(side="left", padx=8)
    tk.Label(root, text="1～3000为协议输入范围，不代表实测带载上限。", bg="#f3f5f7", fg="#6c4a00").pack()
    tk.Label(root, text="按下仅发送一次F6速度指令；松开/急停/34秒上限停止。", bg="#f3f5f7", fg="#555555").pack(pady=4)
    tk.Label(root, text="闭环保留 · 临时500ms失联停机 · 周期查询不重发运动目标", bg="#f3f5f7", fg="#555555").pack()
    tk.Label(root, text="手动模式：转速偏差仅显示，不触发速度/跟随停机。", bg="#f3f5f7", fg="#17416b").pack()
    buttons = tk.Frame(root, bg="#f3f5f7")
    buttons.pack(pady=12)

    def release(event=None):
        if auto.enabled:
            auto.cancel()
            report("cancel_requested")
        state["held"] = False
        intent.release()

    def press(direction):
        if state["ready"] and not state["held"] and root.focus_displayof() is not None:
            state["held"] = True
            intent.press(direction)

    def manual_release(event=None):
        # 拖动自动测试窗口或经过禁用的手动按钮，不应误判为用户急停。
        if state["held"]:
            release()

    controls = []
    for direction, label in [(0, "上升 A\n按住运行"), (1, "下降 B\n按住运行")]:
        button = tk.Button(buttons, text=label, width=16, height=3, font=("Microsoft YaHei UI", 15), state="disabled")
        button.pack(side="left", padx=10)
        button.bind("<ButtonPress-1>", lambda event, d=direction: press(d))
        button.bind("<ButtonRelease-1>", manual_release)
        button.bind("<Leave>", manual_release)
        controls.append(button)
    tk.Button(root, text="急停（CAN停止） / 空格键", command=release, bg="#b3261e", fg="white", font=("Microsoft YaHei UI", 14), width=34).pack(pady=5)
    tk.Label(root, textvariable=feedback, font=("Microsoft YaHei UI", 11), bg="#f3f5f7").pack(pady=8)
    tk.Label(root, textvariable=status, wraplength=560, bg="#f3f5f7", fg="#17416b").pack(pady=4)
    diagnostic_row = tk.Frame(root, bg='#f3f5f7')
    diagnostic_row.pack(pady=4)

    def reconnect():
        if closing.is_set() or state['thread'].is_alive():
            status.set('旧任务尚未结束，请等待停止处理完成')
            return
        release()
        auto.cancel()
        for pending in (events, settings):
            while not pending.empty():
                pending.get_nowait()
        state.update(ready=False, fault=False, running=False, held=False, last_speed=None)
        status.set('正在清除面板锁定并重新连接；不会续跑旧动作')
        append_diagnostic('INFO', '用户请求清除面板锁定/重新连接', target_rpm=RPM)
        state['thread'] = threading.Thread(target=worker, args=(intent, closing, events, settings, True), daemon=False)
        state['thread'].start()

    def open_failure_log():
        append_diagnostic('INFO', '用户打开故障日志')
        os.startfile(str(LOG_PATH))

    reconnect_button = tk.Button(diagnostic_row, text='清除面板锁定 / 重新连接', command=reconnect, state='disabled')
    reconnect_button.pack(side='left', padx=8)
    tk.Button(diagnostic_row, text='打开故障日志', command=open_failure_log).pack(side='left', padx=8)
    tk.Label(root, textvariable=previous_fault, wraplength=700, bg='#f3f5f7', fg='#8b2525', justify='left').pack(pady=4)
    tk.Label(root, text="自动：急停/空格/Esc/关闭取消整轮。手动：松开或失焦停止。", bg="#f3f5f7", fg="#555555").pack(pady=8)
    root.bind_all("<ButtonRelease-1>", manual_release)
    root.bind("<space>", release)
    root.bind("<Escape>", release)
    root.bind("<FocusOut>", lambda event: root.after(10, lambda: release() if not auto.enabled and root.focus_displayof() is None else None))

    def close():
        release()
        state["ready"] = False
        closing.set()
        status.set("正在关闭；等待停止检查……")

    def tick():
        # 即使在窗口外松开，也能撤销运行；UI卡住时工作线程租约会过期。
        if state["held"]:
            if not ctypes.windll.user32.GetAsyncKeyState(1) & 0x8000 or root.focus_displayof() is None:
                release()
            else:
                intent.renew()
        while not events.empty():
            kind, value = events.get_nowait()
            if kind == "ready" and not closing.is_set():
                state["ready"] = True
                state["last_speed"] = 0
                status.set(value)
                if previous_fault.get().startswith('最近故障：'):
                    previous_fault.set('历史故障（当前已连接）：' + previous_fault.get()[5:])
                if auto.enabled:
                    auto.due = time.monotonic() + 5
                    status.set("5秒后开始%d段测试；红色急停/空格随时取消" % auto.legs)
                    report("countdown")
            elif kind == "fault":
                release()
                state["ready"], state["fault"] = False, True
                status.set('已锁定；可查看故障日志，处理后点击“清除面板锁定 / 重新连接”')
                previous_fault.set('最近故障：' + value)
                report("fault: " + value)
            elif kind == "settings_applied":
                state["ready"], state["peak"] = True, 0
                target_label.set("已应用目标 %d RPM  |  按住最多34秒" % value)
                status.set("转速已应用；按住上升或下降才会运动")
            elif kind == "settings_rejected":
                state["ready"] = not state["fault"]
                status.set(value)
            elif kind == "leg_start":
                state["running"], state["peak"] = True, 0
            elif kind == "sample":
                state["last_speed"] = value[1]
                if state["running"]:
                    state["peak"] = max(state["peak"], abs(value[1]))
                feedback.set("实测 %d RPM  |  本次采样峰值 %d RPM\n电机角度 %.2f°  |  相电流 %d mA" % (value[1], state["peak"], value[0], value[2]))
            elif kind in ("status", "stopped") and not state["fault"]:
                status.set(value)
                if kind == "stopped":
                    state["running"] = False
                    state["held"] = False
                    intent.release()
                    if auto.enabled and auto.waiting:
                        auto.cancel()
                        report("stopped_before_target")
                    elif auto_requested and auto.index == auto.legs:
                        status.set("本轮自动测试完成，已停止并保持使能")
                        report("complete")
            elif kind == "leg_complete":
                auto.complete(time.monotonic())
                report("leg_complete")
            elif kind == "closed":
                state["ready"] = False
        for button in controls:
            button.configure(state="normal" if state["ready"] and not auto.enabled else "disabled")
        can_set_speed = state["ready"] and not state["running"] and not state["held"] and not auto.enabled
        apply_button.configure(state="normal" if can_set_speed else "disabled")
        can_reconnect = state['fault'] and not state['thread'].is_alive() and not closing.is_set()
        reconnect_button.configure(state='normal' if can_reconnect else 'disabled')
        if auto.enabled and state["ready"] and not closing.is_set():
            direction = auto.next(time.monotonic())
            if direction is not None:
                intent.press(direction)
                report("running_leg_%d" % (auto.index + 1))
            if auto.waiting:
                intent.renew()
        if closing.is_set() and not state["thread"].is_alive():
            root.destroy()
            return
        root.after(30, tick)

    root.protocol("WM_DELETE_WINDOW", close)
    if auto_requested:
        root.attributes("-topmost", True)
        root.lift()
        root.after(100, root.focus_force)
    state["thread"] = threading.Thread(target=worker, args=(intent, closing, events, settings), daemon=False)
    state["thread"].start()
    root.after(30, tick)
    try:
        root.mainloop()
    finally:
        intent.release()
        closing.set()
        state["thread"].join()
        kernel.CloseHandle(ctypes.c_void_p(mutex))


if __name__ == "__main__":
    main()
