"""Field I/O for the existing C++ game, with physical USB-CAN and RV1106."""

import ctypes
from http.client import HTTPConnection
import json
import math
import os
from pathlib import Path
import queue
import re
import shlex
import shutil
import signal
import socket
import subprocess
import sys
import threading
import time

from drive import Drive, PROFILE, ROOT, SET_SPEED_GAINS, WORK

BENCH = Path(__file__).resolve().parent
ADB = (Path(os.environ['LOCALAPPDATA']) / 'Packages' / 'OpenAI.Codex_2p2nqsd0c76g0' /
       'LocalCache' / 'Local' / 'arm4-tools' / 'platform-tools' / 'adb.exe')
DEVICE = 'abc8f01242c327ec'
STREAM = 'rtsp://192.168.31.24/live/0'
ENGINE = '/userdata/scara-live-match-20261117/engine'
MODEL = '/userdata/scara-yolo-v3/xiangqi14-local-v3-rv1106.rknn'
MODEL_SHA = 'ce9c91f4668cb0ec58f836432546b7eb2aebfc675ba696ce9788968560dcbdd2'
UCI = re.compile(r'[a-i][0-9][a-i][0-9]')
START_BOARD = ['rnbakabnr', '.........', '.c.....c.', 'p.p.p.p.p', '.........',
               '.........', 'P.P.P.P.P', '.C.....C.', '.........', 'RNBAKABNR']

NPU_PORT = 18188
PICK_PULSE_MS = 170
GAME_Z_SPEED = PROFILE.get('game_speed_deg_s', 360)

BOARD_BOOTSTRAP = '''
set -e
test -c /dev/rknpu || insmod /oem/usr/ko/rknpu.ko
test -c /dev/rknpu
for n in 40 35; do
    p=/sys/class/gpio/gpio$n
    test -d "$p" || echo "$n" > /sys/class/gpio/export
    echo low > "$p/direction"
    test "$(cat "$p/direction")" = out
    test "$(cat "$p/value")" = 0
done
echo SCARA_IO_READY
'''

BRIDGE_SCRIPT = '''
from pathlib import Path
import signal
import socket
import time
pins = [Path('/sys/class/gpio/gpio%d/value' % n) for n in (40, 35)]
def outputs(a, b):
    for pin, value in zip(pins, (a, b)):
        pin.write_text(value)
    if tuple(p.read_text().strip() for p in pins) != (a, b):
        raise RuntimeError('H-bridge GPIO feedback mismatch')
def stop(*_):
    raise SystemExit(0)
for event in (signal.SIGHUP, signal.SIGINT, signal.SIGTERM):
    signal.signal(event, stop)
try:
    outputs('0', '0')
    with socket.socket() as listener:
        listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        listener.bind(('127.0.0.1', 18189))
        listener.listen(1)
        listener.settimeout(5)
        print('BRIDGE_READY', flush=True)
        connection, _ = listener.accept()
        with connection, connection.makefile('rb') as commands:
            connection.settimeout(.75)
            connection.sendall(b'BRIDGE_OK\\n')
            for line in commands:
                request = line.strip().decode('ascii')
                if request == 'keep':
                    continue
                command, _, duration = request.partition(' ')
                if command in ('attract', 'release'):
                    ms = int(duration)
                    if not 0 < ms <= (250 if command == 'attract' else 50):
                        raise ValueError('Unexpected magnet pulse duration')
                    outputs('0', '0')
                    time.sleep(.005)
                    try:
                        levels = ('1', '0') if command == 'attract' else ('0', '1')
                        outputs(*levels)
                        time.sleep(ms / 1000)
                    finally:
                        outputs('0', '0')
                elif request == 'off':
                    outputs('0', '0')
                else:
                    break
                connection.sendall(b'BRIDGE_OK\\n')
finally:
    outputs('0', '0')
'''


def remote_python(script, *args):
    return 'exec env PYTHONDONTWRITEBYTECODE=1 python3 -u -c ' + ' '.join(map(shlex.quote, (script, *args)))


def adb_command(*args):
    return subprocess.check_output([str(ADB), '-s', DEVICE, *args], text=True, timeout=4,
                                   creationflags=subprocess.CREATE_NO_WINDOW).strip()


class Lines:
    def __init__(self, command):
        self.lines = queue.Queue()
        self.process = subprocess.Popen([str(ADB), '-s', DEVICE, 'shell', command], stdin=subprocess.PIPE,
                                        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, encoding='utf-8',
                                        errors='replace', bufsize=1, creationflags=subprocess.CREATE_NO_WINDOW)
        self.reader = threading.Thread(target=self.receive, daemon=True)
        self.reader.start()

    def receive(self):
        with self.process.stdout as stream:
            for line in stream:
                self.lines.put(line.strip())
        self.lines.put(None)

    def send(self, line):
        self.process.stdin.write(line + '\n')
        self.process.stdin.flush()

    def read(self, match, seconds, poll=None):
        deadline = time.monotonic() + seconds
        last = ''
        while (remaining := deadline - time.monotonic()) > 0:
            if poll:
                poll()
            try:
                line = self.lines.get(timeout=min(.025, remaining) if poll else remaining)
            except queue.Empty:
                continue
            if line is None:
                raise RuntimeError(f'板端进程退出 {self.process.poll()}，{last}')
            result = match(line)
            if result is not None:
                return result
            last = line[:200]
        raise TimeoutError(f'板端响应超时，{last}')

    def expect(self, token, seconds, poll=None):
        return self.read(lambda line: line if line == token else None, seconds, poll)

    def close(self):
        try:
            self.process.stdin.close()
        except OSError:
            pass
        try:
            return self.process.wait(timeout=2)
        except subprocess.TimeoutExpired:
            self.process.terminate()
            return self.process.wait(timeout=2)


class Magnet:
    def __init__(self, poll):
        self.process = Lines(remote_python(BRIDGE_SCRIPT))
        self.port = None
        self.socket = None
        self.reply = b''
        try:
            self.process.expect('BRIDGE_READY', 4, poll)
            self.port = int(adb_command('forward', 'tcp:0', 'tcp:18189'))
            self.socket = socket.create_connection(('127.0.0.1', self.port), timeout=.4)
            self.socket.settimeout(.02)
            self.expect(poll)
        except BaseException:
            self.close()
            raise

    def send(self, command):
        self.socket.sendall(command.encode('ascii') + b'\n')

    def expect(self, poll):
        deadline = time.monotonic() + .5
        while b'\n' not in self.reply and time.monotonic() < deadline:
            poll()
            try:
                data = self.socket.recv(256)
            except TimeoutError:
                continue
            if not data:
                raise RuntimeError('H 桥连接已关闭')
            self.reply += data
        result, newline, self.reply = self.reply.partition(b'\n')
        if not newline or result != b'BRIDGE_OK':
            raise RuntimeError('H 桥动作回读失败')

    def close(self):
        if self.socket is not None:
            self.socket.close()
            self.socket = None
        try:
            self.process.close()
        finally:
            if self.port is not None:
                adb_command('forward', '--remove', f'tcp:{self.port}')
                self.port = None


def write_json(name, data):
    path = Path(name)
    pending = path.with_name(f'{path.name}.{os.getpid()}.tmp')
    try:
        pending.write_text(json.dumps(data, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
        pending.replace(path)
    finally:
        pending.unlink(missing_ok=True)


def read_game_state(path):
    # OpenCV FileStorage escapes apostrophes in diagnostic strings.
    return json.loads(Path(path).read_text(encoding='utf-8').replace("\\'", "'"))


def configure():
    import numpy as np
    layout = json.loads((WORK / 'layout-candidate.json').read_text(encoding='utf-8'))
    arm = json.loads((ROOT / 'output/xy-registration-20261111/arm-registration.json').read_text(encoding='utf-8'))
    heights = json.loads((ROOT / 'output/first-pick-c3-c4-20261111/trial-plan.json').read_text(encoding='utf-8'))
    park = json.loads((WORK / 'usb-game-park-readback.json').read_text(encoding='utf-8'))
    points = np.array(layout['points']['data']).reshape(3, 4)
    shoulder = arm['shoulder_ref_rad'] + math.radians((-58.539 - arm['j2_reference_deg']) * arm['j2_scale'])
    elbow = arm['elbow_ref_rad'] + math.radians((-86.068 - arm['j3_reference_motor_deg']) *
                                                arm['j3_sign'] / arm['j3_ratio'])
    upper, forearm = arm['arm_lengths_mm']
    taught = upper * np.array([math.cos(shoulder), math.sin(shoulder)])
    taught += forearm * np.array([math.cos(shoulder + elbow), math.sin(shoulder + elbow)])
    initial = points[0, 2:] + (points[1, 2:] - points[0, 2:]) / 2 + (points[2, 2:] - points[0, 2:]) / 9
    correction = taught - initial
    points[:, 2:] += correction
    pixel_to_base = np.linalg.solve(np.column_stack((points[:, :2], np.ones(3))), points[:, 2:])
    # The empty desktop below the outermost i-file, outside the model ROI.
    capture_pixel = [530, 701]
    capture_target = [*capture_pixel, 1] @ pixel_to_base
    if not abs(upper - forearm) < np.linalg.norm(capture_target) < upper + forearm:
        raise RuntimeError('吃子暂存区不在机械臂工作范围内')
    layout['points']['data'] = points.ravel().tolist()
    layout['e1_translation_mm'] = correction.tolist()
    layout['source'] = 'repositioned board pixels and the actual e1 encoder alignment'
    z = heights['z']
    original_rise = z['operator_estimated_rise_mm']
    degrees_per_mm = (z['travel_deg'] - z['pickup_deg']) / original_rise
    z['travel_deg'] += 10 * degrees_per_mm
    z['clearance_deg'] = z['travel_deg']
    z['operator_estimated_rise_mm'] = original_rise + 10
    write_json('layout-live.json', layout)
    write_json('arm-live.json', arm)
    write_json('heights-live.json', {'z': heights['z'], 'release_pulse_ms': heights['release_pulse_ms']})
    write_json('game-live.json', {'layout_file': 'layout-live.json', 'arm_file': 'arm-live.json',
                                 'heights_file': 'heights-live.json', 'camera_url': STREAM,
                                 'label_image': 'labels.png', 'work_directory': 'game-state',
                                 'engine_directory': ENGINE, 'human_red': 1, 'red_to_move': 1, 'think_ms': 650,
                                 'joint_velocity_deg_s': [GAME_Z_SPEED, 22, 30],
                                 'joint_acceleration_deg_s2': [PROFILE['acceleration_deg_s2'], 30, 30],
                                 'park_joints_deg': [park[n]['position'] for n in ('2', '3')],
                                 'capture_place_mm': capture_target.tolist()})
    shutil.copyfile(ROOT / 'config/vision/chinese-labels.png', 'labels.png')
    print(f'e1 坐标平移校正 {correction.tolist()} mm。吃子暂存区 {capture_target.tolist()} mm。', flush=True)


class Bench:
    def __init__(self, resume=False, current_position=False, checkpoint=None):
        self.config = json.loads(Path('game-live.json').read_text(encoding='utf-8'))
        # C++ 与 USB-CAN 从同一份现场参数取得 J1 速度和起动加速度。
        velocity = self.config['joint_velocity_deg_s']
        acceleration = self.config['joint_acceleration_deg_s2']
        if (velocity[0], acceleration[0]) != (GAME_Z_SPEED, PROFILE['acceleration_deg_s2']):
            velocity[0] = GAME_Z_SPEED
            acceleration[0] = PROFILE['acceleration_deg_s2']
            write_json('game-live.json', self.config)
        self.heights = json.loads(Path('heights-live.json').read_text(encoding='utf-8'))
        self.npu = None
        self.npu_pid = None
        self.local_port = None
        self.pikafish = None
        self.drive = None
        self.bridge = None
        self.last_keep = 0.0
        self.start_checks = 0
        self.confirmed_stopped = True
        self.current_position = current_position
        self.saved = checkpoint if checkpoint is not None else (read_game_state('resume.json') if resume else None)
        self.expected = self.saved['board'] if self.saved else START_BOARD

    def resume_history(self):
        return self.saved['history'] if self.saved else ''

    def resume_pending(self):
        return 'b7e7' if 'continue-b7' in sys.argv[1:] else ''

    def expect_board(self, rows):
        self.expected = rows

    def open(self, model_path):
        if model_path != MODEL:
            raise RuntimeError('板端模型路径与本次已核对模型不一致')
        #板重启后重新挂载NPU及H桥控制脚，双路保持低电平。
        if 'SCARA_IO_READY' not in adb_command('shell', BOARD_BOOTSTRAP).splitlines():
            raise RuntimeError('板端NPU或电磁铁控制脚初始化失败')
        # KeyboardInterrupt is allowed to unwind any Python hardware call before returning to C++.
        signal.signal(signal.SIGINT, signal.default_int_handler)

    def infer(self, data, frame):
        try:
            if self.npu is None:
                self.npu = Lines('echo NPU_PID=$$; exec python3 -u /userdata/scara-yolo-v3/board_npu.py '
                                 f'--model {MODEL} --model-sha256 {MODEL_SHA} --bind 127.0.0.1 '
                                 f'--client 127.0.0.1 --port {NPU_PORT}')
                self.npu_pid = self.npu.read(lambda line: int(line[8:]) if line.startswith('NPU_PID=') else None, 4)
                self.npu.expect(f'READY http://127.0.0.1:{NPU_PORT}/', 10)
                self.local_port = int(adb_command('forward', 'tcp:0', f'tcp:{NPU_PORT}'))
            connection = HTTPConnection('127.0.0.1', self.local_port, timeout=5)
            try:
                connection.request('POST', '/infer', body=data, headers={'X-Frame-ID': frame})
                response = connection.getresponse()
                packet = response.read()
                if response.status != 200:
                    raise RuntimeError(f'RV1106 推理端口 {response.status}：{packet[:120]!r}')
                return packet
            finally:
                connection.close()
        except BaseException:
            self.close_vision()
            raise

    def close_vision(self):
        if self.npu is not None:
            npu, self.npu = self.npu, None
            pid, self.npu_pid = self.npu_pid, None
            local_port, self.local_port = self.local_port, None
            try:
                if npu.process.poll() is None and pid:
                    adb_command('shell', f'kill -TERM {pid}')
                npu.close()
            finally:
                if local_port is not None:
                    adb_command('forward', '--remove', f'tcp:{local_port}')

    def engine(self, fen, history, search):
        try:
            if self.pikafish is None:
                self.pikafish = Lines('cd ' + shlex.quote(ENGINE) + ' && ulimit -v 140000 && exec ./pikafish')
                self.pikafish.send('uci')
                self.pikafish.expect('uciok', 8)
                self.pikafish.send('setoption name Threads value 1')
                self.pikafish.send('setoption name Hash value 4')
                self.pikafish.send('isready')
                self.pikafish.expect('readyok', 8)
            engine = self.pikafish
            position = 'position fen ' + fen + (' moves ' + history if history else '')
            engine.send(position)
            engine.send('go perft 1')
            legal = []

            def collect(line):
                fields = line.split(': ', 1)
                if len(fields) == 2 and UCI.fullmatch(fields[0]) and fields[1] == '1':
                    legal.append(fields[0])
                if line.startswith('Nodes searched: '):
                    return int(fields[1])
                return None

            count = engine.read(collect, 5)
            if count != len(legal) or count != len(set(legal)):
                raise RuntimeError('Pikafish 合法走法列表不完整')
            best = ''
            if search and legal:
                engine.send(position)
                engine.send(f"go movetime {self.config['think_ms']}")
                best = engine.read(lambda line: line.split()[1] if line.startswith('bestmove ') else None, 8)
                if best not in legal:
                    raise RuntimeError('Pikafish 应手不在合法表中')
            print(f'Pikafish {count} 个合法走法' + (f'，应手 {best}' if best else ''), flush=True)
            return legal, best
        except BaseException:
            self.close_engine()
            raise

    def close_engine(self):
        if self.pikafish is not None:
            engine, self.pikafish = self.pikafish, None
            try:
                if engine.process.poll() is None:
                    engine.send('quit')
            except OSError:
                pass
            finally:
                engine.close()

    def command(self, phase, prompt):
        if phase == 'prepare':
            return ''
        if phase == 'confirm_start':
            if self.current_position:
                print('CONFIRM_START 核对上位机盘面与实体棋盘一致，再确认开局。', flush=True)
                return input().strip().lower()
            observed = read_game_state('game-state/status.json')['observed']
            self.start_checks += 1
            expected = self.expected
            if observed == expected:
                print('BOARD_CONFIRMED ' + (self.resume_history() or 'opening 32'), flush=True)
                return ''
            changed = [f'{chr(97 + c)}{9-r} {piece}->{observed[r][c]}'
                       for r, row in enumerate(expected) for c, piece in enumerate(row) if observed[r][c] != piece]
            print('OPENING_DIFFERENCE ' + ' '.join(changed), flush=True)
            if self.start_checks >= 6:
                raise RuntimeError('实拍开局盘面仍有差异：' + ' '.join(changed))
            return 'draw'  # Match_start reads another observation for any non-Ready result.
        if phase == 'human':
            print('HUMAN_TURN ' + prompt, flush=True)
        else:
            print(phase.upper() + ' ' + prompt, flush=True)
        try:
            return input().strip().lower()
        except EOFError:
            return 'quit'

    def poll_bridge(self):
        if self.bridge is not None and time.monotonic() - self.last_keep >= .12:
            self.bridge.send('keep')
            self.last_keep = time.monotonic()

    def bridge_command(self, action):
        self.bridge.send(action)
        self.bridge.expect(self.drive.poll)
        self.drive.note('magnet_confirmed', action=action)

    def wait_magnet_height(self, height, travel):
        def at_height():
            a = self.drive.states[1]
            offset = a['position'] - height
            return all(s['stationary'] for s in self.drive.states.values()) and (
                abs(offset) <= 4.5 or (height == travel and 0 < offset <= 12))
        self.drive.wait(at_height, .35)

    def magnet_pulse(self, action, height, travel):
        self.wait_magnet_height(height, travel)
        self.bridge_command(action)

    def board_release(self, height, travel, z, speed, duration):
        self.wait_magnet_height(height, travel)
        start = self.drive.states[1]['position']
        degrees_per_mm = (travel - z['pickup_deg']) / z['operator_estimated_rise_mm']
        trigger = start + z.get('board_release_lift_mm', .7) * degrees_per_mm
        latest = trigger + .65 * degrees_per_mm
        pulse = z.get('board_release_pulse_ms', duration)
        sent = False

        def release_while_rising(a):
            nonlocal sent
            if sent or a['position'] < trigger:
                return
            if (a['position'] > latest or not all(self.drive.states[n]['stationary'] for n in (2, 3))):
                raise RuntimeError('未在放棋高度内收到垂直抬升反馈')
            self.bridge_command(f'release {pulse}')
            self.drive.note('board_release_on_rise', height=a['position'], speed=a['velocity'], pulse_ms=pulse)
            sent = True

        self.drive.z(travel, high=travel, speed_limit=speed, on_rise=release_while_rising)
        if not sent:
            raise RuntimeError('本次未在指定抬升段放棋')

    def execute(self, purpose, move, steps):
        started = time.monotonic()
        print(f'PHYSICAL {purpose} {move} {len(steps)} steps', flush=True)
        self.drive = Drive()
        self.confirmed_stopped = False
        z = self.heights['z']
        travel, pickup = z['travel_deg'], z['pickup_deg']
        release = pickup + (travel - pickup) / z['operator_estimated_rise_mm'] * z['release_offset_mm']
        speed = self.config['joint_velocity_deg_s']
        pulses_left = sum(action == 2 for action, *_ in steps)
        try:
            if purpose != 'prepare':
                self.bridge = Magnet(self.drive.poll)
                self.last_keep = time.monotonic()
                self.drive.on_poll = self.poll_bridge
            self.drive.read_initial()
            if purpose == 'resume_carry':
                _, _, zg, j2g, j3g, _ = steps[0]
                if any(abs(self.drive.states[n]['position'] - goal) > tolerance
                       for n, goal, tolerance in [(1, zg, 4.5), (2, j2g, .18), (3, j3g, 1.2)]):
                    raise RuntimeError('当前姿态与已抬升的取子点不符')
                steps = steps[1:]
            if purpose == 'resume_pickup':
                _, _, zg, j2g, j3g, _ = steps[0]
                if any(abs(self.drive.states[n]['position'] - goal) > tolerance
                       for n, goal, tolerance in [(2, j2g, .18), (3, j3g, 1.6)]) or \
                        not zg - 4.5 <= self.drive.states[1]['position'] <= travel + 12:
                    raise RuntimeError('当前位置与已到位的黑炮取子点不符')
            # 驱动板重启后恢复对弈使用的速度环参数。旧配置缺项沿用板端默认增益。
            kp = PROFILE.get('speed_kp_milli', 1600)
            ki = PROFILE.get('speed_ki_milli', 200000)
            if self.drive.states[1]['power']:
                self.drive.idle(1)
            self.drive.command(1, SET_SPEED_GAINS, kp, ki)
            last_xy = None
            release_at = None
            release_capture = False
            for index, (action, axes, z_goal, j2_goal, j3_goal, duration) in enumerate(steps):
                if action == 0:
                    continue
                if action == 2:
                    if release_at is None:
                        raise RuntimeError('放棋前没有到达释放高度')
                    print('H 桥放棋：' + ('盘外暂存' if release_capture else '抬升时短脉冲'), flush=True)
                    if release_capture:
                        self.magnet_pulse(f'release {duration}', release_at, travel)
                    else:
                        self.board_release(release_at, travel, z, speed[0], duration)
                    release_at = None
                    pulses_left -= 1
                    continue
                release_at = None
                dropping_capture = pulses_left == 2 and abs(z_goal - release) < .1
                if dropping_capture:
                    z_goal = z.get('capture_release_deg', travel)
                if axes & 6:
                    xy = (j2_goal, j3_goal)
                    if xy != last_xy and not (purpose == 'resume_pickup' and index == 0):
                        self.drive.z(travel, high=travel, speed_limit=speed[0])
                        shoulder_speed = min(speed[1], 12 if index == len(steps) - 1 else 18)
                        self.drive.xy(j2_goal, j3_goal, shoulder_speed, speed[2])
                    last_xy = xy
                if axes & 1:
                    self.drive.z(z_goal, high=travel, speed_limit=speed[0])
                    if self.bridge is not None and abs(z_goal - pickup) < .1:
                        self.magnet_pulse(f'attract {PICK_PULSE_MS}', pickup, travel)
                    if dropping_capture or abs(z_goal - release) < .1:
                        release_at = z_goal
                        release_capture = dropping_capture
            if self.bridge is not None:
                self.bridge_command('off')
            if self.drive.states[1]['power']:
                self.drive.idle(1)
            self.drive.read_initial()
            self.confirmed_stopped = True
            elapsed = time.monotonic() - started
            print(f'PHYSICAL_STOPPED {purpose} {move} {elapsed:.2f}s', flush=True)
            self.drive.note('action_complete', purpose=purpose, move=move, elapsed_s=elapsed)
            return True
        except BaseException:
            self.drive.on_poll = None
            self.confirmed_stopped = self.drive.stop()
            raise
        finally:
            self.drive.on_poll = None
            if self.bridge is not None:
                bridge, self.bridge = self.bridge, None
                bridge.close()
            self.drive.close()
            self.drive = None

    def stop(self):
        if self.drive is not None:
            self.confirmed_stopped = self.drive.stop()
        return self.confirmed_stopped

    def close(self):
        self.close_vision()
        self.close_engine()
        return self.stop()


def main():
    os.chdir(BENCH)
    if sys.argv[1:] == ['configure']:
        configure()
        return 0
    opencv_bin = Path(os.environ['USERPROFILE']) / '.cache/scara-geometry/sdk/opencv/build/x64/vc16/bin'
    os.environ['OPENCV_FFMPEG_CAPTURE_OPTIONS'] = 'rtsp_transport;tcp'
    with os.add_dll_directory(str(opencv_bin)):
        dll = ctypes.PyDLL(str(BENCH / 'build/scara_usb_game.dll'))
        run = dll.run_usb_game
        run.argtypes = ctypes.py_object, ctypes.c_char_p, ctypes.c_char_p
        run.restype = ctypes.c_int
        resume = bool({'resume', 'continue-b7'} & set(sys.argv[1:]))
        current = sys.argv[1:] == ['current']
        checkpoint = None
        next_game = Path('resume-next.json')
        if current and next_game.is_file():
            checkpoint = read_game_state(next_game)
            next_game.replace('resume-next-used.json')
        return run(Bench(resume, current and checkpoint is None, checkpoint), b'game-live.json', MODEL.encode('ascii'))


if __name__ == '__main__':
    raise SystemExit(main())
