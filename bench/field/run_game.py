"""Field I/O for the existing C++ game, with physical USB-CAN and RV1106."""

import base64
import ctypes
import io
from http.client import HTTPConnection
import json
import os
from pathlib import Path
import queue
import re
import select
import shlex
import signal
import socket
import subprocess
import sys
import threading
import time
import zlib

from drive import BUILDS, CLEAR, Drive, PROFILE, ROOT, SET_SPEED_GAINS, apply_rotary_tuning

BENCH = Path(__file__).resolve().parent
ADB = os.environ.get('SCARA_ADB', 'adb')
ADB_TARGET = ['-s', os.environ['ANDROID_SERIAL']] if os.environ.get('ANDROID_SERIAL') else []
MODEL = '/userdata/scara-yolo-v3/xiangqi14-local-v3-rv1106.rknn'
MODEL_SHA = 'ce9c91f4668cb0ec58f836432546b7eb2aebfc675ba696ce9788968560dcbdd2'
UCI = re.compile(r'[a-i][0-9][a-i][0-9]')
START_BOARD = ['rnbakabnr', '.........', '.c.....c.', 'p.p.p.p.p', '.........',
               '.........', 'P.P.P.P.P', '.C.....C.', '.........', 'RNBAKABNR']

NPU_PORT = 18188
PICK_PULSE_MAX_MS = 450

# ---------板端设备准备区---------
# ADB 先确认 NPU 设备节点，随后用独占连接控制电磁铁。
BOARD_BOOTSTRAP = '''
set -e
test -c /dev/rknpu || insmod /oem/usr/ko/rknpu.ko
test -c /dev/rknpu
echo SCARA_IO_READY
'''

# 这段程序在 RV1106 上运行。hold 将吸合 GPIO 拉高，off 将两路控制清零，退出也会关断。
BRIDGE_SCRIPT = f'''
from fcntl import flock, LOCK_EX, LOCK_NB
from pathlib import Path
import signal
import socket
import time
ownership = Path('/tmp/scara-pwm7.lock').open('a')
flock(ownership, LOCK_EX | LOCK_NB)
gpio = Path('/sys/class/gpio/gpio40')
if not gpio.is_dir():
    (gpio.parent / 'export').write_text('40')
(gpio / 'direction').write_text('low')
attract = gpio / 'value'
chip = next(chip for chip in Path('/sys/class/pwm').glob('pwmchip*')
            if '/ff360030.pwm/' in str(chip.resolve()))
pwm = chip / 'pwm0'
if not pwm.is_dir():
    (chip / 'export').write_text('0')
duty_cycle = pwm / 'duty_cycle'
if (pwm / 'polarity').read_text().strip() != 'normal':
    raise RuntimeError('Unexpected PWM7 polarity')
if int((pwm / 'period').read_text()) == 0:
    (pwm / 'period').write_text('2000000')
duty_cycle.write_text('0')
if (pwm / 'enable').read_text().strip() != '1':
    (pwm / 'enable').write_text('1')

def write(path, value):
    value = str(value)
    path.write_text(value)
    if path.read_text().strip() != value:
        raise RuntimeError('H-bridge output feedback mismatch')

def neutral():
    write(duty_cycle, 0)
    write(attract, 0)

def stop(*_):
    raise SystemExit(0)

for event in (signal.SIGHUP, signal.SIGINT, signal.SIGTERM):
    signal.signal(event, stop)
try:
    neutral()
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
                if request == 'hold':
                    write(attract, 1)
                elif request == 'off':
                    neutral()
                elif request == 'quit':
                    break
                else:
                    raise ValueError('Expected hold, off or keep')
                reply = b'BRIDGE_OK\\n'
                connection.sendall(reply)
finally:
    neutral()
'''


def remote_python(script, *args):
    return 'exec env PYTHONDONTWRITEBYTECODE=1 python3 -u -c ' + ' '.join(map(shlex.quote, (script, *args)))


def adb_command(*args):
    return subprocess.check_output([str(ADB), *ADB_TARGET, *args], text=True, timeout=4,
                                   creationflags=getattr(subprocess, 'CREATE_NO_WINDOW', 0)).strip()


# ---------板端进程收发区---------
# 标准输出由后台线程逐行放进队列，主线程用匹配条件等候本次回复。
class Lines:
    def __init__(self, command):
        self.lines = queue.Queue()
        self.process = subprocess.Popen([str(ADB), *ADB_TARGET, 'shell', command], stdin=subprocess.PIPE,
                                        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, encoding='utf-8',
                                        errors='replace', bufsize=1, creationflags=getattr(subprocess, 'CREATE_NO_WINDOW', 0))
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
            try:
                line = self.lines.get(timeout=0 if poll else remaining)
            except queue.Empty:
                if poll:
                    poll()
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


# ---------电磁铁会话区---------
class Magnet:
    def __init__(self, poll):
        # 板上的旧 ADB 限制命令长度，因此压缩脚本后再传输。
        payload = base64.b64encode(zlib.compress(BRIDGE_SCRIPT.encode('utf-8'))).decode('ascii')
        bootstrap = 'import base64, sys, zlib; exec(zlib.decompress(base64.b64decode(sys.argv[1])))'
        self.process = Lines(remote_python(bootstrap, payload))
        self.port = None
        self.socket = None
        self.reply = b''
        try:
            # 先等板端监听就绪，再通过 ADB 转发端口连接并核对首个回执。
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
            # H 桥没有回执时继续处理 CAN，避免每轮都等套接字超时。
            if not select.select([self.socket], [], [], 0)[0]:
                continue
            try:
                data = self.socket.recv(256)
            except TimeoutError:
                continue
            if not data:
                raise RuntimeError('H 桥连接已关闭')
            self.reply += data
        result, newline, self.reply = self.reply.partition(b'\n')
        status, _, pulse_ms = result.partition(b' ')
        if not newline or status != b'BRIDGE_OK':
            raise RuntimeError('H 桥动作回读失败')
        return float(pulse_ms) if pulse_ms else None

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


FIELD_BASE = [1.65, 18, 42, 106, 110, 180, 75, 125, True, 0, 0, 16]
FIELD_Z = {'game_speed_deg_s': 4800,
 'speed_kp_milli': 1480,
 'speed_ki_milli': 100000,
 'voltage_limit_mv': 40000,
 'acceleration_deg_s2': 5700,
 'brake_deg_s2': 5200,
 'descent_brake_deg_s2': 5000,
 'descent_brake_ki_permille': 950,
 'release_acceleration_deg_s2': 5700,
 'z_settle_gain_milli': 24000}

def field_limits(samples):
    # 台架实测的可达区及驱动调校，独立于对局与示教参数。
    return dict(cad_workspace=dict(samples=samples, neutral_shoulder_deg=-92.,
                                   registration_uncertainty_deg=2.5, transfer_shoulder_deg=-85.,
                                   transfer_speed_deg_s=[18, 60]),
                motion_tuning=dict(base=FIELD_BASE, z=FIELD_Z))

# ---------安装数据交接区---------
# 把 DLL 给出的参考角、工作层和 CAD 数据转成台架查路、发指令时使用的配置。
def compiled_config(site):
    j2, j3, shoulder, elbow, k2, k3 = site['arm']
    pickup, travel, release = site['z']
    return dict(j2_reference_deg=j2, j3_reference_motor_deg=j3, joint_per_drive=[k2, k3],
                shoulder_ref_rad=shoulder, elbow_ref_rad=elbow,
                arm_lengths_mm=site['arm_lengths_mm'],
                park_joints_deg=list(site['park']), joint_velocity_deg_s=list(site['velocity']),
                joint_acceleration_deg_s2=list(site['acceleration']), pickup_at_rest_ms=site['grip_ms'],
                z=dict(pickup_deg=pickup, clearance_deg=site.get('clearance_deg', travel), travel_deg=travel, release_deg=release),
                engine_directory=site['engine_directory'], think_ms=site['think_ms'],
                **field_limits(io.BytesIO(site['cad'])))


# ---------台架对局会话区---------
class Bench:
    def __init__(self, config, current_position=False, checkpoint=None, self_play=0):
        self.config = config
        self.initial_board = (checkpoint or {}).get('initial_board')
        cad = self.config.get('cad_workspace')
        self.workspace = None
        if cad:
            from workspace import Workspace
            # 用此配置和 CAD 采样表建立工作空间，搬棋前从当前关节姿态查路线。
            self.workspace = Workspace(config, cad)
        self.tuning = config.get('motion_tuning')
        self.tuning_applied = False
        self.pickup_at_rest_ms = self.config.get('pickup_at_rest_ms')
        if self.pickup_at_rest_ms is not None and not (type(self.pickup_at_rest_ms) is int and 0 < self.pickup_at_rest_ms <= PICK_PULSE_MAX_MS):
            raise ValueError(f'取子定点吸合时长必须为 1 到 {PICK_PULSE_MAX_MS} 毫秒')
        # C++ 与 USB-CAN 从同一份现场参数取得 J1 速度和起动加速度。
        velocity = self.config['joint_velocity_deg_s']
        acceleration = self.config['joint_acceleration_deg_s2']
        if self.tuning:
            PROFILE.update(self.tuning['z'])
            rotary_speed = apply_rotary_tuning(self.tuning['base'])
            assert velocity == [PROFILE['game_speed_deg_s'], *rotary_speed]
            assert acceleration == [PROFILE['acceleration_deg_s2'], self.tuning['base'][4], self.tuning['base'][5]]
        self.heights = self.config
        self.self_play = self_play
        self.auto_choice = ''
        self.confirmed_plies = -1
        self.npu = None
        self.npu_pid = None
        self.local_port = None
        self.shared_npu_port = self.config.get('npu_forward_port')
        self.pikafish = None
        self.drive = None
        self.hold_thread = None
        self.hold_stop = threading.Event()
        self.bridge = None
        self.last_keep = 0.0
        self.start_checks = 0
        self.confirmed_stopped = True
        self.current_position = current_position
        self.saved = checkpoint
        self.expected = self.saved['board'] if self.saved else self.config.get('starting_board', START_BOARD)

    # 把 C++ 传来的两串 90 格拆成十行，连同帧号、棋谱、走法和执行状态推给面板。
    def state(self, message, frame, history, board, observed, result, pending, valid, legal):
        rows = [board[i:i+9] for i in range(0,90,9)]
        seen = [observed[i:i+9] for i in range(0,90,9)]
        state = dict(error=message, frame_id=frame, history=history, board=rows,
                     observed=seen, result=result, pending=pending, observation_valid=valid,
                     simulated=0, legal_rc=legal, initial_board=self.initial_board)
        print('MATCH_STATE ' + json.dumps(state, ensure_ascii=False), flush=True)

    def preview(self, jpeg):
        print('MATCH_PREVIEW ' + base64.b64encode(jpeg).decode('ascii'), flush=True)

    # 按横移的起止姿态选择实测档位，其余路段使用基本档位。
    def xy_speed(self, origin, target):
        if not self.tuning:
            return self.config['joint_velocity_deg_s'][1:]
        references = self.config.get('motion_tuning_joints_deg', {})
        for leg, settings in self.tuning.get('legs', {}).items():
            source, destination = leg.split('_to_')
            if source not in references or destination not in references:
                continue
            # 专用速度只用于和实测轨迹相同的两端姿态。
            if all(abs(a - b) < .01 for a, b in zip((*origin, *target),
                   (*references[source], *references[destination]))):
                return apply_rotary_tuning(settings)
        return apply_rotary_tuning(self.tuning['base'])

    def resume_history(self):
        return self.saved['history'] if self.saved else ''

    def resume_board(self):
        return self.saved.get('initial_board', START_BOARD) if self.saved else START_BOARD

    # ---------板端视觉区---------
    def open(self, model_path):
        if model_path != MODEL:
            raise RuntimeError('板端模型路径与本次已核对模型不一致')
        # 板重启后准备 NPU；磁铁由独占的 H 桥会话初始化。
        if 'SCARA_IO_READY' not in adb_command('shell', BOARD_BOOTSTRAP).splitlines():
            raise RuntimeError('板端NPU或电磁铁控制脚初始化失败')
        if self.shared_npu_port is not None:
            connection = HTTPConnection('127.0.0.1', self.shared_npu_port, timeout=3)
            try:
                connection.request('GET', '/health')
                response = connection.getresponse()
                if response.status != 200 or json.loads(response.read()).get('model_sha256') != MODEL_SHA:
                    raise RuntimeError('已有 NPU 服务的模型与当前配置不符')
            finally:
                connection.close()
        # KeyboardInterrupt is allowed to unwind any Python hardware call before returning to C++.
        signal.signal(signal.SIGINT, signal.default_int_handler)

    def infer(self, data, frame):
        try:
            if self.npu is None and self.shared_npu_port is None:
                self.npu = Lines('echo NPU_PID=$$; exec python3 -u /userdata/scara-yolo-v3/board_npu.py '
                                 f'--model {MODEL} --model-sha256 {MODEL_SHA} --bind 127.0.0.1 '
                                 f'--client 127.0.0.1 --port {NPU_PORT}')
                self.npu_pid = self.npu.read(lambda line: int(line[8:]) if line.startswith('NPU_PID=') else None, 4)
                self.npu.expect(f'READY http://127.0.0.1:{NPU_PORT}/', 10)
                self.local_port = int(adb_command('forward', 'tcp:0', f'tcp:{NPU_PORT}'))
            # 把本帧 RGB 字节和帧号送到推理服务，原样交回张量包，由 C++ 解码。
            connection = HTTPConnection('127.0.0.1', self.shared_npu_port or self.local_port, timeout=5)
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

    # ---------皮卡鱼与操作员输入区---------
    def engine(self, fen, history, search):
        # 引擎收到的 FEN 来自已确认的开局。面板接续本局时需要还原这十行。
        if self.initial_board is None:
            self.initial_board = [''.join('.' * int(piece) if piece.isdigit() else piece for piece in row)
                                  for row in fen.split(' ', 1)[0].split('/')]
        try:
            if self.self_play:
                plies = len(history.split())
                if plies > self.confirmed_plies:
                    self.confirmed_plies = plies
                    progress = str(plies) if self.self_play < 0 else f'{plies}/{self.self_play}'
                    print(f'SELF_PLAY_CONFIRMED {progress} {history}', flush=True)
                search = self.self_play < 0 or plies < self.self_play
            if self.pikafish is None:
                self.pikafish = Lines('cd ' + shlex.quote(self.config['engine_directory']) + ' && ulimit -v 140000 && exec ./pikafish')
                self.pikafish.send('uci')
                self.pikafish.expect('uciok', 8)
                self.pikafish.send('setoption name Threads value 1')
                self.pikafish.send('setoption name Hash value 4')
                self.pikafish.send('isready')
                self.pikafish.expect('readyok', 8)
            engine = self.pikafish
            # 引擎先收到起始 FEN 和完整棋谱，随后返回这个局面的一层合法走法。
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
            # 需要应手时在允许的走法中搜索。自对弈可进一步指定可执行格点。
            if search and legal:
                cells = self.config.get('self_play_cells') if self.self_play else None
                available = legal if cells is None else [
                    move for move in legal if move[:2] in cells and move[2:] in cells]
                if not available:
                    raise RuntimeError('当前肘形没有可执行的合法走法')
                # 上次已选出的应手在同一局面继续执行，后续局面仍正常搜索。
                best = self.saved.get('retry_reply', '') if self.saved and self.saved['history'] == history else ''
                if not best:
                    query = f"go movetime {self.config['think_ms']}"
                    if len(available) != len(legal):
                        query += ' searchmoves ' + ' '.join(available)
                    engine.send(query)
                    best = engine.read(lambda line: line.split()[1] if line.startswith('bestmove ') else None, 8)
                if best not in available:
                    raise RuntimeError('Pikafish 应手不在可执行走法表中')
            if self.self_play:
                self.auto_choice = best
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

    # 当前盘面由用户核对。标准开局和续局对照本次要求的棋盘。
    def confirm_opening(self, observed):
        if self.current_position:
            print('OPENING 核对上位机盘面与实体棋盘一致，再确认开局。', flush=True)
            answer = input().strip().lower()
            if answer == 'quit':
                return None
            if answer in ('', 'ready'):
                return True
            if answer in ('resign', 'draw'):
                return False
            raise RuntimeError('未识别的终端命令')
        seen = [observed[i:i+9] for i in range(0,90,9)]
        self.start_checks += 1
        if seen == self.expected:
            print('BOARD_CONFIRMED ' + (self.resume_history() or 'opening 32'), flush=True)
            return True
        changed = [f'({r},{c}) {piece}->{seen[r][c]}'
                   for r, row in enumerate(self.expected) for c, piece in enumerate(row) if seen[r][c] != piece]
        print('OPENING_DIFFERENCE ' + ' '.join(changed), flush=True)
        if self.start_checks >= 6:
            raise RuntimeError('实拍开局盘面仍有差异：' + ' '.join(changed))
        return False

    def command(self, prompt, choosing_move):
        if self.self_play:
            return (self.auto_choice or 'quit') if choosing_move else 'quit'
        print(('HUMAN_TURN ' if choosing_move else 'CONFIRM ') + prompt, flush=True)
        return input().strip().lower()

    # ---------吸持与反馈区---------
    def poll_bridge(self):
        if self.bridge is not None and time.monotonic() - self.last_keep >= .12:
            self.bridge.send('keep')
            self.last_keep = time.monotonic()

    def bridge_command(self, action):
        self.bridge.send(action)
        pulse_ms = self.wait_bridge_reply()
        self.drive.note('magnet_confirmed', action=action)
        return pulse_ms

    def wait_bridge_reply(self):
        def poll():
            self.drive.poll()
            self.drive.healthy()
        pulse_ms = self.bridge.expect(poll)
        self.drive.healthy()
        return pulse_ms

    # 切换磁铁前，等三轴静止，并核对 J1 的实到角与本步指定的工作高度。
    def wait_magnet_height(self, height, travel):
        def at_height():
            a = self.drive.states[1]
            offset = a['position'] - height
            return all(s['stationary'] for s in self.drive.states.values()) and (
                abs(offset) <= 4.5 or (height == travel and 0 < offset <= 12))
        self.drive.wait(at_height, .35)

    def stop_hold_feedback(self):
        if self.hold_thread is not None:
            self.hold_stop.set()
            self.hold_thread.join()
            self.hold_thread = None

    def hold_feedback(self):
        while not self.hold_stop.is_set():
            try:
                self.drive.poll()
            except (OSError, RuntimeError) as error:
                self.drive.note('hold_feedback_error', error=str(error))
                break

    def recover_parked_elbow(self):
        self.drive.wait(lambda: len(self.drive.states) == 3, .9, check=False)
        axis = self.drive.states[3]
        if (axis['fault'], axis['detail']) != (5, 4):
            return
        if (axis['build'] != BUILDS[3] or axis['power'] or not axis['stationary']
                or not axis['referenced'] or not axis['encoder_valid']):
            raise RuntimeError('J3 的当前位置尚不能接续')
        self.drive.sync_sequence(3, check=False)
        self.drive.command(3, CLEAR, check=False)
        self.drive.wait(lambda: not self.drive.states[3]['fault'] and self.drive.states[3]['referenced'],
                        1, check=False)
        self.drive.note('parked_elbow_recovered', position=axis['position'])

    def apply_drive_tuning(self):
        # 驱动板重启后恢复对弈使用的速度环参数。旧配置缺项沿用板端默认增益。
        if not self.tuning or not self.tuning_applied:
            kp = PROFILE.get('speed_kp_milli', 1600)
            ki = PROFILE.get('speed_ki_milli', 200000)
            if self.drive.states[1]['power']:
                self.drive.idle(1)
            self.drive.command(1, SET_SPEED_GAINS, kp, ki, c=PROFILE.get('voltage_limit_mv', 0),
                               d=PROFILE.get('z_settle_gain_milli', 0))
            if self.tuning:
                if self.drive.states[3]['power']:
                    self.drive.idle(3)
                self.drive.command(3, SET_SPEED_GAINS, *PROFILE['j3_speed_gains'],
                                   d=PROFILE['j3_settle_gain_milli'])
                self.tuning_applied = True

    # ---------按 C++ 动作表搬棋区---------
    def execute(self, purpose, move, steps):
        started = time.monotonic()
        print(f'PHYSICAL {purpose} {move} {len(steps)} steps', flush=True)
        # 结束上一手的保持读回线程后，由这一手接管总线反馈。
        self.stop_hold_feedback()
        if self.drive is None:
            self.drive = Drive(record=False)
        self.drive.note('action_start', purpose=purpose, move=move)
        self.confirmed_stopped = False
        keep_hold = False
        z = self.heights['z']
        travel = z['travel_deg']
        speed = self.config['joint_velocity_deg_s']

        def move_z(goal):
            self.drive.z(goal, high=travel, speed_limit=speed[0])

        try:
            if self.tuning:
                apply_rotary_tuning(self.tuning['base'])
            self.bridge = Magnet(self.drive.poll)
            self.last_keep = time.monotonic()
            self.drive.on_poll = self.poll_bridge
            if purpose == 'prepare' and self.self_play:
                self.recover_parked_elbow()
            self.drive.read_initial()
            self.apply_drive_tuning()
            actual_xy = tuple(self.drive.states[n]['position'] for n in (2, 3))
            routes = self.workspace.plan(actual_xy, steps, self.config['park_joints_deg']) if self.workspace else {}
            # 用当前 J2、J3 实测角和整手目标预先选好路径，横移按这份结果执行。
            previous_goal = None
            reached = actual_xy
            height = self.drive.states[1]['position']
            for index, (action, axes, z_goal, j2_goal, j3_goal, dwell_ms) in enumerate(steps):
                if action in (0, 2, 3):
                    if purpose != 'prepare':
                        self.wait_magnet_height(height, travel)
                    self.bridge_command('hold' if action == 0 else 'off')
                    if action == 0 and dwell_ms:
                        until = time.monotonic() + dwell_ms/1000
                        self.drive.wait(lambda: time.monotonic() >= until, dwell_ms/1000+.2)
                    if action in (2, 3):
                        self.drive.note('piece_released',move=move,height_deg=height,off_board=action==3)
                    continue
                if action != 1:
                    raise ValueError('未知 C++ 动作')
                goal = (j2_goal,j3_goal)
                # J2 或 J3 的目标发生变化时，先让 J1 到搬运层，再依次走横移途经点。
                if axes & 6 and goal != previous_goal:
                    move_z(travel)
                    height = travel
                    route, transfer = routes.get(index, ([goal],False))
                    if transfer:
                        self.drive.note('elbow_transfer',move=move,waypoints=route)
                    for waypoint in route:
                        rates = self.xy_speed(reached, waypoint)
                        if transfer:
                            rates = [min(rate,limit) for rate,limit in zip(rates,self.workspace.transfer_speed)]
                        self.drive.xy(*waypoint,*rates)
                        reached = waypoint
                    previous_goal = goal
                # 横移结束后，含 J1 的动作才执行本步指定的目标高度。
                if axes & 1:
                    move_z(z_goal)
                    height = z_goal
            if self.bridge is not None:
                self.bridge_command('off')
            if not self.tuning and self.drive.states[1]['power']:
                self.drive.idle(1)
            # 重新获取驱动状态后报告本次执行完成，实际落点由 C++ 视觉核对。
            self.drive.read_initial()
            self.confirmed_stopped = True
            elapsed = time.monotonic() - started
            print(f'PHYSICAL_STOPPED {purpose} {move} {elapsed:.2f}s', flush=True)
            self.drive.note('action_complete', purpose=purpose, move=move, elapsed_s=elapsed)
            keep_hold = bool(self.self_play)
            return True
        # 执行中断时记录动作和各轴反馈，再请求关断磁铁和停车。
        except BaseException as error:
            self.tuning_applied = False
            self.drive.note('action_error', purpose=purpose, move=move,
                            error=str(error), axes=self.drive.states.copy())
            if self.bridge is not None:
                try:
                    self.bridge.send('off')
                except OSError:
                    pass
            self.drive.on_poll = None
            self.confirmed_stopped = self.drive.stop()
            raise
        finally:
            self.drive.on_poll = None
            if self.bridge is not None:
                bridge, self.bridge = self.bridge, None
                bridge.close()
            # 自对弈在两手之间继续刷新保持反馈，其他情况关闭本次驱动连接。
            if keep_hold:
                self.hold_stop.clear()
                self.hold_thread = threading.Thread(target=self.hold_feedback, daemon=True)
                self.hold_thread.start()
            else:
                self.drive.close()
                self.drive = None

    def stop(self):
        self.stop_hold_feedback()
        if self.drive is not None:
            self.confirmed_stopped = self.drive.stop()
        return self.confirmed_stopped

    def close(self):
        self.close_vision()
        self.close_engine()
        try:
            return self.stop()
        finally:
            if self.drive is not None:
                self.drive.close()
                self.drive = None


# ---------Windows 对局入口区---------
def main():
    sys.path.insert(0, str(ROOT/'bench'))
    from teach import default_site, field_config

    selected = os.environ.get('SCARA_SITE')
    path = field_config(Path(selected).resolve() if selected else default_site())
    site_text = path.read_bytes()
    binary_dir = Path(os.environ.get('SCARA_BENCH_BIN', ROOT/'build/bench/bin')).resolve()
    os.environ['OPENCV_FFMPEG_CAPTURE_OPTIONS'] = 'rtsp_transport;tcp'
    with os.add_dll_directory(str(binary_dir)):
        dll = ctypes.PyDLL(str(binary_dir/'scara_usb_game.dll'))
        dll.usb_installation.argtypes = ctypes.c_char_p, ctypes.c_size_t
        dll.usb_installation.restype = ctypes.py_object
        site = dll.usb_installation(site_text, len(site_text))
        run = dll.run_usb_game
        run.argtypes = ctypes.py_object, ctypes.c_char_p, ctypes.c_char_p, ctypes.c_size_t
        run.restype = ctypes.c_int
        args = sys.argv[1:]
        checkpoint = json.loads(os.environ['SCARA_MATCH_RESUME']) if 'SCARA_MATCH_RESUME' in os.environ else None
        if args and args[0] in ('resume', 'self-play-resume') and not checkpoint:
            raise RuntimeError('当前进程没有已确认的棋谱，请从上位机接续本局。')
        full = args[:1] == ['self-play-full']
        if full and len(args) > 1:
            if (len(args) != 2 or not UCI.fullmatch(args[1]) or not checkpoint or not checkpoint.get('pending')
                    or not checkpoint.get('observation_valid') or checkpoint['observed'] != checkpoint['board']):
                raise RuntimeError('没有可重试的已确认盘面')
            checkpoint['retry_reply'] = args[1]
        plies = -1 if full else 10 if args in (['self-play'], ['self-play-resume']) else 0
        bench = Bench(current_position=args == ['current'] and checkpoint is None, checkpoint=checkpoint,
                      self_play=plies, config=compiled_config(site))
        return run(bench, MODEL.encode(), site_text, len(site_text))


if __name__ == '__main__':
    raise SystemExit(main())
