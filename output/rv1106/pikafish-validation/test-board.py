"""Run bounded UCI searches over the board's Ethernet SSH connection."""
import argparse
import json
from pathlib import Path
import queue
import re
import subprocess
import threading
import time

ROOT = Path(__file__).resolve().parent
SSH = ['ssh', '-b', '192.168.55.1', '-i', 'C:/Users/lst/.ssh/rv1106_codex_auto',
       '-o', 'BatchMode=yes', '-o', 'StrictHostKeyChecking=yes',
       '-o', f'UserKnownHostsFile={ROOT / "network/known_hosts"}',
       '-o', 'ConnectTimeout=5', 'root@192.168.55.6']

parser = argparse.ArgumentParser()
parser.add_argument('version', choices=['2023-03-05', 'current'])
parser.add_argument('--vm-limit-kib', type=int, default=140000)
parser.add_argument('--moves', type=int, default=6)
args = parser.parse_args()
folder = ROOT / args.version
folder.mkdir(exist_ok=True)
log = (folder / 'board-uci.log').open('w', encoding='utf-8')
events = queue.Queue()
board_pid = None
memory = []
result = {'version': args.version, 'transport': 'Ethernet SSH', 'hash_mib': 4,
          'threads': 1, 'virtual_memory_limit_kib': args.vm_limit_kib, 'searches': []}
command = (f'cd /root/scara-pikafish-test/{args.version} && '
           f'ulimit -v {args.vm_limit_kib} && echo ENGINE_PID=$$ && exec ./pikafish')
proc = subprocess.Popen(SSH + [command], stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                        stderr=subprocess.STDOUT, text=True, encoding='utf-8', errors='replace', bufsize=1)

def reader():
    for line in proc.stdout:
        line = line.rstrip()
        log.write(f'{time.monotonic():.3f} < {line}\n')
        log.flush()
        events.put(line)
    events.put(None)

threading.Thread(target=reader, daemon=True).start()

def send(command):
    log.write(f'{time.monotonic():.3f} > {command}\n')
    log.flush()
    proc.stdin.write(command + '\n')
    proc.stdin.flush()

def receive_until(prefix, timeout):
    global board_pid
    deadline = time.monotonic() + timeout
    seen = []
    while time.monotonic() < deadline:
        try:
            line = events.get(timeout=max(0.01, deadline - time.monotonic()))
        except queue.Empty:
            break
        if line is None:
            raise RuntimeError(f'Engine exited before {prefix}; last output: {seen[-5:]}')
        seen.append(line)
        if line.startswith('ENGINE_PID='):
            board_pid = int(line.split('=')[1])
        if line.startswith(prefix):
            return line, seen
    raise TimeoutError(f'Timed out waiting for {prefix}; last output: {seen[-5:]}')

def snapshot(label):
    output = subprocess.check_output(SSH + [f'cat /proc/{board_pid}/status; head -5 /proc/meminfo'],
                                     text=True, encoding='utf-8', timeout=10)
    values = {'phase': label}
    for name in ('VmPeak', 'VmSize', 'VmHWM', 'VmRSS', 'MemAvailable', 'MemFree'):
        match = re.search(rf'^{name}:\s+(\d+)\s+kB', output, re.MULTILINE)
        if match:
            values[name + '_kib'] = int(match[1])
    memory.append(values)

try:
    started = time.monotonic()
    send('uci')
    receive_until('uciok', 30)
    result['startup_seconds'] = round(time.monotonic() - started, 3)
    send('setoption name Threads value 1')
    send('setoption name Hash value 4')
    send('isready')
    receive_until('readyok', 30)
    snapshot('ready')
    played = []
    for index in range(args.moves):
        send('position startpos' + (' moves ' + ' '.join(played) if played else ''))
        duration = 1000 if index < 2 else 3000
        started = time.monotonic()
        send(f'go movetime {duration}')
        best, lines = receive_until('bestmove ', duration / 1000 + 20)
        elapsed = time.monotonic() - started
        move = best.split()[1]
        if not re.fullmatch(r'[a-i][0-9][a-i][0-9]', move):
            raise RuntimeError(f'Unexpected bestmove: {best}')
        infos = [line for line in lines if line.startswith('info depth ')]
        item = {'ply': index + 1, 'requested_ms': duration,
                'elapsed_seconds': round(elapsed, 3), 'bestmove': move,
                'last_info': infos[-1] if infos else None}
        result['searches'].append(item)
        print(json.dumps(item), flush=True)
        played.append(move)
        send('isready')
        receive_until('readyok', 5)
        snapshot(f'after_ply_{index + 1}')
    send('position startpos moves ' + ' '.join(played))
    send('go infinite')
    time.sleep(0.5)
    started = time.monotonic()
    send('stop')
    receive_until('bestmove ', 5)
    result['stop_response_seconds'] = round(time.monotonic() - started, 3)
    snapshot('after_stop')
    result['passed'] = True
except Exception as exc:
    result['passed'] = False
    result['error'] = str(exc)
finally:
    if proc.poll() is None:
        try:
            send('stop')
            send('quit')
            proc.wait(timeout=10)
        except Exception:
            proc.terminate()
    result['memory'] = memory
    result['exit_code'] = proc.poll()
    (folder / 'board-results.json').write_text(json.dumps(result, indent=2), encoding='utf-8')
    log.close()
    print(json.dumps(result, indent=2), flush=True)
    if not result.get('passed'):
        raise SystemExit(1)
