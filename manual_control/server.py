"""Local browser jog panel, reusing the project's verified Candlelight transport."""
import argparse
from collections import deque
from datetime import datetime
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import queue
import secrets
import signal
import struct
import sys
import threading
import time
from types import SimpleNamespace
from urllib.parse import urlsplit

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(ROOT / 'temp' / 'transport'))
from core import Controller, PACKET, MOVE, STOP, PING, VELOCITY
from telemetry import diagnostic


class Simulator:
    """Deterministic protocol peer for UI testing; never opens USB."""
    def __init__(self, config):
        self.rows = {int(n): dict(position=0., target=0., velocity=0., speed=1., state=0,
                     last_command=time.monotonic(), seq=0, interlocked=False,
                     build=int(c['builds'][0], 16)) for n, c in config['axes'].items()}
        self.rx = deque()
        self.last = time.monotonic()
        self.start = self.last
        self.last_status = 0

    def send_packet(self, n, data):
        kind, _, _, _, seq, *v = PACKET.unpack(data)
        r = self.rows[n]
        delta = (seq - r['seq']) & 0xffffffff
        accepted = 0 < delta < 0x80000000
        if accepted:
            r['seq'] = seq
            if kind == MOVE:
                accepted = r['state'] in (0, 1) and not r['interlocked']
                if accepted:
                    r.update(target=v[0] / 1000, speed=v[1] / 1000, state=2)
            elif kind==VELOCITY:
                accepted=r['state'] in (0,1) and not r['interlocked']
                if accepted:r.update(speed=v[0]/1000,state=5)
            elif kind == STOP:
                accepted = r['state'] in (1, 2, 3, 5)
                if accepted:
                    r['state'] = 3
            elif kind != PING:
                accepted = False
        if accepted:
            r['last_command'] = time.monotonic()
        self.rx.append(SimpleNamespace(identifier=0x480+n, data=PACKET.pack(
            kind | 0x80, n, int(not accepted), 3, seq, *([0] * 14)), echo=False, error=False,
            fd=True, brs=True, extended=False))

    def recv(self, timeout):
        now = time.monotonic()
        dt, self.last = min(.05, now - self.last), now
        for r in self.rows.values():
            if now - r['last_command'] > .5 and r['state'] in (1, 2, 3, 5):
                r['interlocked'], r['state'] = True, 3
            want = 0.
            if r['state']==5:want=r['speed']
            if r['state'] == 2:
                distance = r['target'] - r['position']
                want = min(r['speed'], (abs(distance) * 60) ** .5) * (1 if distance > 0 else -1)
            old = r['velocity']
            r['velocity'] += max(-30 * dt, min(30 * dt, want - old))
            r['position'] += (old + r['velocity']) * dt / 2
            if r['state'] == 3 and abs(r['velocity']) < .001:
                r['state'], r['target'] = 1, r['position']
            if r['state'] == 2 and abs(r['target'] - r['position']) < .01 and abs(r['velocity']) < .1:
                r['position'], r['velocity'], r['state'] = r['target'], 0., 1
        if now - self.last_status >= .05:
            self.last_status = now
            for n, r in self.rows.items():
                flags = 1 | 2 | 16 | 32 | (64 if r['state'] != 0 else 0) | (256 if abs(r['velocity']) < .05 else 0) | (8 if r['interlocked'] else 0)
                v = [0] * 14
                v[0], v[1], v[2], v[4] = int((now-self.start)*1000), round(r['position']*1000), round(r['target']*1000), round(r['velocity']*1000)
                v[9] = r['state'] | flags << 16
                v[10] = struct.unpack('<i', struct.pack('<I', r['build']))[0]
                self.rx.append(SimpleNamespace(identifier=0x200+n, data=PACKET.pack(0x40, n, 0, 3, 0, *v),
                    echo=False, error=False, fd=True, brs=True, extended=False))
        if self.rx:
            return self.rx.popleft()
        time.sleep(min(timeout, .005))

    def close(self):
        pass


class Worker:
    def __init__(self, config, simulate, log_dir):
        self.config, self.simulate = config, simulate
        log_dir.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now().strftime('%Y%m%d-%H%M%S-%f')
        self.log_path = log_dir / (stamp + ('-sim' if simulate else '-real') + '.jsonl')
        self.log_file = self.log_path.open('x', encoding='utf-8', buffering=65536)
        self.requests = queue.Queue(maxsize=32)
        self.stop_requests = queue.SimpleQueue()
        self.shutdown = threading.Event()
        self.lock = threading.Lock()
        self.view = {}
        self.adapter = None
        self.diagnostics = {}
        self.rx_queue = queue.Queue(maxsize=4096)
        self.rx_stop = threading.Event()
        self.rx_thread = None
        self.rx_error = None
        self.next_snapshot = 0.
        self.last_flush = 0.
        self.reference_path = log_dir / ('pending-j1-reference-sim.json' if simulate else 'pending-j1-reference.json')
        self.core = Controller(config, self.send, audit=self.audit, reference_path=self.reference_path)
        self.thread = threading.Thread(target=self.run, name='can-owner')

    def audit(self, event):
        self.log_file.write(json.dumps(dict(wall_time=datetime.now().astimezone().isoformat(),
                                           simulated=self.simulate, **event), ensure_ascii=False) + '\n')
        if 'message' in event or event.get('event') == 'tx':
            self.log_file.flush()

    def start_receiver(self):
        self.rx_queue = queue.Queue(maxsize=4096)
        self.rx_stop.clear()
        self.rx_error = None
        adapter = self.adapter
        def receive():
            try:
                while not self.rx_stop.is_set():
                    frame = adapter.recv(.01)
                    if frame is not None:
                        self.rx_queue.put_nowait((frame, time.monotonic()))
            except Exception as exc:
                self.rx_error = exc
        self.rx_thread = threading.Thread(target=receive, name='can-usb-receive')
        self.rx_thread.start()

    def read_frame(self):
        if self.simulate:
            return self.adapter.recv(.005), time.monotonic()
        if self.rx_error is not None:
            raise RuntimeError('CAN接收线程失败：' + str(self.rx_error))
        try:
            return self.rx_queue.get(timeout=.005)
        except queue.Empty:
            return None, time.monotonic()

    def close_adapter(self):
        self.rx_stop.set()
        if self.rx_thread:
            self.rx_thread.join(1)
            if self.rx_thread.is_alive():
                raise RuntimeError('USB读取尚未退出，未并发关闭设备')
            self.rx_thread = None
        if self.adapter:
            self.adapter.close()
            self.adapter = None

    def send(self, node, data):
        self.audit(dict(event='tx', id=0x400+node, data=data.hex()))
        if self.simulate:
            self.adapter.send_packet(node, data)
        else:
            from stepfoc_candlelight import Frame
            self.adapter.send(Frame(0x400+node, data, fd=True, brs=True))

    def connect(self):
        if self.core.armed or self.core.active or self.core.recovering is not None:
            raise ValueError('先停止并取消手动控制，再重新连接')
        self.close_adapter()
        previous_events = self.core.events
        self.core = Controller(self.config, self.send, audit=self.audit, reference_path=self.reference_path)
        self.core.events = previous_events
        self.diagnostics = {}
        if self.simulate:
            self.adapter = Simulator(self.config)
        else:
            from stepfoc_candlelight import Candlelight
            self.adapter = Candlelight(serial=self.config['serial'], fd=True)
            self.start_receiver()
        self.core.connected = True
        self.core.log('USB-CAN 已打开，只接收节点状态' if not self.simulate else '模拟器已连接，不访问真实硬件')

    def submit(self, body):
        request = dict(body=body, expires=time.monotonic()+.25, done=threading.Event(), error=None)
        if body.get('action') == 'stop':
            self.stop_requests.put(request)
        else:
            self.requests.put_nowait(request)
        if not request['done'].wait(.6):
            raise ValueError('控制线程未及时响应；页面停止续发心跳')
        if request['error']:
            raise ValueError(request['error'])

    def process(self, req):
        try:
            action = req['body'].get('action')
            if action != 'stop' and time.monotonic() > req['expires']:
                raise ValueError('排队操作已过期，未执行')
            if action == 'connect':
                self.connect()
            else:
                self.core.action(**req['body'])
        except Exception as exc:
            req['error'] = str(exc)
            if not isinstance(exc, (ValueError, TypeError)):
                self.core.fail('控制异常：' + str(exc))
            self.core.log('操作未完成：' + str(exc))
        finally:
            req['done'].set()

    def run(self):
        try:
            try:
                self.connect()
            except Exception as exc:
                self.core.message = str(exc)
                self.core.log('连接失败：' + str(exc))
            while not self.shutdown.is_set():
                try:
                    # A delayed start can never overtake a later stop.
                    while not self.stop_requests.empty():
                        self.process(self.stop_requests.get_nowait())
                    self.core.tick()
                    try:
                        req = self.requests.get_nowait()
                    except queue.Empty:
                        req = None
                    if req:
                        self.process(req)
                    if self.adapter:
                        f, received_at = self.read_frame()
                        if f and not f.echo:
                            if time.monotonic()-received_at > .25:
                                self.core.fail('接收处理延迟，旧帧不能当作实时反馈')
                                continue
                            if f.error:
                                self.audit(dict(event='can_error', id=f.identifier, data=f.data.hex()))
                                from stepfoc_candlelight import is_candlelight_status_report, describe_bus_error
                                if not is_candlelight_status_report(f):
                                    self.core.fail(describe_bus_error(f))
                            elif f.identifier - 0x200 in self.core.axes or f.identifier - 0x480 in self.core.axes:
                                self.audit(dict(event='rx', id=f.identifier, data=f.data.hex()))
                                if not f.fd or not f.brs or f.extended:
                                    self.core.fail('节点帧不是预期的标准 CAN FD/BRS 格式')
                                else:
                                    self.core.receive(f.identifier, f.data, received_at)
                            elif f.identifier - 0x300 in self.core.axes or f.identifier - 0x380 in self.core.axes:
                                self.audit(dict(event='rx', id=f.identifier, data=f.data.hex()))
                                if not f.fd or not f.brs or f.extended:
                                    self.core.fail('诊断帧不是标准 CAN FD/BRS 格式')
                                else:
                                    node, kind, values = diagnostic(f.identifier, f.data)
                                    self.diagnostics.setdefault(node, {})[kind] = dict(values, received=received_at)
                    else:
                        time.sleep(.01)
                except Exception as exc:
                    self.core.fail('USB/CAN 异常：' + str(exc))
                    self.core.connected = False
                    if self.adapter:
                        try:
                            self.close_adapter()
                        except Exception:
                            pass
                        self.adapter = None
                now = time.monotonic()
                if now >= self.next_snapshot:
                    self.next_snapshot = now+.02
                    diag = {n: {kind: dict(v, age_ms=round((now-v['received'])*1000))
                                for kind,v in row.items()} for n,row in self.diagnostics.items()}
                    with self.lock:
                        self.view = dict(self.core.snapshot(), simulated=self.simulate,
                                         log_path=str(self.log_path), diagnostics=diag)
                if now-self.last_flush >= .1:
                    self.log_file.flush()
                    self.last_flush = now
        finally:
            self.core.stop('上位机退出，停止并保持位置', disarm=True)
            # Do not send OFF: releasing the Z axis could drop the mechanism.
            # No PING follows; firmware's 500 ms watchdog is the fallback.
            if self.adapter:
                try:
                    self.close_adapter()
                except Exception:
                    pass
            self.log_file.close()


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config', type=Path, default=ROOT / 'config/manual-jog.json')
    p.add_argument('--port', type=int, default=8090)
    p.add_argument('--simulate', action='store_true')
    p.add_argument('--log-dir', type=Path, default=ROOT / 'output/manual-control')
    args = p.parse_args()
    worker = Worker(json.loads(args.config.read_text(encoding='utf-8')), args.simulate, args.log_dir)
    key = secrets.token_urlsafe(32)
    address = f'127.0.0.1:{args.port}'
    page = (Path(__file__).parent / 'index.html').read_text(encoding='utf-8').replace('__CONTROL_KEY__', key).encode('utf-8')

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def reply(self, status, data, mime='application/json; charset=utf-8'):
            if not isinstance(data, bytes):
                data = json.dumps(data, ensure_ascii=False).encode('utf-8')
            self.send_response(status)
            self.send_header('Content-Type', mime)
            self.send_header('Content-Length', str(len(data)))
            self.send_header('Cache-Control', 'no-store')
            self.send_header('X-Frame-Options', 'DENY')
            self.send_header('X-Content-Type-Options', 'nosniff')
            self.end_headers()
            try:
                self.wfile.write(data)
            except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
                pass

        def do_GET(self):
            if self.headers.get('Host') != address:
                return self.reply(403, {'error': '请使用本机 127.0.0.1 地址'})
            path = urlsplit(self.path).path
            if path == '/':
                return self.reply(200, page, 'text/html; charset=utf-8')
            if path == '/api/state':
                with worker.lock:
                    view = worker.view
                return self.reply(200, view)
            if path.startswith('/api/'):
                return self.reply(404, {'error': '不存在'})
            self.send_response(302)
            self.send_header('Location', '/')
            self.send_header('Content-Length', '0')
            self.end_headers()

        def do_POST(self):
            if self.path != '/api/action' or self.headers.get('Host') != address:
                return self.reply(403, {'error': '无效地址'})
            origin = self.headers.get('Origin')
            if origin and origin != 'http://' + address:
                return self.reply(403, {'error': '不允许跨站控制'})
            try:
                size = int(self.headers.get('Content-Length', '0'))
                if not 0 < size <= 2048:
                    raise ValueError('请求长度无效')
                body = json.loads(self.rfile.read(size))
                if not isinstance(body, dict) or body.pop('key', None) != key:
                    return self.reply(403, {'error': '控制凭据不匹配'})
                worker.submit(body)
                self.reply(200, {'ok': True})
            except (ValueError, TypeError, queue.Full) as exc:
                self.reply(409, {'error': str(exc) or '操作队列已满'})

    server = ThreadingHTTPServer(('127.0.0.1', args.port), Handler)
    server.daemon_threads = True
    worker.thread.start()
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))
    print(f'http://{address}/  mode={"SIMULATED" if args.simulate else "REAL / passive until armed"}', flush=True)
    try:
        server.serve_forever(poll_interval=.1)
    except KeyboardInterrupt:
        pass
    finally:
        worker.shutdown.set()
        worker.thread.join(2)
        server.server_close()


if __name__ == '__main__':
    main()
