"""Feed a timed local camera stream to C++ tests; no physical camera is opened."""
import argparse
import os
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
import subprocess
import threading
import tempfile
import time

ROOT = Path(__file__).resolve().parents[1]
FRAMES = [(ROOT/'tests/fixtures/camera'/f'{i}.jpg').read_bytes() for i in range(8)]


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *_):
        pass

    def do_GET(self):
        if self.path.endswith('/not-a-stream'):
            self.send_response(404)
            self.end_headers()
            return
        self.send_response(200)
        self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
        self.end_headers()
        index = 0
        try:
            while True:
                data = FRAMES[index % len(FRAMES)]
                self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\nContent-Length: '+
                                 str(len(data)).encode()+b'\r\n\r\n'+data+b'\r\n')
                self.wfile.flush()
                index += 1
                if self.path.endswith('/finite') and index==12:
                    return
                if self.path.endswith('/stall') and index==12:
                    time.sleep(15)
                    return
                time.sleep(.1)
        except (ConnectionError, OSError):
            pass


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('executable')
    parser.add_argument('--entry', action='store_true')
    args = parser.parse_args()
    server = ThreadingHTTPServer(('127.0.0.1', 0), Handler)
    worker = threading.Thread(target=server.serve_forever, daemon=True)
    worker.start()
    url = f'http://127.0.0.1:{server.server_port}/stream.mjpg'
    command = [args.executable, url]
    if args.entry:
        command = [args.executable, 'mock.rknn', url]
    try:
        if args.entry:
            # 在空目录中运行，只从管道接收 JPEG，故障时仍回收 SDK 资源。
            with tempfile.TemporaryDirectory() as cwd:
                def entry(cmd, code, runs, env=None):
                    result = subprocess.run(cmd, cwd=cwd, env=env, capture_output=True, timeout=20)
                    expected = f'TEST cleanup: allocations=0 runs={runs}'.encode()
                    if result.returncode != code or expected not in result.stderr:
                        raise AssertionError((result.returncode, result.stderr[-600:]))
                    assert not list(Path(cwd).iterdir()), 'Vision wrote files in the working directory'
                    return result.stdout
                entry([command[0], command[1], url+'/not-a-stream'], 1, 0)
                for stage in ('query','allocate','bind','run'):
                    env = dict(os.environ, SCARA_MOCK_FAILURE=stage)
                    result = subprocess.run(command, cwd=cwd, env=env, capture_output=True, timeout=15)
                    assert result.returncode == 1 and b'TEST cleanup: allocations=0' in result.stderr, stage
                frames = entry(command, 0, 3)
                assert frames.startswith(b'\xff\xd8') and frames.endswith(b'\xff\xd9')
                assert frames.count(b'\xff\xd8') == frames.count(b'\xff\xd9') == 3
                entry(command, 0, 0, dict(os.environ, SCARA_MOCK_CANCEL_INIT='1'))
            return 0
        return subprocess.run(command, cwd=ROOT, timeout=30).returncode
    finally:
        server.shutdown()
        server.server_close()
        worker.join()


if __name__ == '__main__':
    raise SystemExit(main())
