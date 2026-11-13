"""Feed a timed local camera stream to C++ tests; no physical camera is opened."""
import argparse
import os
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
import subprocess
import threading
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
        command = [args.executable, 'mock.rknn', url, 'config/vision/chinese-labels.png',
                   'output/vision-stage-verify/entry.jpg']
    try:
        if args.entry:
            # The teaching entry assumes the bundled atlas. Exercise actual draw
            # failure after inference, rather than its removed startup size gate.
            for labels in ('missing-labels.png','tests/fixtures/vision/tiny-labels.pgm'):
                invalid = command.copy()
                invalid[3] = labels
                result = subprocess.run(invalid,cwd=ROOT,capture_output=True,text=True,timeout=10)
                if result.returncode!=1 or 'TEST cleanup: allocations=0 runs=1' not in result.stdout:
                    raise RuntimeError('Label drawing failure did not release resources: '+result.stderr+result.stdout)
            unavailable = command.copy()
            unavailable[2] = url+'/not-a-stream'
            result = subprocess.run(unavailable,cwd=ROOT,capture_output=True,text=True,timeout=15)
            if result.returncode!=1 or 'TEST cleanup: allocations=0 runs=0' not in result.stdout:
                raise RuntimeError('Camera-open failure did not release the model: '+result.stderr+result.stdout)
            for stage in ('query','allocate','bind','run'):
                env = dict(os.environ, SCARA_MOCK_FAILURE=stage)
                result = subprocess.run(command,cwd=ROOT,env=env,capture_output=True,text=True,timeout=15)
                if result.returncode!=1 or 'TEST cleanup: allocations=0' not in result.stdout:
                    raise RuntimeError('Application did not release resources after '+stage+': '+result.stderr+result.stdout)
            result = subprocess.run(command,cwd=ROOT,capture_output=True,text=True,timeout=30)
            if result.returncode!=0 or 'TEST cleanup: allocations=0 runs=3' not in result.stdout:
                raise RuntimeError('Continuous entry did not stop cleanly: '+result.stderr+result.stdout)
            return 0
        return subprocess.run(command, cwd=ROOT, timeout=30).returncode
    finally:
        server.shutdown()
        server.server_close()
        worker.join()


if __name__ == '__main__':
    raise SystemExit(main())
