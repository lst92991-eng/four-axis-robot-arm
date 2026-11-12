"""Feed a timed local camera stream to C++ tests; no physical camera is opened."""
import argparse
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
                   'output/vision-stage-verify/entry.jpg', '3']
    try:
        return subprocess.run(command, cwd=ROOT, timeout=30).returncode
    finally:
        server.shutdown()
        server.server_close()
        worker.join()


if __name__ == '__main__':
    raise SystemExit(main())
