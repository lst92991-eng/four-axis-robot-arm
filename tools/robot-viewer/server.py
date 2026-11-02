"""只桥接一个 C++ 进程；棋规、引擎和运动确认全部由 C++ 负责。"""
import json
import base64
import queue
import re
import signal
import subprocess
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

ROOT = Path(__file__).resolve().parent
PORT = 8767
COMMAND = re.compile(r"(?:new|pause|resign|draw|scan|quit|move [a-i][0-9][a-i][0-9]|demo (?:[1-9]|1[0-9]|20))")


class Bridge:
    def __init__(self, config):
        if not isinstance(config, dict):
            raise ValueError("connection.json 必须是 JSON 对象")
        args = config.get("argv")
        if not isinstance(args, list) or not args or not all(isinstance(x, str) and x and "\0" not in x for x in args):
            raise ValueError("connection.json 的 argv 必须是非空命令数组")
        self.lock, self.commands = threading.Lock(), queue.Queue(maxsize=8)
        self.closed = False
        self.images, self.image_version = {}, 0
        self.state = dict(connected=False, phase="waiting", message="等待 C++ 状态")
        self.process = subprocess.Popen(args, cwd=config.get("cwd") or None, shell=False,
                                        stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                        stderr=subprocess.DEVNULL, text=True, encoding="utf-8",
                                        creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0))
        threading.Thread(target=self.read, daemon=True).start()
        threading.Thread(target=self.write, daemon=True).start()

    def failed(self, message):
        with self.lock:
            if not self.closed:
                self.state = {**self.state, "connected": False, "phase": "error", "message": message}

    def read(self):
        try:
            while 1:
                line = self.process.stdout.readline(1048577)
                if not line:
                    break
                if len(line) > 1048576:
                    raise ValueError("C++ 状态行过长")
                state = json.loads(line)
                if not isinstance(state, dict):
                    raise ValueError("C++ 状态必须是 JSON 对象")
                if "camera_jpeg" in state:
                    images = {name: base64.b64decode(state[name+"_jpeg"], validate=True)
                              for name in ("camera", "board")}
                    with self.lock:
                        self.images = images
                        self.image_version += 1
                    continue
                state["connected"] = True
                with self.lock:
                    self.state = state
            self.failed("C++ 进程已结束，请检查连接")
        except (OSError, ValueError):
            self.failed("C++ 状态读取失败，请检查连接或输出格式")

    def write(self):
        try:
            while 1:
                command = self.commands.get()
                if self.closed and command != "quit":
                    continue
                self.process.stdin.write(command + "\n")
                self.process.stdin.flush()
                if command == "quit":
                    return
        except (OSError, ValueError):
            self.failed("无法向 C++ 进程发送命令")

    def command(self, text):
        if not isinstance(text, str) or not COMMAND.fullmatch(text):
            raise ValueError("命令不在允许列表内")
        if self.closed or self.process.poll() is not None:
            raise RuntimeError("C++ 进程未连接")
        try:
            self.commands.put_nowait(text)
        except queue.Full:
            raise RuntimeError("命令队列繁忙，请稍后再试") from None

    def snapshot(self):
        with self.lock:
            return {**self.state, "image_version": self.image_version,
                    "camera_image": bool(self.images.get("camera")),
                    "board_image": bool(self.images.get("board"))}

    def image(self, name):
        with self.lock:
            return self.images.get(name, b"")

    def close(self):
        self.closed = True
        # 丢掉未发送操作；只请求退出，不在停机时继续走棋。
        while 1:
            try:
                self.commands.get_nowait()
            except queue.Empty:
                break
        self.commands.put_nowait("quit")
        try:
            self.process.wait(timeout=2)
        except subprocess.TimeoutExpired:
            self.process.terminate()
            try:
                self.process.wait(timeout=1)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait(timeout=1)


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *args):
        pass

    def setup(self):
        super().setup()
        self.connection.settimeout(3)

    def reply(self, status, value, kind="application/json; charset=utf-8"):
        body = value if isinstance(value, bytes) else json.dumps(value, ensure_ascii=False).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", kind)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.end_headers()
        try:
            self.wfile.write(body)
        except (BrokenPipeError, ConnectionResetError):
            pass

    def local(self):
        return self.headers.get("Host") in (f"127.0.0.1:{PORT}", f"localhost:{PORT}")

    def do_GET(self):
        if not self.local():
            return self.reply(403, {"error": "仅接受本机访问"})
        if self.path == "/api/state":
            return self.reply(200, self.server.bridge.snapshot())
        path = self.path.split("?", 1)[0]
        if path in ("/camera.jpg", "/board.jpg"):
            body = self.server.bridge.image(path[1:-4])
            return self.reply(200 if body else 404, body, "image/jpeg")
        if self.path in ("/", "/index.html"):
            return self.reply(200, (ROOT / "index.html").read_bytes(), "text/html; charset=utf-8")
        self.reply(404, {"error": "页面不存在"})

    def do_POST(self):
        if not self.local() or self.headers.get("Origin") not in (None, "http://" + self.headers.get("Host", "")) or self.headers.get("Sec-Fetch-Site") == "cross-site":
            return self.reply(403, {"error": "仅接受同源命令"})
        if self.path != "/api/command":
            return self.reply(404, {"error": "接口不存在"})
        try:
            length = int(self.headers.get("Content-Length", "0"))
            if not 0 < length <= 128 or self.headers.get_content_type() != "application/json" or self.headers.get("Transfer-Encoding"):
                raise ValueError("需要长度不超过128字节的 JSON 命令")
            data = json.loads(self.rfile.read(length))
            if not isinstance(data, dict) or set(data) != {"command"}:
                raise ValueError("请求只能包含 command")
            self.server.bridge.command(data["command"])
            self.reply(200, {"accepted": True})
        except (ValueError, UnicodeError) as error:
            self.reply(400, {"error": str(error)})
        except RuntimeError as error:
            self.reply(409, {"error": str(error)})
        except OSError:
            self.reply(408, {"error": "命令读取超时"})


def stop(signum, frame):
    raise KeyboardInterrupt


if __name__ == "__main__":
    bridge = None
    try:
        config = json.loads((ROOT / "connection.json").read_text(encoding="utf-8-sig"))
        bridge = Bridge(config)
        signal.signal(signal.SIGTERM, stop)
        with ThreadingHTTPServer(("127.0.0.1", PORT), Handler) as server:
            server.bridge = bridge
            print(f"机械臂查看器：http://127.0.0.1:{PORT}", flush=True)
            server.serve_forever(poll_interval=.2)
    except KeyboardInterrupt:
        pass
    except (OSError, ValueError) as error:
        print(f"查看器未启动：{error}")
    finally:
        if bridge:
            bridge.close()
