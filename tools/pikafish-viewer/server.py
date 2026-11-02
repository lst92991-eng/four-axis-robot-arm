"""Local chess viewer. All legal moves and AI moves come from the RV1106 CPU."""
import copy
import json
import queue
import re
import subprocess
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

PORT = 8766
ROOT = Path(__file__).resolve().parents[2]
START = ("rnbakabnr", ".........", ".c.....c.", "p.p.p.p.p", ".........",
         ".........", "P.P.P.P.P", ".C.....C.", ".........", "RNBAKABNR")
MOVE = re.compile(r"^[a-i][0-9][a-i][0-9]$")


def fresh_board():
    return [["" if p == "." else p for p in row] for row in START]


def play(board, move):
    x, y, nx, ny = ord(move[0]) - 97, 9 - int(move[1]), ord(move[2]) - 97, 9 - int(move[3])
    piece, captured = board[y][x], board[ny][nx]
    board[ny][nx], board[y][x] = piece, ""
    return piece, captured


def analysis():
    return dict(depth=0, nodes=0, nps=0, score=None, pv=[])


class Engine:
    def __init__(self):
        self.lines, self.write_lock = queue.Queue(), threading.Lock()
        args = ["ssh", "-T", "-b", "192.168.55.1", "-i",
                str(Path.home() / ".ssh/rv1106_codex_auto"),
                "-o", "BatchMode=yes", "-o", "IdentitiesOnly=yes",
                "-o", "StrictHostKeyChecking=yes", "-o", "ConnectTimeout=8",
                "-o", "ServerAliveInterval=3", "-o", "ServerAliveCountMax=2",
                "-o", "UserKnownHostsFile=" + str(ROOT / "output/rv1106/pikafish-validation/network/known_hosts"),
                "root@192.168.55.6", "cd /root/scara-pikafish-test/2023-03-05 && ulimit -v 140000 && exec ./pikafish"]
        self.process = subprocess.Popen(args, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                        stderr=subprocess.STDOUT, text=True, encoding="utf-8",
                                        errors="replace", bufsize=1,
                                        creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0))
        threading.Thread(target=self.read, daemon=True).start()

    def read(self):
        try:
            for line in self.process.stdout:
                self.lines.put(line.strip())
        finally:
            self.lines.put(None)

    def send(self, command):
        with self.write_lock:
            if self.process.poll() is not None:
                raise RuntimeError("开发板引擎已断开，请检查连接后重启查看器")
            self.process.stdin.write(command + "\n")
            self.process.stdin.flush()

    def until(self, end, seconds, on_line=lambda line: None):
        deadline = time.monotonic() + seconds
        while True:
            try:
                line = self.lines.get(timeout=max(0.01, deadline - time.monotonic()))
            except queue.Empty:
                raise RuntimeError("开发板引擎响应超时，请检查连接后重启查看器") from None
            if line is None:
                raise RuntimeError("开发板引擎已断开，请检查供电和以太网连接后重启查看器")
            on_line(line)
            if end(line):
                return line
            if time.monotonic() >= deadline:
                raise RuntimeError("开发板引擎响应超时，请重启查看器")

    def connect(self):
        self.send("uci")
        self.until(lambda s: s == "uciok", 20)
        self.send("setoption name Threads value 1")
        self.send("setoption name Hash value 4")
        self.send("isready")
        self.until(lambda s: s == "readyok", 20)

    def position(self, history):
        self.send("position startpos" + (" moves " + " ".join(history) if history else ""))

    def legal(self, history):
        self.position(history)
        self.send("go perft 1")
        moves = []
        end = self.until(lambda s: s.startswith("Nodes searched:"), 15,
                         lambda s: moves.append(s[:4]) if re.fullmatch(r"[a-i][0-9][a-i][0-9]: 1", s) else None)
        if end.split(":", 1)[1].strip() != str(len(moves)):
            raise RuntimeError("板端合法着法列表不完整，棋局已停止，请重启查看器")
        return moves

    def stop(self):
        try:
            self.send("stop")
        except (OSError, RuntimeError):
            pass

    def close(self):
        try:
            self.send("stop\nquit")
            self.process.wait(timeout=4)
        except (OSError, RuntimeError, subprocess.TimeoutExpired):
            if self.process.poll() is None:
                self.process.terminate()


class Game:
    def __init__(self):
        self.lock, self.wake = threading.RLock(), threading.Event()
        self.engine = None
        self.closed, self.dirty, self.running, self.step = False, True, False, False
        self.not_before, self.budget = 0, 120
        self.state = dict(board=fresh_board(), turn="red", status="connecting", mode="auto",
                          legal=[], history=[], last_move=None, thinking_ms=1000, connected=False,
                          error=None, engine="Pikafish 2023-03-05", device="RV1106", revision=0,
                          analysis=analysis(), message="正在连接 RV1106 板端引擎…")
        self.positions = [self.position_key()]
        self.worker = threading.Thread(target=self.run, daemon=True)
        self.worker.start()

    def position_key(self):
        return tuple(tuple(row) for row in self.state["board"]), self.state["turn"]

    def snapshot(self):
        with self.lock:
            return copy.deepcopy(self.state)

    def cancel(self):
        self.state["revision"] += 1
        self.dirty = True
        if self.engine:
            self.engine.stop()

    def commit(self, move, elapsed=0):
        s = self.state
        piece, captured = play(s["board"], move)
        s["history"].append(dict(move=move, piece=piece, captured=captured, elapsed_ms=elapsed,
                                 **{k: s["analysis"][k] if elapsed else 0 for k in ("depth", "nodes", "nps")}))
        s["turn"] = "black" if s["turn"] == "red" else "red"
        s["last_move"], s["legal"] = move, []
        s["revision"] += 1
        self.dirty = True
        self.positions.append(self.position_key())
        s["status"] = "ready" if self.running else "paused"
        s["message"] = "红方请落子" if s["mode"] == "human" and s["turn"] == "red" else "等待下一步"
        if self.positions.count(self.positions[-1]) >= 3:
            self.running, self.step = False, False
            s["status"], s["message"] = "paused", "重复局面，已暂停"
        if s["mode"] == "auto" and self.running:
            self.budget -= 1
            if self.budget <= 0:
                self.running = False
                s["status"], s["message"] = "paused", "已观战 120 步，暂停休息；可继续播放"
        self.not_before = time.monotonic() + 0.7
        self.wake.set()

    def action(self, data):
        if not isinstance(data, dict):
            raise ValueError("请求必须是 JSON 对象")
        action = data.get("action")
        if action not in ("new", "mode", "step", "pause", "resume", "move", "undo", "time"):
            raise ValueError("未知操作")
        with self.lock:
            s = self.state
            if action == "time":
                ms = data.get("ms")
                if type(ms) is not int or ms not in (1000, 3000, 5000):
                    raise ValueError("思考时间只能是 1000、3000 或 5000 毫秒")
                s["thinking_ms"] = ms
                return self.snapshot()
            if s["status"] in ("connecting", "error"):
                raise ValueError("请等待板端连接完成" if s["status"] == "connecting" else "板端连接异常，请重启查看器")
            if action == "move":
                move = data.get("move")
                if s["mode"] != "human" or s["turn"] != "red" or s["status"] != "ready" or self.dirty:
                    raise ValueError("当前不能落子，请等待红方回合或继续棋局")
                if not isinstance(move, str) or not MOVE.fullmatch(move) or move not in s["legal"]:
                    raise ValueError("这一步不是合法着法")
                self.commit(move)
                return self.snapshot()
            if action in ("new", "mode"):
                mode = data.get("mode", "auto" if action == "new" else None)
                if mode not in ("auto", "human"):
                    raise ValueError("模式只能是 auto 或 human")
            if action in ("resume", "step") and s["status"] == "ended":
                raise ValueError("棋局已结束，请新开一局或悔棋")
            if action == "undo" and not s["history"]:
                raise ValueError("还没有可以悔的棋")
            self.cancel()
            self.step = action == "step"
            if action == "new":
                s.update(board=fresh_board(), turn="red", history=[], last_move=None, legal=[], analysis=analysis())
                self.positions = [self.position_key()]
            if action in ("new", "mode"):
                s["mode"], self.running = mode, mode == "human"
            elif action == "undo":
                count = 2 if s["mode"] == "human" and len(s["history"]) % 2 == 0 else 1
                s["history"] = s["history"][:-count]
                s.update(board=fresh_board(), turn="red", legal=[], analysis=analysis())
                self.positions = [self.position_key()]
                for item in s["history"]:
                    play(s["board"], item["move"])
                    s["turn"] = "black" if s["turn"] == "red" else "red"
                    self.positions.append(self.position_key())
                s["last_move"] = s["history"][-1]["move"] if s["history"] else None
                self.running = s["mode"] == "human"
            else:
                self.running = action == "resume"
            self.budget, self.not_before = 120, time.monotonic()
            s["status"] = "ready" if self.running or self.step else "paused"
            s["message"] = "红方请落子" if self.running and s["mode"] == "human" and s["turn"] == "red" else "准备继续" if self.running or self.step else "已暂停，点击播放或单步"
            self.wake.set()
            return self.snapshot()

    def info(self, line, revision):
        if not line.startswith("info "):
            return
        with self.lock:
            if revision != self.state["revision"]:
                return
            a, words = self.state["analysis"], line.split()
            for key in ("depth", "nodes", "nps"):
                match = re.search(r"\b" + key + r" (\d+)\b", line)
                if match:
                    a[key] = int(match[1])
            score = re.search(r"\bscore (cp|mate) (-?\d+)", line)
            if score:
                a["score"] = score[1] + " " + score[2]
            if "pv" in words:
                a["pv"] = [m for m in words[words.index("pv") + 1:] if MOVE.fullmatch(m)]

    def run(self):
        try:
            self.engine = Engine()
            self.engine.connect()
            with self.lock:
                self.state.update(connected=True, status="paused", message="板端已连接，点击播放观战或选择人机对弈")
            while not self.closed:
                self.wake.wait(0.15)
                self.wake.clear()
                with self.lock:
                    s = self.state
                    if self.engine.process.poll() is not None:
                        raise RuntimeError("开发板引擎已断开，请检查连接后重启查看器")
                    revision = s["revision"]
                    history = [item["move"] for item in s["history"]]
                    refresh = self.dirty
                    if refresh:
                        self.dirty = False
                    elif not (self.step or self.running and (s["mode"] == "auto" or s["turn"] == "black")) or time.monotonic() < self.not_before:
                        continue
                    else:
                        self.step = False
                        s.update(status="thinking", analysis=analysis(), message=("红方" if s["turn"] == "red" else "黑方") + " · RV1106 正在思考")
                        ms = s["thinking_ms"]
                if refresh:
                    legal = self.engine.legal(history)
                    with self.lock:
                        if revision == self.state["revision"]:
                            self.state["legal"] = legal
                            if not legal:
                                self.running, self.step = False, False
                                self.state.update(status="ended", message="当前一方无合法着法，棋局结束")
                    continue
                self.engine.position(history)
                started = time.monotonic()
                # Serialize go with cancellation: a pause cannot send stop before this go.
                with self.lock:
                    if revision != self.state["revision"]:
                        continue
                    self.engine.send("go movetime " + str(ms))
                result = self.engine.until(lambda line: line.startswith("bestmove "), ms / 1000 + 20,
                                           lambda line: self.info(line, revision))
                with self.lock:
                    if revision == self.state["revision"]:
                        move = result.split()[1]
                        if move not in self.state["legal"]:
                            raise RuntimeError("板端返回了无效着法，棋局已停止，请重启查看器")
                        self.commit(move, round((time.monotonic() - started) * 1000))
        except Exception as exc:
            with self.lock:
                self.running, self.step = False, False
                message = str(exc) if isinstance(exc, RuntimeError) else "无法连接或读取 RV1106 引擎，请检查开发板和 SSH 配置后重启查看器"
                self.state.update(status="error", connected=False, error=message, message=message, legal=[])
        finally:
            if self.engine:
                self.engine.close()

    def close(self):
        with self.lock:
            self.closed = True
            self.cancel()
        self.wake.set()
        self.worker.join(timeout=6)
        if self.worker.is_alive() and self.engine:
            self.engine.close()


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *args):
        pass  # Polling state every 400 ms should not flood the terminal.

    def setup(self):
        super().setup()
        self.connection.settimeout(5)

    def reply(self, code, data, kind="application/json; charset=utf-8"):
        body = data if isinstance(data, bytes) else json.dumps(data, ensure_ascii=False).encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", kind)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.end_headers()
        try:
            self.wfile.write(body)
        except (BrokenPipeError, ConnectionResetError):
            pass

    def same_host(self):
        return self.headers.get("Host") in ("127.0.0.1:" + str(PORT), "localhost:" + str(PORT))

    def do_GET(self):
        if not self.same_host():
            return self.reply(403, {"error": "仅接受本机访问"})
        if self.path == "/api/state":
            return self.reply(200, self.server.game.snapshot())
        if self.path in ("/", "/index.html"):
            try:
                return self.reply(200, Path(__file__).with_name("index.html").read_bytes(), "text/html; charset=utf-8")
            except OSError:
                return self.reply(500, {"error": "查看器页面文件缺失"})
        self.reply(404, {"error": "页面不存在"})

    def do_POST(self):
        origin = self.headers.get("Origin")
        if not self.same_host() or origin and origin != "http://" + self.headers.get("Host", "") or self.headers.get("Sec-Fetch-Site") == "cross-site":
            return self.reply(403, {"error": "仅接受同源请求"})
        if self.path != "/api/action":
            return self.reply(404, {"error": "接口不存在"})
        try:
            length = int(self.headers.get("Content-Length", "0"))
            if not 0 < length <= 2048 or self.headers.get_content_type() != "application/json" or self.headers.get("Transfer-Encoding"):
                raise ValueError("请求必须是长度不超过 2048 字节的 JSON")
            data = json.loads(self.rfile.read(length))
            self.reply(200, self.server.game.action(data))
        except (ValueError, UnicodeError) as exc:
            self.reply(400, {"error": str(exc) if not isinstance(exc, json.JSONDecodeError) else "JSON 格式错误"})
        except Exception:
            self.reply(500, {"error": "操作失败，请刷新页面查看连接状态"})


if __name__ == "__main__":
    server = ThreadingHTTPServer(("127.0.0.1", PORT), Handler)
    server.game = Game()
    print("Pikafish viewer: http://127.0.0.1:%d — RV1106 CPU / Threads 1 / Hash 4 MiB" % PORT, flush=True)
    try:
        server.serve_forever(poll_interval=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        server.game.close()
        server.server_close()
