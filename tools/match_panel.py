"""本机象棋界面。点击开局才启动实机；--mock 使用离线棋局。"""

import argparse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import os
from pathlib import Path
import re
import secrets
import signal
import subprocess
import sys
import threading
import time
import webbrowser


ROOT = Path(__file__).resolve().parents[1]
BENCH = ROOT / "output/whole-machine-20261117/usb-game"
OPENING = ["rnbakabnr", ".........", ".c.....c.", "p.p.p.p.p", ".........",
           ".........", "P.P.P.P.P", ".C.....C.", ".........", "RNBAKABNR"]
RECOVERY = {"confirm_start", "vision_recovery", "engine_recovery", "hardware_recovery",
            "placement_recovery", "restore_board", "human_retry"}
MOVE = re.compile(r"[a-i][0-9][a-i][0-9]")


class Match:
    def __init__(self, mock):
        self.mock = mock
        self.lock = threading.Lock()
        self.proc = None
        self.stamp = None
        self.state = {}
        self.detail = ""
        self.prompt = ""
        self.stopping = False
        self.console_stop = False

    def read_status(self, baseline=False):
        try:
            file = BENCH / "game-state/status.json"
            stat = file.stat()
            stamp = stat.st_mtime_ns, stat.st_size
            if baseline or stamp != self.stamp:
                if not baseline:
                    state = json.loads(file.read_text(encoding="utf-8"))
                    if state.get("simulated") == 0:
                        self.state = state
                self.stamp = stamp
        except (OSError, ValueError):
            if baseline:
                self.stamp = None

    def start(self, current):
        if self.proc is not None:
            raise ValueError("已有对局正在运行。")
        runner = BENCH / "run_game.py"
        if not self.mock and not runner.is_file():
            raise ValueError(f"找不到实机入口：{runner}")
        cmd, cwd = [sys.executable, "-u"], BENCH
        kwargs = {}
        args = ["current"] if current else []
        if self.mock:
            cmd += [str(Path(__file__).resolve()), "--mock-driver", *args]
            cwd = ROOT
        elif os.name == "nt":
            import ctypes

            self.console_stop = bool(ctypes.windll.kernel32.GetConsoleCP())
            wrapper = ("import runpy,signal,sys;"
                       "signal.signal(signal.SIGBREAK,signal.default_int_handler);"
                       "sys.argv=sys.argv[1:];runpy.run_path(sys.argv[0],run_name='__main__')")
            cmd += ["-c", wrapper, str(runner), *args]
            kwargs["creationflags"] = subprocess.CREATE_NEW_PROCESS_GROUP
        else:
            self.console_stop = True
            cmd += [str(runner), *args]
            kwargs["start_new_session"] = True
        if not self.mock:
            self.read_status(baseline=True)
        self.state, self.prompt, self.stopping = {}, "", False
        self.detail = "正在准备棋盘和观察位。"
        self.proc = subprocess.Popen(cmd, cwd=cwd, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                     stderr=subprocess.STDOUT, text=True, encoding="utf-8",
                                     errors="replace", bufsize=1,
                                     env={**os.environ, "PYTHONIOENCODING": "utf-8"}, **kwargs)
        threading.Thread(target=self.read_output, args=(self.proc,), daemon=True).start()

    def read_output(self, proc):
        with proc.stdout:
            for raw in proc.stdout:
                line = raw.strip()
                if not line:
                    continue
                with self.lock:
                    if self.mock and line.startswith("MOCK_STATE "):
                        self.state = json.loads(line[11:])
                        continue
                    prefix, _, text = line.partition(" ")
                    key = "human" if prefix == "HUMAN_TURN" else prefix.lower()
                    if key == "human" or key in RECOVERY:
                        self.prompt = key
                        if self.stopping:
                            try:
                                self.send("quit")
                            except OSError:
                                pass
                        else:
                            self.detail = text
                    elif not self.prompt and not self.stopping:
                        self.detail = line
        code = proc.wait()
        with self.lock:
            if not self.mock:
                self.read_status()
            self.proc, self.prompt, self.stopping = None, "", False
            if code:
                self.detail = f"对局已退出（{code}）。{self.detail}"
            self.state["phase"] = "stopped"
        proc.stdin.close()

    def snapshot(self):
        if self.proc and not self.mock:
            self.read_status()
        state = dict(self.state)
        state.update(active=self.proc is not None, prompt=self.prompt, detail=self.detail,
                     stopping=self.stopping, mock=self.mock)
        return state

    def send(self, command):
        self.proc.stdin.write(command + "\n")
        self.proc.stdin.flush()
        self.prompt = ""

    def move(self, move):
        if not isinstance(move, str) or not MOVE.fullmatch(move):
            raise ValueError("走法格式不正确。")
        if not self.proc or self.stopping or self.prompt != "human" or self.state.get("phase") != "wait_human":
            raise ValueError("请等到轮到红方。")
        legal = self.state.get("legal")
        if isinstance(legal, list) and move not in legal:
            raise ValueError("这步棋不在合法走法表里。")
        self.send(move)
        self.detail = f"已提交红方走法：{move[:2]} → {move[2:]}"

    def resume(self):
        if not self.proc or self.stopping or self.prompt not in RECOVERY:
            raise ValueError("程序没有在等待确认。")
        self.send("")
        self.detail = "正在重新读取实盘。"

    def stop(self):
        if not self.proc or self.stopping:
            return
        self.stopping = True
        self.detail = "等待程序停止并退出。"
        if self.prompt or self.mock:
            self.send("quit")
        elif self.console_stop:
            try:
                self.proc.send_signal(signal.CTRL_BREAK_EVENT if os.name == "nt" else signal.SIGINT)
            except OSError:
                self.detail = "程序进入下一次输入等待时会收到退出命令。"
        else:
            self.detail = "程序进入下一次输入等待时会收到退出命令。"


PAGE = """<!doctype html>
<html lang="zh-CN"><meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1">
<title>机械臂象棋</title>
<style>
*{box-sizing:border-box} body{margin:0;background:#f8f4eb;color:#32241a;font:16px "Microsoft YaHei",sans-serif}
main{max-width:560px;margin:auto;padding:19px} h1{margin:0 0 5px;font-size:25px}
#phase{color:#185c36;font-weight:bold} #detail{height:3em;overflow:auto}
svg{width:100%;display:block;background:#f0d3a2;border-radius:9px;border:1px solid #ad7b4c;touch-action:manipulation}
.controls{display:flex;flex-wrap:wrap;align-items:center;gap:8px;margin-top:12px}
button{font:inherit;border:1px solid #a67957;background:#fffaf2;color:#30231b;border-radius:6px;padding:7px 12px;cursor:pointer}
button:disabled{opacity:.42;cursor:default} input{vertical-align:middle} #start{background:#21603d;color:white}
#history{font:14px Consolas,monospace;overflow-wrap:anywhere} #message{color:#a02b1a;min-height:1.2em}
</style>
<main><h1>机械臂象棋 <small id="mock"></small></h1><div id="phase">等待开局</div>
<div id="detail">开局后机械臂会退到观察位。</div>
<svg id="board" viewBox="0 0 444 491" aria-label="象棋棋盘"></svg>
<div class="controls">
<button id="start">开局</button><label><input id="current" type="checkbox" checked>用当前盘面开局</label>
<button id="resume" disabled>确认</button><button id="stop" disabled>结束对局</button>
</div><p id="history">棋谱：—</p><p id="message" role="status"></p></main>
<script>
const token = "__TOKEN__";
const opening = __OPENING__;
const words = {observing:"正在识别棋盘",thinking:"皮卡鱼正在思考",wait_human:"轮到红方",
 executing:"机械臂正在落子",restore_board:"请复位棋子",placement_recovery:"请检查落子结果",
 vision_recovery:"请恢复相机画面",hardware_recovery:"请检查机械臂",engine_recovery:"请检查引擎",
 stopped:"对局已结束"};
const by = id => document.getElementById(id);
const board = by("board"), glyph = Object.fromEntries(Array.from("kabnrcpKABNRCP",
 (p,i)=>[p,"將士象馬車砲卒帥仕相馬車炮兵"[i]]));
let state = {}, selected = "", posting = false, drawn = "", ticket = 0;
function node(tag, attrs, text) {
 const el=document.createElementNS("http://www.w3.org/2000/svg",tag);
 Object.keys(attrs).forEach(k=>el.setAttribute(k,attrs[k]));
 if(text) el.textContent=text;
 board.append(el); return el;
}
const sx=c=>34+47*c, sy=r=>34+47*r;
const sq=(c,r)=>String.fromCharCode(97+c)+(9-r);
const ready=()=>state.active && !state.stopping && state.prompt==="human" && state.phase==="wait_human";
function draw() {
 const pieces=state.prompt==="confirm_start" && state.observation_valid ? state.observed : state.board;
 const rows=Array.isArray(pieces) && pieces.length===10 ? pieces : opening;
 const last=(state.history||"").split(" ").pop();
 const goals=selected && Array.isArray(state.legal) ? state.legal.filter(m=>m.startsWith(selected)).map(m=>m.slice(2)) : [];
 const signature=JSON.stringify([rows,last,selected,goals]);
 if(signature===drawn) return;
 drawn=signature; board.replaceChildren();
 const line=(x1,y1,x2,y2)=>node("line",{x1,y1,x2,y2,stroke:"#875636"});
 const label=(x,y,text,size=13)=>node("text",{x,y,fill:"#875636","text-anchor":"middle","font-size":size},text);
 for(let r=0;r<10;r++){line(sx(0),sy(r),sx(8),sy(r));label(12,sy(r)+4,String(9-r));}
 for(let c=0;c<9;c++){
  line(sx(c),sy(0),sx(c),sy(4));line(sx(c),sy(5),sx(c),sy(9));
  if(c===0||c===8) line(sx(c),sy(4),sx(c),sy(5));
  label(sx(c),sy(9)+27,String.fromCharCode(97+c));
 }
 for(const r of [0,7]){line(sx(3),sy(r),sx(5),sy(r+2));line(sx(5),sy(r),sx(3),sy(r+2));}
 label(sx(4),sy(4.5)+7,"楚 河　　漢 界",19);
 for(let r=0;r<10;r++) for(let c=0;c<9;c++) {
  const p=rows[r][c], name=sq(c,r), cx=sx(c), cy=sy(r), red=/[A-Z]/.test(p);
  if(name===selected||name===last.slice(0,2)||name===last.slice(2))
   node("circle",{cx,cy,r:21.5,fill:"none",stroke:name===selected?"#a16813":"#387743","stroke-width":2.5});
  if(glyph[p]){
   const color=red?"#a72921":"#332318";
   node("circle",{cx,cy,r:17.5,fill:"#fff1d2",stroke:color,"stroke-width":1.6});
   node("text",{x:cx,y:cy+8,fill:color,"text-anchor":"middle","font-size":23,"font-weight":"bold"},glyph[p]);
  }
  if(goals.includes(name)) node("circle",{cx:cx+13,cy:cy+13,r:4.5,fill:"#246740",stroke:"white"});
 }
}
function render() {
 if(!ready()) selected="";
 by("mock").textContent=state.mock?"· 离线": "";
 by("phase").textContent=state.stopping?"正在结束":state.prompt==="confirm_start"?"核对当前盘面":
   words[state.phase] || (state.active ? "对局启动中" : "等待开局");
 by("detail").textContent=ready() ? (selected
   ? "点击绿色标出的目标格。机械臂会先走红方，再替黑方应手。"
   : "点击要走的红棋，再选绿色目标格。红黑双方都由机械臂落子。")
   : state.error || state.detail || "开局后机械臂会退到观察位。";
 by("history").textContent="棋谱："+(state.history||"—");
 by("start").disabled=posting||state.active;
 by("current").disabled=posting||state.active;
 by("resume").disabled=posting||state.stopping||!state.active||!state.prompt||state.prompt==="human";
 by("resume").textContent=state.prompt==="confirm_start"?"确认当前盘面":"恢复后继续";
 by("stop").disabled=posting||state.stopping||!state.active;
 draw();
}
async function getState() {
 const request=ticket;
 try {
  const next=await (await fetch("/state",{cache:"no-store"})).json();
  if(request===ticket&&!posting){state=next;render();}
 } catch(_) { by("message").textContent="已断开本机面板。"; }
}
async function post(action,data={}) {
 if(posting)return;
 ++ticket; posting=true; by("message").textContent=""; render();
 try {
  const res=await fetch("/"+action,{method:"POST",headers:{"Content-Type":"application/json","X-Panel":token},
   body:JSON.stringify(data)});
  const result=await res.json();
  if(res.ok){state=result;selected="";}
  else by("message").textContent=result.error;
 } catch(_) { by("message").textContent="命令发送失败。"; }
 posting=false;render();
}
board.onclick=e=>{
 if(posting||!ready())return;
 const pt=new DOMPoint(e.clientX,e.clientY).matrixTransform(board.getScreenCTM().inverse());
 const c=Math.round((pt.x-34)/47),r=Math.round((pt.y-34)/47);
 if(c<0||c>8||r<0||r>9)return;
 const p=state.board[r][c], name=sq(c,r);
 if(/[A-Z]/.test(p)){
  if(Array.isArray(state.legal)&&!state.legal.some(move=>move.startsWith(name))){
   by("message").textContent="这枚棋子现在没有合法走法。";return;
  }
  selected=selected===name?"":name;
  by("message").textContent="";render();return;
 }
 if(selected){
  const move=selected+name;
  if(Array.isArray(state.legal)&&!state.legal.includes(move)){
   by("message").textContent="请选择绿色标出的目标格。";return;
  }
  post("move",{move});
 }
};
by("start").onclick=()=>post("start",{current:by("current").checked});
by("resume").onclick=()=>post("resume");
by("stop").onclick=()=>post("stop");
render();getState();setInterval(getState,320);
</script></html>"""


def handler_for(match, token):
    page = PAGE.replace("__TOKEN__", token).replace("__OPENING__", json.dumps(OPENING))

    class Handler(BaseHTTPRequestHandler):
        def local_host(self):
            return f"127.0.0.1:{self.server.server_port}"

        def respond(self, status, content, mime="application/json; charset=utf-8"):
            if not isinstance(content, str):
                content = json.dumps(content, ensure_ascii=False)
            data = content.encode("utf-8")
            self.send_response(status)
            self.send_header("Content-Type", mime)
            self.send_header("Content-Length", str(len(data)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(data)

        def do_GET(self):
            if self.headers.get("Host") != self.local_host():
                self.respond(403, {"error": "请使用面板显示的本机地址。"})
            elif self.path == "/":
                self.respond(200, page, "text/html; charset=utf-8")
            elif self.path == "/state":
                with match.lock:
                    state = match.snapshot()
                self.respond(200, state)
            else:
                self.respond(404, {"error": "页面不存在。"})

        def do_POST(self):
            origin = self.headers.get("Origin")
            local = self.local_host()
            if (self.headers.get("Host") != local or origin != f"http://{local}" or
                    not secrets.compare_digest(self.headers.get("X-Panel", "").encode("latin-1"), token.encode())):
                self.respond(403, {"error": "请从本机面板操作。"})
                return
            try:
                size = int(self.headers.get("Content-Length", "0"))
                if not 0 < size <= 256:
                    raise ValueError("命令长度不正确。")
                data = json.loads(self.rfile.read(size))
                if not isinstance(data, dict):
                    raise ValueError("命令数据不正确。")
                with match.lock:
                    match.snapshot()
                    if self.path == "/start":
                        match.start(data.get("current") is True)
                    elif self.path == "/move":
                        match.move(data.get("move"))
                    elif self.path == "/resume":
                        match.resume()
                    elif self.path == "/stop":
                        match.stop()
                    else:
                        raise ValueError("命令不存在。")
                    state = match.snapshot()
                self.respond(200, state)
            except (OSError, ValueError) as exc:
                self.respond(409, {"error": str(exc)})

        def log_message(self, *_):
            pass

    return Handler


def mock_driver(current):
    board, history = [list(row) for row in OPENING], []
    if current:
        print("MOCK_STATE " + json.dumps(dict(simulated=1, phase="observing", observed=OPENING,
                                             observation_valid=1)), flush=True)
        print("CONFIRM_START 请确认当前盘面。", flush=True)
        if sys.stdin.readline().strip() == "quit":
            return
    while True:
        legal = ["h2f2", "a3a4"] if not history else ["c3c4"]
        state = dict(simulated=1, board=["".join(row) for row in board], phase="wait_human",
                     history=" ".join(history), legal=legal)
        print("MOCK_STATE " + json.dumps(state), flush=True)
        print("HUMAN_TURN 离线演示", flush=True)
        move = sys.stdin.readline().strip()
        if not move or move == "quit":
            return
        if move in legal:
            for step in (move, "b7e7") if not history else (move,):
                a, b, c, d = step
                src, dst = board[9 - int(b)], board[9 - int(d)]
                x, y = ord(a) - 97, ord(c) - 97
                dst[y], src[x] = src[x], "."
                history.append(step)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mock", action="store_true")
    parser.add_argument("--mock-driver", action="store_true", help=argparse.SUPPRESS)
    parser.add_argument("--port", type=int, default=8121)
    parser.add_argument("--no-browser", action="store_true")
    parser.add_argument("current", nargs="?", choices=["current"], help=argparse.SUPPRESS)
    args = parser.parse_args()
    if args.mock_driver:
        mock_driver(args.current == "current")
        return
    match = Match(args.mock)
    with ThreadingHTTPServer(("127.0.0.1", args.port), handler_for(match, secrets.token_hex(16))) as server:
        url = f"http://127.0.0.1:{server.server_port}/"
        print(url, flush=True)
        if not args.no_browser:
            webbrowser.open(url)
        try:
            server.serve_forever()
        except KeyboardInterrupt:
            with match.lock:
                match.stop()
            while match.proc:
                time.sleep(.1)


if __name__ == "__main__":
    main()
