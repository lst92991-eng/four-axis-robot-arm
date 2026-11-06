"""Temporary RKNN 1.6 RV1106 inference/preview service; no motor interfaces.

ABI reference: Rockchip rknn_api.h at model-zoo eaa94d6.
Uses the documented RV1106 native NHWC zero-copy API, not PyTorch.
"""
import argparse
import ctypes as C
import hashlib
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import signal
import struct
import threading
import time

MODEL_SHA = 'c843fac9714d97ca3b478193fb98557fef3146ca4c216a98380295a87c693943'
U32 = C.c_uint32


class Attr(C.Structure):
    _fields_ = [('index', U32), ('n_dims', U32), ('dims', U32 * 16),
                ('name', C.c_char * 256), ('n_elems', U32), ('size', U32),
                ('fmt', C.c_int), ('type', C.c_int), ('qnt_type', C.c_int),
                ('fl', C.c_int8), ('zp', C.c_int32), ('scale', C.c_float),
                ('w_stride', U32), ('size_with_stride', U32),
                ('pass_through', C.c_uint8), ('h_stride', U32)]


class Version(C.Structure):
    _fields_ = [('api', C.c_char * 256), ('driver', C.c_char * 256)]


def info(a):
    return dict(index=a.index, dims=list(a.dims)[:a.n_dims],
                name=a.name.decode(), elements=a.n_elems, size=a.size,
                fmt=a.fmt, type=a.type, qnt_type=a.qnt_type,
                zero_point=a.zp, scale=a.scale, w_stride=a.w_stride,
                allocation=a.size_with_stride)


class NPU:
    def __init__(self, path, expected_sha256=MODEL_SHA):
        assert C.sizeof(C.c_void_p) == 4 and C.sizeof(Attr) == 376, 'ARM32 ABI required'
        model_sha = hashlib.sha256(Path(path).read_bytes()).hexdigest()
        if model_sha != expected_sha256.lower():
            raise ValueError('Model SHA256 does not match the selected deployment')
        self.api = C.CDLL('/oem/usr/lib/librknnmrt.so')
        signatures = {
            'rknn_init': ([C.POINTER(U32), C.c_void_p, U32, U32, C.c_void_p], C.c_int),
            'rknn_query': ([U32, C.c_int, C.c_void_p, U32], C.c_int),
            'rknn_create_mem': ([U32, U32], C.c_void_p),
            'rknn_set_io_mem': ([U32, C.c_void_p, C.POINTER(Attr)], C.c_int),
            'rknn_run': ([U32, C.c_void_p], C.c_int),
            'rknn_destroy_mem': ([U32, C.c_void_p], C.c_int),
            'rknn_destroy': ([U32], C.c_int),
        }
        for name, (args, result) in signatures.items():
            fn = getattr(self.api, name)
            fn.argtypes, fn.restype = args, result
        self.ctx, self.memories = U32(), []
        try:
            self.check(self.api.rknn_init(C.byref(self.ctx), C.c_char_p(path.encode()), 0, 0, None))
            io = (U32 * 2)()
            self.query(0, io)
            assert list(io) == [1, 3], list(io)
            version = Version()
            self.query(5, version)
            self.version = dict(runtime=version.api.decode(), driver=version.driver.decode())
            self.input_attr = Attr(index=0)
            self.query(8, self.input_attr)
            assert list(self.input_attr.dims)[:4] == [1, 640, 640, 3], info(self.input_attr)
            self.input_attr.type, self.input_attr.fmt = 3, 1  # UINT8, NHWC; runtime normalizes.
            self.input_attr.pass_through = 0
            self.input_address = self.allocate(self.input_attr)
            self.outputs = []
            for index, width in enumerate((80, 40, 20)):
                a = Attr(index=index)
                self.query(11, a)
                assert list(a.dims)[:4] == [1, width, width, 57], info(a)
                assert a.fmt == 1 and a.type == 2 and a.qnt_type == 2 and a.scale > 0
                assert a.w_stride in (0, width) and a.n_elems == width*width*57
                self.outputs.append((a, self.allocate(a)))
            self.description = dict(model_sha256=model_sha, model_name=Path(path).name, version=self.version,
                input=info(self.input_attr), outputs=[info(a) for a, _ in self.outputs])
            print(json.dumps(self.description), flush=True)
        except BaseException:
            self.close()
            raise

    @staticmethod
    def check(result):
        if result != 0:
            raise RuntimeError('RKNN returned ' + str(result))

    def query(self, command, obj):
        self.check(self.api.rknn_query(self.ctx, command, C.byref(obj), C.sizeof(obj)))

    def allocate(self, attr):
        size = attr.size_with_stride
        assert size >= attr.n_elems and size < 16*1024*1024
        memory = self.api.rknn_create_mem(self.ctx, size)
        if not memory:
            raise MemoryError('rknn_create_mem')
        self.memories.append(memory)
        address = C.c_void_p.from_address(memory).value  # first field: virt_addr
        if not address:
            raise MemoryError('Null tensor address')
        C.memset(address, 0, size)
        self.check(self.api.rknn_set_io_mem(self.ctx, memory, C.byref(attr)))
        return address

    def run(self, data, frame_id):
        if len(data) != 640*640*3:
            raise ValueError('Expected exactly 640x640 RGB bytes')
        stride = self.input_attr.w_stride or 640
        assert stride >= 640 and stride*640*3 <= self.input_attr.size_with_stride
        if stride == 640:
            C.memmove(self.input_address, data, len(data))
        else:
            for y in range(640):
                C.memmove(self.input_address+y*stride*3, data[y*1920:(y+1)*1920], 1920)
        start = time.monotonic()
        self.check(self.api.rknn_run(self.ctx, None))
        elapsed = (time.monotonic()-start)*1000
        blobs = [C.string_at(address, attr.n_elems) for attr, address in self.outputs]
        header = json.dumps(dict(frame_id=frame_id, npu_ms=elapsed,
            outputs=[info(a) for a, _ in self.outputs])).encode()
        return struct.pack('<I', len(header)) + header + b''.join(blobs)

    def close(self):
        for memory in reversed(self.memories):
            self.api.rknn_destroy_mem(self.ctx, memory)
        self.memories = []
        if self.ctx.value:
            self.api.rknn_destroy(self.ctx)
            self.ctx = U32()


PAGE = '''<!doctype html><meta charset="utf-8"><title>象棋 YOLO 实时测试</title>
<style>body{margin:24px;background:#111827;color:#e5e7eb;font:18px sans-serif}img{max-width:96vw;max-height:80vh;border:1px solid #475569}#state{padding:12px 0;color:#fbbf24}</style>
<h2>象棋 YOLO · RV1106 NPU 实时测试</h2><div>只做视觉识别，不连接机械臂运动。类别框未经过棋规纠正。</div>
<div id="model">读取模型信息…</div><div id="state">等待识别画面…</div><img src="/stream.mjpg">
<script>fetch('/health',{cache:'no-store'}).then(r=>r.json()).then(m=>{document.querySelector('#model').textContent='当前模型：'+m.model_name+' · SHA256 '+m.model_sha256.slice(0,12);});</script>
<script>setInterval(async()=>{try{let s=await(await fetch('/status',{cache:'no-store'})).json();document.querySelector('#state').textContent=s.age_s===null?'等待识别画面':s.age_s>3?'画面已过期，请检查取流/推理进程':('实时画面 · '+s.age_s.toFixed(1)+' 秒前更新');}catch(e){document.querySelector('#state').textContent='连接中断';}},1000);</script>'''

DUAL_PAGE = '''<!doctype html><html lang="zh-CN"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>象棋双画面：识别与交点</title>
<style>body{margin:18px;background:#111827;color:#e5e7eb;font:17px sans-serif}h1{font-size:23px;margin:0 0 12px}.titles{display:grid;grid-template-columns:1fr 1fr;gap:12px;margin:14px 0 8px}.titles div{padding:10px;background:#1e293b;border-radius:6px}img{display:block;width:100%;height:auto;border:1px solid #475569;box-sizing:border-box}#state{color:#fbbf24}small{color:#94a3b8}main{max-width:1720px;margin:auto}</style>
<main><h1>象棋实时双画面</h1><div id="state">正在连接画面…</div>
<div class="titles"><div>左侧：YOLO 棋子识别</div><div>右侧：OpenCV 棋盘交点</div></div>
<img id="views" src="/dual/stream.mjpg" alt="左侧棋子识别，右侧棋盘交点，同一帧画面">
<p><small>两侧来自同一帧。只显示画面，不控制机械臂。黄色圆点是交点，红色数字是列、行编号。</small></p></main>
<script>const im=document.querySelector('#views');im.onerror=()=>setTimeout(()=>{im.src='/dual/stream.mjpg'},1500);
setInterval(async()=>{const el=document.querySelector('#state');try{let s=await(await fetch('/dual/status',{cache:'no-store'})).json();
el.textContent=s.age_s===null?'等待画面':s.age_s>3?'画面已暂停，请检查采集服务':('实时画面 · '+s.age_s.toFixed(1)+' 秒前更新 · '+(s.geometry_valid?'已找到 90 个交点':'暂未找到完整棋盘')+' · '+(s.orientation_valid?'朝向已确认':'朝向待确认'));
}catch(e){el.textContent='连接中断，正在等待恢复';}},1000);</script></html>'''


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--model', required=True)
    parser.add_argument('--model-sha256', default=MODEL_SHA)
    parser.add_argument('--bind', required=True)
    parser.add_argument('--client', required=True)
    parser.add_argument('--port', type=int, default=8088)
    args = parser.parse_args()
    npu = NPU(args.model, args.model_sha256)
    lock, condition = threading.Lock(), threading.Condition()
    preview = dict(jpeg=None, updated=None, frame_id=None)
    dual = dict(jpeg=None, updated=None, frame_id=None, geometry_valid=False, orientation_valid=False)

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *_):
            pass

        def reply(self, body, kind='application/json', code=200):
            self.send_response(code)
            self.send_header('Content-Type', kind)
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            view = dual if self.path.startswith('/dual/') else preview
            if self.path == '/':
                self.reply(PAGE.encode(), 'text/html; charset=utf-8')
            elif self.path in ('/dual','/dual/'):
                self.reply(DUAL_PAGE.encode(), 'text/html; charset=utf-8')
            elif self.path == '/health':
                self.reply(json.dumps(npu.description).encode())
            elif self.path in ('/status','/dual/status'):
                with condition:
                    self.reply(json.dumps(dict(frame_id=view['frame_id'],
                        age_s=None if view['updated'] is None else time.monotonic()-view['updated'],
                        geometry_valid=view.get('geometry_valid'),orientation_valid=view.get('orientation_valid'))).encode())
            elif self.path in ('/latest.jpg','/dual/latest.jpg'):
                with condition:
                    jpeg = view['jpeg']
                self.reply(jpeg or b'No frame', 'image/jpeg' if jpeg else 'text/plain', 200 if jpeg else 503)
            elif self.path in ('/stream.mjpg','/dual/stream.mjpg'):
                self.send_response(200)
                self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
                self.send_header('Cache-Control', 'no-store')
                self.end_headers()
                previous = None
                try:
                    while True:
                        with condition:
                            condition.wait_for(lambda: view['updated'] != previous, timeout=5)
                            jpeg, previous = view['jpeg'], view['updated']
                        if jpeg:
                            self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\nContent-Length: '+str(len(jpeg)).encode()+b'\r\n\r\n'+jpeg+b'\r\n')
                            self.wfile.flush()
                except (BrokenPipeError, ConnectionResetError, OSError):
                    pass
            else:
                self.reply(b'Not found', 'text/plain', 404)

        def do_POST(self):
            if self.client_address[0] != args.client:
                self.reply(b'Client not allowed', 'text/plain', 403)
                return
            try:
                self.connection.settimeout(10)
                length = int(self.headers.get('Content-Length', '0'))
                limit = 640*640*3 if self.path == '/infer' else 2*1024*1024
                if length <= 0 or length > limit:
                    raise ValueError('Invalid payload length')
                data = self.rfile.read(length)
                if len(data) != length:
                    raise ValueError('Incomplete body')
                frame_id = self.headers.get('X-Frame-ID', '')[:80]
                if self.path == '/infer':
                    with lock:
                        result = npu.run(data, frame_id)
                    self.reply(result, 'application/octet-stream')
                elif self.path in ('/preview','/dual/preview') and data.startswith(b'\xff\xd8'):
                    with condition:
                        view = dual if self.path == '/dual/preview' else preview
                        view.update(jpeg=data, updated=time.monotonic(), frame_id=frame_id)
                        if view is dual:
                            view['geometry_valid'] = self.headers.get('X-Geometry-Valid') == '1'
                            view['orientation_valid'] = self.headers.get('X-Orientation-Valid') == '1'
                        condition.notify_all()
                    self.reply(b'{"ok":true}')
                else:
                    self.reply(b'Not found', 'text/plain', 404)
            except (BrokenPipeError, ConnectionResetError):
                pass
            except Exception as exc:
                self.reply(json.dumps({'error':str(exc)}).encode(), code=500)

    server = ThreadingHTTPServer((args.bind, args.port), Handler)
    server.daemon_threads = True
    def stop(*_):
        threading.Thread(target=server.shutdown, daemon=True).start()
    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    print('READY http://%s:%s/' % (args.bind, args.port), flush=True)
    try:
        server.serve_forever()
    finally:
        server.server_close()
        with lock:
            npu.close()


if __name__ == '__main__':
    main()
