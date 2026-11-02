"""桌面仿真专用：WSL CUDA 推理，不进入 RV1106 的教学控制代码。"""
import hashlib
import json
import os
from pathlib import Path
import socket
import struct
import sys
import threading
import time

os.environ['YOLOv5_AUTOINSTALL'] = 'false'
os.environ.pop('RKNN_model_hack', None)
source, checkpoint = map(Path, sys.argv[1:3])
expected = 'b18cabfe0202b05ad4b127cbe52632aa7d20026af2f5b9d5c9186ab476050a61'
if hashlib.sha256(checkpoint.read_bytes()).hexdigest() != expected:
    raise RuntimeError('模型哈希不符，请使用已验证的 xiangqi14-s-best-portable.pt')
sys.path.insert(0, str(source.resolve()))
import numpy as np
import torch
from torchvision.ops import nms

if not torch.cuda.is_available():
    raise RuntimeError('CUDA 不可用；不允许回退 CPU')
torch.set_num_threads(1)
torch.cuda.set_per_process_memory_fraction(.25, 0)  # 本机8GB显存，限制本程序分配器预算。
torch.backends.cudnn.benchmark = False
# 只反序列化上面核验过哈希的项目自有权重。
model = torch.load(checkpoint, map_location='cpu', weights_only=False)['model']
model = model.float().eval().fuse().cuda()
import cv2
if cv2.__version__ != '5.0.0':
    raise RuntimeError('GPU配套环境也要求OpenCV5.0.0')
original = [model.names[i] for i in range(14)]
letters = {'General': 'k', 'Advisor': 'a', 'Elephant': 'b', 'Knight': 'n',
           'Rook': 'r', 'Cannon': 'c', 'Pawn': 'p'}
names = []
for name in original:
    if name in 'kabnrcpKABNRCP' and len(name) == 1:
        names.append(name)
    else:
        kind, side = name.split('_')
        names.append(letters[kind].upper() if side == 'red' else letters[kind])
order = [names.index(c) for c in 'kabnrcpKABNRCP']
info = {'backend': 'cuda:0', 'gpu': torch.cuda.get_device_name(0),
        'torch': torch.__version__, 'opencv': cv2.__version__,
        'classes': 'kabnrcpKABNRCP', 'checkpoint_sha256': expected,
        'precision': 'FP32', 'allocator_budget_fraction': .25}
inference_lock = threading.Lock()

@torch.inference_mode()
def detect(data):
    with inference_lock:  # 查询可并行，只有一份模型，实际推理串行执行。
        start = time.perf_counter()
        rgb = np.frombuffer(data, np.uint8).reshape(640, 640, 3).copy()
        image = torch.from_numpy(rgb).permute(2, 0, 1).unsqueeze(0).cuda().float() / 255
        prediction = model(image)[0][0]
        prediction = prediction[prediction[:, 4] >= .5]
        scores, labels = (prediction[:, 5:][:, order] * prediction[:, 4:5]).max(1)
        selected = scores >= .5
        prediction, scores, labels = prediction[selected], scores[selected], labels[selected]
        xy, wh = prediction[:, :2], prediction[:, 2:4] / 2
        boxes = torch.cat((xy - wh, xy + wh), 1)
        keep = nms(boxes, scores, .45)  # CUDA NMS，沿用原先类无关抑制。
        assert image.is_cuda and prediction.is_cuda and keep.is_cuda
        detections = torch.cat((labels[keep, None], scores[keep, None], boxes[keep]), 1).cpu().tolist()
        return {**info, 'ms': (time.perf_counter() - start) * 1000, 'detections': detections,
                'model_cuda': int(next(model.parameters()).is_cuda), 'input_cuda': int(image.is_cuda),
                'nms_cuda': int(keep.is_cuda), 'allocated_mb': torch.cuda.memory_allocated() / 2**20}

def read_exact(connection, length):
    data = bytearray()
    while len(data) < length:
        block = connection.recv(length - len(data))
        if not block:
            raise EOFError()
        data.extend(block)
    return data

def serve(connection):
    with connection:
        connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        connection.settimeout(30)
        try:
            while True:
                length, = struct.unpack('!I', read_exact(connection, 4))
                if length not in (0, 640 * 640 * 3):
                    raise ValueError('输入必须是640×640 RGB')
                answer = info if length == 0 else detect(read_exact(connection, length))
                payload = json.dumps(answer).encode()
                connection.sendall(struct.pack('!I', len(payload)) + payload)
        except (EOFError, ConnectionError, TimeoutError, ValueError):
            pass

# 启动时预热并真的执行CUDA NMS，不能只凭设备名字宣称GPU推理。
proof = detect(bytes([114]) * (640 * 640 * 3))
(checkpoint.parent / 'simulation-gpu-proof.json').write_text(json.dumps(proof, indent=2))
print(json.dumps(proof), flush=True)
with socket.socket() as server:
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind(('127.0.0.1', 8767))
    server.listen(2)
    while True:
        connection, _ = server.accept()
        threading.Thread(target=serve, args=(connection,), daemon=True).start()
