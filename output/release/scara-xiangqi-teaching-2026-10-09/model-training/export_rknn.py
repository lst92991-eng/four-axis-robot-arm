"""Export a trained 14-class YOLOv5 to the RV1106 RKNN 1.6.0 profile.

Usage: python export_rknn.py trained.pt calibration.txt output.rknn YOLOV5_SOURCE
Run only in the isolated RKNN conversion environment described in SOURCES.md.
The input checkpoint must be trusted: PyTorch 1.13 checkpoints use pickle.
"""
import gc
import hashlib
import importlib.metadata
import json
from pathlib import Path
import sys
import types

CLASS_ORDER = 'kabnrcpKABNRCP'
FEN_BY_CLASS = {
    'General_black': 'k', 'Advisor_black': 'a', 'Elephant_black': 'b',
    'Knight_black': 'n', 'Rook_black': 'r', 'Cannon_black': 'c', 'Pawn_black': 'p',
    'General_red': 'K', 'Advisor_red': 'A', 'Elephant_red': 'B',
    'Knight_red': 'N', 'Rook_red': 'R', 'Cannon_red': 'C', 'Pawn_red': 'P',
}
ANCHORS = [[10, 13, 16, 30, 33, 23], [30, 61, 62, 45, 59, 119],
           [116, 90, 156, 198, 373, 326]]


def main():
    if len(sys.argv) != 5:
        raise SystemExit(__doc__)
    checkpoint, calibration, output, source = map(Path, sys.argv[1:])
    if importlib.metadata.version('rknn-toolkit2').split('+')[0] != '1.6.0':
        raise SystemExit('This board requires RKNN Toolkit2 1.6.0.')
    if not checkpoint.is_file() or not (source / 'models/yolo.py').is_file():
        raise SystemExit('Checkpoint or pinned YOLOv5 source is missing.')
    calibration_images = [Path(line.strip()) for line in calibration.read_text().splitlines() if line.strip()]
    if not calibration_images or any(not p.is_absolute() or not p.is_file() for p in calibration_images):
        raise SystemExit('Calibration list must contain absolute paths to real prepared training images.')
    sys.path.insert(0, str(source.resolve()))
    import torch
    from models.experimental import attempt_load
    from models.yolo import Detect

    model = attempt_load(str(checkpoint.resolve()), device=torch.device('cpu'), fuse=True)
    if any(isinstance(module, torch.nn.SiLU) for module in model.modules()):
        raise SystemExit('Train the pinned Rockchip ReLU model; do not replace SiLU after training.')
    head = model.model[-1]
    if not isinstance(head, Detect) or head.nc != 14 or head.nl != 3 or head.na != 3:
        raise SystemExit('Expected a trained YOLOv5 with 14 classes, 3 heads and 3 anchors/head.')
    names = [model.names[i] for i in range(14)]
    mapped = [FEN_BY_CLASS.get(name, name) for name in names]
    if sorted(mapped) != sorted(CLASS_ORDER):
        raise SystemExit('Class names must be canonical FEN letters or the documented 14 dataset names.')
    actual_anchors = (head.anchors * head.stride.view(3, 1, 1)).reshape(3, 6)
    if not torch.allclose(actual_anchors.cpu(), torch.tensor(ANCHORS, dtype=torch.float32), atol=0.001):
        raise SystemExit('Anchors changed. Train with --noautoanchor; this detector has fixed anchors.')
    if head.stride.tolist() != [8.0, 16.0, 32.0]:
        raise SystemExit('Expected stride 8/16/32 in that output order.')

    # 重排真实分类权重，不只改名字，确保 C++ classId 与棋子字符一一对应。
    class_indices = [mapped.index(letter) for letter in CLASS_ORDER]
    channels = [anchor * 19 + column for anchor in range(3)
                for column in list(range(5)) + [5 + i for i in class_indices]]
    for layer in head.m:
        layer.weight.data = layer.weight.data[channels].clone()
        layer.bias.data = layer.bias.data[channels].clone()
    model.names = dict(enumerate(CLASS_ORDER))

    # 仅移出坐标解码；Sigmoid 仍在图中，C++ 读取反量化后的概率。
    def forward_heads(self, features):
        return tuple(torch.sigmoid(self.m[i](features[i])) for i in range(3))
    head.forward = types.MethodType(forward_heads, head)
    output = output.resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    onnx_path = output.with_suffix('.onnx')
    with torch.no_grad():
        torch.onnx.export(model, torch.zeros(1, 3, 640, 640), str(onnx_path),
                          opset_version=12, do_constant_folding=True,
                          input_names=['rgb'], output_names=['stride8', 'stride16', 'stride32'])
    del model, head
    gc.collect()

    from rknn.api import RKNN
    rknn = RKNN(verbose=False)
    try:
        rknn.config(mean_values=[[0, 0, 0]], std_values=[[255, 255, 255]], target_platform='rv1106')
        if rknn.load_onnx(model=str(onnx_path)) != 0:
            raise RuntimeError('RKNN ONNX loading failed.')
        if rknn.build(do_quantization=True, dataset=str(calibration.resolve())) != 0:
            raise RuntimeError('RKNN INT8 conversion failed.')
        if rknn.export_rknn(str(output)) != 0:
            raise RuntimeError('RKNN export failed.')
    finally:
        rknn.release()
    metadata = {
        'target': 'rv1106', 'toolkit': '1.6.0', 'input': [1, 3, 640, 640],
        'training_profile': 'pinned Rockchip YOLOv5 ReLU, autoanchor disabled',
        'input_order': 'RGB', 'class_order': CLASS_ORDER, 'original_names': names,
        'class_permutation': class_indices, 'anchors': ANCHORS,
        'head_strides': [8, 16, 32], 'output_values': 'sigmoid probabilities, INT8 native NHWC',
        'source_checkpoint_sha256': hashlib.sha256(checkpoint.read_bytes()).hexdigest(),
        'rknn_sha256': hashlib.sha256(output.read_bytes()).hexdigest(),
        'calibration_images': len(calibration_images),
        'accuracy_status': 'Conversion only. Real validation and board inference remain separate.',
    }
    output.with_suffix('.json').write_text(json.dumps(metadata, ensure_ascii=False, indent=2), encoding='utf-8')
    print(output)


if __name__ == '__main__':
    main()
