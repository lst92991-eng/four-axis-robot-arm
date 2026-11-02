"""Prepare only real training images for the fixed 640x640 RGB model.

Usage: python prepare_calibration.py DATASET_ROOT OUTPUT_DIRECTORY
Requires OpenCV 5.0.0. No validation/test images are read.
"""
import hashlib
import json
from pathlib import Path
import sys

import cv2


def main():
    if len(sys.argv) != 3:
        raise SystemExit(__doc__)
    if cv2.__version__ != '5.0.0':
        raise SystemExit('Use the OpenCV 5.0.0 training/preprocessing environment.')
    dataset, output = map(Path, sys.argv[1:])
    output = output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    images = sorted((dataset / 'images/train').glob('*'))
    if not images:
        raise SystemExit('No real training images found.')
    records = []
    cv2.setNumThreads(1)
    for path in images:
        image = cv2.imread(str(path))
        if image is None:
            raise RuntimeError(f'Cannot read training image: {path}')
        height, width = image.shape[:2]
        ratio = min(640 / width, 640 / height)
        w, h = round(width * ratio), round(height * ratio)
        left, top = (640 - w) // 2, (640 - h) // 2
        resized = cv2.resize(image, (w, h), interpolation=cv2.INTER_LINEAR)
        padded = cv2.copyMakeBorder(resized, top, 640 - h - top, left, 640 - w - left,
                                   cv2.BORDER_CONSTANT, value=(114, 114, 114))
        target = output / (path.stem + '.png')
        # OpenCV 的 BGR 像素直接 imwrite；PNG 被 SDK 按 RGB 读取时通道才正确。
        if not cv2.imwrite(str(target), padded):
            raise RuntimeError(f'Cannot save calibration image: {target}')
        records.append({'source': path.name, 'output': str(target),
                        'source_sha256': hashlib.sha256(path.read_bytes()).hexdigest(),
                        'png_sha256': hashlib.sha256(target.read_bytes()).hexdigest()})
    (output / 'calibration.txt').write_bytes(''.join(r['output'] + '\n' for r in records).encode())
    (output / 'manifest.json').write_text(json.dumps({'opencv': cv2.__version__,
        'source_split': 'train only', 'size': [640, 640], 'padding': [114, 114, 114],
        'images': records}, indent=2), encoding='utf-8')
    print(output / 'calibration.txt')


if __name__ == '__main__':
    main()
