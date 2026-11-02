"""Generate labeled synthetic inputs for the same board-recognition executable."""
import json
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw

root = Path(__file__).resolve().parent.parent
out = root / "output" / "rv1106" / "validation" / "inputs"
out.mkdir(parents=True, exist_ok=True)
base = Image.new("RGB", (900, 900), "white")
base.paste(Image.open(root / "assets" / "board" / "board.png"), (40, 40))
base.save(out / "empty.png")
board = [["."] * 15 for _ in range(15)]
mixed = base.copy()
draw = ImageDraw.Draw(mixed)
for state, cells, color in [
    ("B", [(0, 0), (7, 7), (14, 14)], (20, 20, 20)),
    ("W", [(0, 14), (7, 8), (14, 0)], (240, 240, 240)),
]:
    for row, col in cells:
        x, y = 100 + 50 * col, 100 + 50 * row
        draw.ellipse((x - 19, y - 19, x + 19, y + 19), fill=color)
        board[row][col] = state
mixed.save(out / "mixed.png")
mixed.transpose(Image.Transpose.ROTATE_90).save(out / "rotated.png")

# PIL needs destination-to-source coefficients; four corners define the inverse map.
source = [(0, 0), (899, 0), (899, 899), (0, 899)]
destination = [(70, 40), (860, 85), (825, 850), (35, 790)]
matrix, values = [], []
for (x, y), (u, v) in zip(destination, source):
    matrix.extend([[x, y, 1, 0, 0, 0, -u*x, -u*y],
                   [0, 0, 0, x, y, 1, -v*x, -v*y]])
    values.extend([u, v])
coefficients = np.linalg.solve(matrix, values)
mixed.transform((900, 900), Image.Transform.PERSPECTIVE, coefficients,
                resample=Image.Resampling.BICUBIC, fillcolor="white").save(out / "perspective.png")
missing = base.copy()
ImageDraw.Draw(missing).rectangle((38, 38, 85, 85), fill="white")
missing.save(out / "missing-marker.png")
uncertain = base.copy()
ImageDraw.Draw(uncertain).ellipse((431, 431, 469, 469), fill=(140, 140, 140))
uncertain.save(out / "uncertain.png")
# Reuse the same runner for invalid marker layouts and an unsupported paper color.
marker0 = base.crop((36, 36, 84, 84))
marker1 = base.crop((816, 36, 864, 84))
duplicate = base.copy()
duplicate.paste(marker0, (816, 36))
duplicate.save(out / "duplicate-marker.png")
turned = base.copy()
turned.paste(marker0.transpose(Image.Transpose.ROTATE_90), (36, 36))
turned.save(out / "turned-marker.png")
swapped = base.copy()
swapped.paste(marker1, (36, 36))
swapped.paste(marker0, (816, 36))
swapped.save(out / "swapped-markers.png")
white_paper = np.array(base)
white_paper[np.all(white_paper == (199, 167, 106), axis=2)] = 255
Image.fromarray(white_paper).save(out / "white-background.png")
expected = {
    "empty.png": {"exit": 0, "board": [["."] * 15 for _ in range(15)]},
    "mixed.png": {"exit": 0, "board": board},
    "rotated.png": {"exit": 0, "board": board},
    "perspective.png": {"exit": 0, "board": board},
    "missing-marker.png": {"exit": 1},
    "uncertain.png": {"exit": 2, "unknown_cell": [7, 7]},
    "duplicate-marker.png": {"exit": 1},
    "turned-marker.png": {"exit": 1},
    "swapped-markers.png": {"exit": 1},
    "white-background.png": {"exit": 2, "board": [["?"] * 15 for _ in range(15)]},
}

# 固定相机输入：原始 900x900 图不缩放，居中放在 1280x960 白底上。
# BT.601 limited-range，RGB 为 0..255；Y 为 16..235，U/V 为 16..240。
# 这是合成标签测试，不代表真实相机的量化范围和颜色已经校准。
width, height = 1280, 960
for png_name, label in expected.items():
    frame = Image.new("RGB", (width, height), "white")
    picture = Image.open(out / png_name).convert("RGB")
    frame.paste(picture, ((width - picture.width) // 2, (height - picture.height) // 2))
    rgb = np.asarray(frame, dtype=np.float32)
    red, green, blue = rgb[:, :, 0], rgb[:, :, 1], rgb[:, :, 2]
    y = 16 + (65.481 * red + 128.553 * green + 24.966 * blue) / 255
    u = 128 + (-37.797 * red - 74.203 * green + 112 * blue) / 255
    v = 128 + (112 * red - 93.786 * green - 18.214 * blue) / 255
    y_plane = np.clip(np.rint(y), 16, 235).astype(np.uint8)
    uv_plane = np.empty((height // 2, width), dtype=np.uint8)
    # 先对每个 2x2 区块平均，再量化；NV12 按 U,V 交错，不是 NV21。
    for channel, offset in ((u, 0), (v, 1)):
        averaged = channel.reshape(height // 2, 2, width // 2, 2).mean(axis=(1, 3))
        uv_plane[:, offset::2] = np.clip(np.rint(averaged), 16, 240).astype(np.uint8)
    raw = y_plane.tobytes() + uv_plane.tobytes()
    assert len(raw) == 1843200
    nv12_name = Path(png_name).with_suffix(".nv12").name
    (out / nv12_name).write_bytes(raw)
    label["nv12_file"] = nv12_name
    label["frame_size"] = [width, height]

(out / "expected.json").write_text(json.dumps(expected, indent=2), encoding="utf-8")
print(f"Generated {len(expected)} PNG/NV12 pairs. These do not establish real-camera accuracy.")
