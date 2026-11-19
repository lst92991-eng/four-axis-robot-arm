#!/bin/sh
set -eu

root=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
model=${1:-/userdata/scara-yolo-v3/xiangqi14-local-v3-rv1106.rknn}
source=${2:-rtsp://127.0.0.1/live/0}
preview=${3:-/tmp/scara-vision.jpg}

# 先核对既定模型，再进入相机与识别主流程。
expected=ce9c91f4668cb0ec58f836432546b7eb2aebfc675ba696ce9788968560dcbdd2
actual=$(sha256sum "$model")
if [ "${actual%% *}" != "$expected" ]; then
  printf '%s\n' '模型 SHA256 与已验证的 v3 模型不一致' >&2
  exit 1
fi
export OPENCV_FFMPEG_CAPTURE_OPTIONS=rtsp_transport\;tcp
exec "$root/bin/scara_vision" "$model" "$source" "$root/config/vision/chinese-labels.png" "$preview"
