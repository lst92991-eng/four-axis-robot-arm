#!/bin/sh
set -eu
root=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
config=${1:?需要对局配置JSON}
model=${2:-/userdata/scara-yolo-v3/xiangqi14-local-v3-rv1106.rknn}
expected=ce9c91f4668cb0ec58f836432546b7eb2aebfc675ba696ce9788968560dcbdd2
actual=$(sha256sum "$model")
if [ "${actual%% *}" != "$expected" ]; then
  printf '%s\n' '模型SHA256与已验证v3模型不一致' >&2
  exit 1
fi
export OPENCV_FFMPEG_CAPTURE_OPTIONS=rtsp_transport\;tcp
exec "$root/bin/scara_game" "$config" "$model"
