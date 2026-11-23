#!/bin/sh
set -eu

# ---------独立视觉预览入口区---------
# 两个参数依次覆盖默认的模型路径和相机 RTSP 地址。
root=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
model=${1:-/userdata/scara-yolo-v3/xiangqi14-local-v3-rv1106.rknn}
source=${2:-rtsp://127.0.0.1/live/0}

export OPENCV_FFMPEG_CAPTURE_OPTIONS=rtsp_transport\;tcp
# 识别程序将 JPEG 帧写到标准输出，供播放器直接读取。
exec "$root/bin/scara_vision" "$model" "$source"
