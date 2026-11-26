#!/bin/sh
set -eu

# ---------独立视觉预览入口区---------
# 从板载相机采图，参数指定 RKNN 模型路径。
root=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
model=${1:-/userdata/scara-yolo-v3/xiangqi14-local-v3-rv1106.rknn}
case "$model" in /*) ;; *) model="$PWD/$model" ;; esac
# 从安装根目录读取本机 TOML。
cd "$root"
# 识别程序将 JPEG 帧写到标准输出，供播放器直接读取。
exec "$root/bin/scara_vision" "$model"
