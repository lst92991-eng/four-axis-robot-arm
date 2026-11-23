#!/bin/sh
set -eu
# ---------对局程序启动区---------
# 由安装目录确定默认 RKNN 模型，也可以用第一个参数指定模型。
root=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
model=${1:-"$root/models/xiangqi14-local-v3-rv1106.rknn"}
case "$model" in /*) ;; *) model="$PWD/$model" ;; esac
export OPENCV_FFMPEG_CAPTURE_OPTIONS=rtsp_transport\;tcp
# 进入安装根目录，使引擎和权重的相对路径能在板端解析。
cd "$root"
exec "$root/bin/scara_game" "$model"
