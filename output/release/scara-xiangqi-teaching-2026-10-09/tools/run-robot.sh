#!/bin/sh
# 在RV1106包目录运行；仅加载本次所需模块，不修改开机脚本、系统库或网络。
set -eu
app_dir=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
cd "$app_dir"
export PATH="/oem/usr/bin:${PATH:-/bin:/sbin:/usr/bin:/usr/sbin}"
export LD_LIBRARY_PATH="$app_dir/lib:/oem/usr/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
# 识别器先于相机初始化，所以NPU模块在主程序启动前准备。
if [ ! -d /sys/module/rknpu ]; then
    insmod /oem/usr/ko/rknpu.ko
fi
exec "$app_dir/scara_robot" "${1:-config/robot.json}"
