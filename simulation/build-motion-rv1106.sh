#!/bin/sh
# 只交叉编译CAN/运动滤波库，不连接板子、不发送报文。
set -eu
cd "$(dirname "$0")"
cache="${SCARA_CACHE_DIR:-$HOME/.cache/scara-vision}"
toolchain="$cache/luckfox-toolchain/tools/linux/toolchain/arm-rockchip830-linux-uclibcgnueabihf"
opencv="$cache/install-rv1106"
mkdir -p output/arm
"$toolchain/bin/arm-rockchip830-linux-uclibcgnueabihf-g++" \
  -std=c++17 -O2 -fPIC -shared -pthread -mcpu=cortex-a7 -mfpu=neon-vfpv4 -mfloat-abi=hard \
  -I"$opencv/include/opencv5" -L"$opencv/lib" \
  x42s.cpp motion_filter.cpp -lopencv_core -o output/arm/libscara_motion.so
file output/arm/libscara_motion.so
