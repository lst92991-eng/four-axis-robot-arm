#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/src"
make -j4 all ARCH=armv7-neon COMP=gcc CXX=arm-linux-gnueabihf-g++ EXTRACXXFLAGS="-mcpu=cortex-a7 -mfloat-abi=hard" EXTRALDFLAGS="-static -mcpu=cortex-a7 -mfloat-abi=hard"
