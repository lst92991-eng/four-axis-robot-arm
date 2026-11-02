#!/bin/sh
set -eu
cd "$(dirname "$0")"
export LD_LIBRARY_PATH="$PWD/deps-opencv-5.0.0/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
exec ./build/scara_sim "$@"
