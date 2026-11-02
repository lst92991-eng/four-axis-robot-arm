#!/bin/sh
# root在新网络命名空间创建唯一vcan0；不读取或修改宿主物理CAN。
set -eu
if [ "$#" -ne 1 ] || [ ! -x "$1" ]; then
    echo "用法：sudo sh scripts/test-vcan.sh /absolute/path/scara_tests" >&2
    exit 2
fi
exec unshare --net sh -c '
    set -eu
    ip link add vcan0 type vcan
    ip link set vcan0 up
    SCARA_TEST_CAN_INTERFACE=vcan0 exec "$1"
' sh "$1"
