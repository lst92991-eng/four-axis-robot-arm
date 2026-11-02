#!/bin/sh
# 取得一帧彩色图。先跳过30帧，让ISP自动曝光/白平衡稳定。
set -eu
app_dir=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
cd "$app_dir"
lock_dir=/tmp/scara-vision-capture.lock
if ! mkdir "$lock_dir" 2>/dev/null; then
    printf '%s\n' 'Another capture is running; wait for it to finish.' >&2
    exit 1
fi
capture_pid=
guard_pid=
cleanup() {
    if [ -n "$capture_pid" ]; then kill -KILL "$capture_pid" 2>/dev/null || :; fi
    if [ -n "$guard_pid" ]; then kill "$guard_pid" 2>/dev/null || :; fi
    rm -f "$lock_dir/timed-out"
    rmdir "$lock_dir"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
# 开始前清理上次的原始帧和照片，避免误看旧结果。
mkdir -p live
rm -f camera.nv12 live/source.jpg
sh ./camera-mis5001.sh
export LD_LIBRARY_PATH="$app_dir/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
rkisp_demo --device /dev/video11 --width 1280 --height 960 --format NV12 \
    --count 40 --stream-skip 30 --stream-count 1 \
    --stream-to camera.nv12 --rkaiq --iqpath /oem/usr/share/iqfiles \
    > camera.log 2>&1 &
capture_pid=$!
(sleep 20
 if kill -0 "$capture_pid" 2>/dev/null; then
     : > "$lock_dir/timed-out"
     kill -TERM "$capture_pid" 2>/dev/null || :
     sleep 2
     kill -KILL "$capture_pid" 2>/dev/null || :
 fi) &
guard_pid=$!
capture_status=0
wait "$capture_pid" || capture_status=$?
capture_pid=
kill "$guard_pid" 2>/dev/null || :
guard_pid=
if [ -f "$lock_dir/timed-out" ]; then
    printf '%s\n' 'Camera capture timed out; see camera.log.' >&2
    exit 124
fi
if [ "$capture_status" -ne 0 ]; then
    printf '%s\n' 'Camera capture failed; see camera.log.' >&2
    exit "$capture_status"
fi
result=0
./scara_vision || result=$?
if [ "$result" -eq 0 ]; then
    printf '%s\n' 'Done: live/source.jpg'
else
    rm -f live/source.jpg
    printf '%s\n' 'Conversion failed: check camera.nv12.' >&2
fi
exit "$result"
