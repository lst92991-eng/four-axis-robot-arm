#!/bin/sh
# 在 RV1106 上持续采集 MIS5001。正式视觉进程启动前停止此服务。
set -eu
loader=$(dirname "$0")/camera-mis5001.sh
pidfile=/tmp/scara-camera-keepalive.pid
log=/tmp/scara-camera-keepalive.log

case "$*" in
  start)
    if [ -s "$pidfile" ] && kill -0 "$(cat "$pidfile")" 2>/dev/null; then
      exit 0
    fi
    nohup sh "$0" run </dev/null >"$log" 2>&1 &
    echo "$!" >"$pidfile"
    exit 0
    ;;
  stop)
    if [ -s "$pidfile" ]; then
      kill "$(cat "$pidfile")" 2>/dev/null || true
      rm -f "$pidfile"
    fi
    exit 0
    ;;
  ""|run) ;;
  *) echo "用法：$0 start|stop|run" >&2; exit 2 ;;
esac

child=
trap 'if [ -n "$child" ]; then kill "$child" 2>/dev/null || true; wait "$child" 2>/dev/null || true; fi; exit 0' TERM INT
command -v rkisp_demo >/dev/null
sh "$loader"
while true
do
    rkisp_demo --device /dev/video11 --width 1280 --height 960 --format NV12 \
        --count 1000030 --stream-skip 30 --stream-count 1000000 \
        --stream-to /dev/null --rkaiq --iqpath /oem/usr/share/iqfiles --silent &
    child=$!
    wait "$child" || true
    child=
    sleep 3
done

