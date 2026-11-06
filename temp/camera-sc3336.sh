#!/bin/sh
# BoomPi RV1106, verified 2026-11-04. Foreground preview; stop with Ctrl+C.
set -eu
if pidof simple_vi_bind_venc_rtsp >/dev/null 2>&1; then
    echo 'RTSP preview is already running; stop its original session first.' >&2
    exit 1
fi
cd /oem/usr/ko
for module in rk_dvbm sc3336 video_rkcif video_rkisp phy-rockchip-csi2-dphy-hw phy-rockchip-csi2-dphy rga3 mpp_vcodec
do
    module_name=$(printf '%s' "$module" | tr '-' '_')
    if ! grep -q "^$module_name " /proc/modules; then
        insmod "$module.ko"
    fi
done
if [ ! -L /sys/bus/i2c/devices/4-0030/driver ] ||
   [ "$(basename "$(readlink /sys/bus/i2c/devices/4-0030/driver)")" != sc3336 ]; then
    echo 'SC3336 is not bound; check the sensor and cable with power off.' >&2
    exit 1
fi
echo 1 > /sys/module/video_rkcif/parameters/clr_unready_dev
echo 1 > /sys/module/video_rkisp/parameters/clr_unready_dev
if ! grep -q '^rockit ' /proc/modules; then
    insmod rockit.ko mcu_fw_path=/oem/usr/ko/hpmcu_wrap.bin mcu_fw_addr=0xff6fe000 isp_max_h=1296
fi
exec /oem/usr/bin/simple_vi_bind_venc_rtsp -I 0 -w 1280 -h 720 -e h264 -b 2000 -a /oem/usr/share/iqfiles
