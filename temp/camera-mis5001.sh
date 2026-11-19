#!/bin/sh
# 仅加载本板 MIS5001 采集所需模块，不修改开机脚本或启动网络/音频服务。
set -eu
module_dir=/oem/usr/ko
for module in rk_dvbm mis5001 video_rkcif video_rkisp phy-rockchip-csi2-dphy-hw phy-rockchip-csi2-dphy
do
    module_name=$(printf '%s' "$module" | tr '-' '_')
    if ! grep -q "^$module_name " /proc/modules; then
        insmod "$module_dir/$module.ko"
    fi
done
printf 1 > /sys/module/video_rkcif/parameters/clr_unready_dev
printf 1 > /sys/module/video_rkisp/parameters/clr_unready_dev
