#!/bin/sh
# 复用旧项目已验证的MIS5001模块加载顺序；只在显式--run采图时调用。
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
