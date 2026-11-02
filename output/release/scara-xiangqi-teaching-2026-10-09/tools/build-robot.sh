#!/usr/bin/env bash
# 完整RV1106程序入口；依赖留在缓存，发布包只放运行所需文件。
set -euo pipefail
if [[ ${1:-} == --help || $# -lt 2 || $# -gt 3 ]]; then
    cat <<'EOF'
用法：bash tools/build-robot.sh MODEL_RKNN ENGINE_DIR [PACKAGE_DIR]
ENGINE_DIR须包含pikafish和已解压的pikafish.nnue。
环境变量：SCARA_TOOLCHAIN_ROOT、SCARA_OPENCV_ROOT、SCARA_OPENCV_SOURCE、SCARA_RKNN_ROOT、
SCARA_BUILD_DIR、SCARA_PACKAGE_ROOT、SCARA_BUILD_JOBS、SCARA_BUILD_TESTING。
SCARA_RKNN_ROOT目录结构为include/rknn_api.h、lib/librknnmrt.so。
包目录须位于SCARA_PACKAGE_ROOT内，默认是项目output；构建成功后替换旧发布包。
EOF
    [[ ${1:-} == --help ]] && exit 0
    exit 2
fi

project_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
cache_dir=${SCARA_CACHE_DIR:-"$HOME/.cache/scara-vision"}
toolchain=${SCARA_TOOLCHAIN_ROOT:-"$cache_dir/luckfox-toolchain/tools/linux/toolchain/arm-rockchip830-linux-uclibcgnueabihf"}
opencv=${SCARA_OPENCV_ROOT:-"$cache_dir/install-rv1106"}
opencv_source=${SCARA_OPENCV_SOURCE:-"$cache_dir/opencv-5.0.0"}
rknn=${SCARA_RKNN_ROOT:-${RKNN_SDK_DIR:-"$HOME/.cache/scara-robot-deps/rknn"}}
build_dir=${SCARA_BUILD_DIR:-"$HOME/.cache/scara-robot-rv1106"}
package_root=$(realpath -m -- "${SCARA_PACKAGE_ROOT:-"$project_dir/output"}")
package_dir=$(realpath -m -- "${3:-"$package_root/rv1106/scara-robot"}")
model=$(realpath -- "$1")
engine=$(realpath -- "$2")
jobs=${SCARA_BUILD_JOBS:-4}
testing=${SCARA_BUILD_TESTING:-ON}
prefix=arm-rockchip830-linux-uclibcgnueabihf

case "$package_dir/" in
    "$package_root/"?*) ;;
    *) printf '%s\n' '包目录必须是SCARA_PACKAGE_ROOT下的子目录，不能清理根目录。' >&2; exit 2 ;;
esac
if [[ -e "$package_dir" && ! -f "$package_dir/scara_robot" ]]; then
    printf '拒绝替换非运行包目录：%s\n' "$package_dir" >&2
    exit 2
fi
for file in "$toolchain/bin/$prefix-g++" "$opencv/lib/cmake/opencv5/OpenCVConfig.cmake" \
            "$rknn/include/rknn_api.h" "$rknn/lib/librknnmrt.so" \
            "$model" "$engine/pikafish" "$engine/pikafish.nnue" "$engine/Copying.txt" \
            "$opencv_source/LICENSE" "$opencv_source/3rdparty/libjpeg-turbo/LICENSE.md" \
            "$opencv_source/3rdparty/libjpeg-turbo/README.ijg" \
            "$opencv_source/3rdparty/libpng/LICENSE" "$opencv_source/3rdparty/zlib/LICENSE"; do
    [[ -s "$file" ]] || { printf '缺少构建输入：%s\n' "$file" >&2; exit 2; }
done
[[ "$jobs" =~ ^[1-9][0-9]*$ ]] || { printf '%s\n' '并行数必须为正整数。' >&2; exit 2; }

cmake -S "$project_dir" -B "$build_dir" \
    -DCMAKE_TOOLCHAIN_FILE="$project_dir/cmake/rv1106.cmake" \
    -DSCARA_TOOLCHAIN_ROOT="$toolchain" \
    -DOpenCV_DIR="$opencv/lib/cmake/opencv5" -DRKNN_SDK_DIR="$rknn" \
    -DSCARA_WITH_RKNN=ON -DBUILD_TESTING="$testing" -DCMAKE_BUILD_TYPE=Release
cmake --build "$build_dir" --parallel "$jobs"

# 在同一输出根中组装完整新包，输入模型即使来自旧包，也先复制完成再替换。
mkdir -p -- "$(dirname -- "$package_dir")"
stage=$(mktemp -d "$(dirname -- "$package_dir")/.scara-package.XXXXXX")
trap 'rm -rf -- "$stage"' EXIT
mkdir -p "$stage/config" "$stage/lib" "$stage/models" "$stage/engine" "$stage/licenses"
cp "$build_dir/scara_robot" "$stage/"
cp "$project_dir/config/robot.json" "$stage/config/robot.json"
cp "$project_dir/tools/run-robot.sh" "$project_dir/tools/camera-mis5001.sh" "$stage/"
cp "$model" "$stage/models/xiangqi-yolov5s-relu.rknn"
cp "$engine/pikafish" "$engine/pikafish.nnue" "$stage/engine/"
for module in core imgproc imgcodecs objdetect geometry features flann; do
    cp -L "$opencv/lib/libopencv_$module.so.500" "$stage/lib/"
done
if [[ "$testing" == ON && -f "$build_dir/scara_tests" ]]; then
    cp "$build_dir/scara_tests" "$stage/"
    "$toolchain/bin/$prefix-strip" "$stage/scara_tests"
fi
"$toolchain/bin/$prefix-strip" "$stage/scara_robot" "$stage/lib/"*.so.500
chmod +x "$stage/scara_robot" "$stage/run-robot.sh" "$stage/camera-mis5001.sh" "$stage/engine/pikafish"
# 保留已有第三方许可；不复制整套SDK，也不在包中覆盖板端RKNN运行库。
if [[ -d "$opencv/share/licenses/opencv5" ]]; then
    cp -R "$opencv/share/licenses/opencv5" "$stage/licenses/OpenCV-thirdparty"
fi
cp "$opencv_source/LICENSE" "$stage/licenses/OpenCV-LICENSE.txt"
cp "$opencv_source/3rdparty/libjpeg-turbo/LICENSE.md" "$stage/licenses/libjpeg-turbo-LICENSE.md"
cp "$opencv_source/3rdparty/libjpeg-turbo/README.ijg" "$stage/licenses/libjpeg-README.ijg"
cp "$opencv_source/3rdparty/libpng/LICENSE" "$stage/licenses/libpng-LICENSE.txt"
cp "$opencv_source/3rdparty/zlib/LICENSE" "$stage/licenses/zlib-LICENSE.txt"
cp "$engine/Copying.txt" "$stage/licenses/Pikafish-Copying.txt"
cp "$project_dir/tools/model/SOURCES.md" "$stage/models/SOURCES.md"
cp "$project_dir/tools/model/validation.json" "$stage/models/validation.json"
cp "$project_dir/tools/model/"*LICENSE*.txt "$project_dir/tools/model/YOLOv5-GPL-3.0.txt" \
   "$project_dir/tools/model/Rockchip-Apache-2.0.txt" "$stage/licenses/"
{
    printf 'OpenCV=5.0.0\nOpenCV_expected_commit=40738fb16ceddb5fb3fea747585f7ce6abb0605b\n'
    printf 'Expected_toolchain_source=LuckfoxTECH/luckfox-pico@824b817f889c2cbff1d48fcdb18ab494a68f69d1\n'
    printf 'Compiler='; "$toolchain/bin/$prefix-g++" -dumpfullversion
    printf 'RKNN_link_library_sha256='; sha256sum "$rknn/lib/librknnmrt.so" | cut -d ' ' -f 1
    printf 'RKNN_runtime=/oem/usr/lib/librknnmrt.so (not replaced by this package)\n'
} > "$stage/BUILD-INFO.txt"
(cd "$stage" && find . -type f ! -name SHA256SUMS -print0 | sort -z | xargs -0 sha256sum > SHA256SUMS)
source_archive="$(dirname -- "$package_dir")/source-4762ca3.tar.gz"
if [[ -f "$engine/source-4762ca3.tar.gz" && ! "$engine/source-4762ca3.tar.gz" -ef "$source_archive" ]]; then
    cp "$engine/source-4762ca3.tar.gz" "$source_archive"
fi

# 只替换上面已确认位于输出根、且含scara_robot标识的生成包；不清理源码或缓存。
if [[ -e "$package_dir" ]]; then rm -rf -- "$package_dir"; fi
mv -- "$stage" "$package_dir"
printf '完整运行包：%s\n' "$package_dir"
printf '%s\n' '程序未启动；模型、引擎和库的SHA256见包内SHA256SUMS。'
