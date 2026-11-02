#!/usr/bin/env bash
# 在 WSL Ubuntu-24.04-D 执行；依赖库与构建产物留在独立缓存目录。
set -euo pipefail
project_dir="$(cd "$(dirname "$0")/.." && pwd)"
cache_dir="${SCARA_CACHE_DIR:-$HOME/.cache/scara-vision}"
source_dir="$cache_dir/opencv-5.0.0"
toolchain_dir="$cache_dir/luckfox-toolchain/tools/linux/toolchain/arm-rockchip830-linux-uclibcgnueabihf"
build_dir="$cache_dir/build-rv1106"
install_dir="$cache_dir/install-rv1106"
jobs="${SCARA_BUILD_JOBS:-8}"

cmake -S "$source_dir" -B "$build_dir" -G 'Unix Makefiles' \
  -DCMAKE_TOOLCHAIN_FILE="$project_dir/cmake/rv1106.cmake" \
  -DSCARA_TOOLCHAIN_ROOT="$toolchain_dir" \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_STANDARD=17 \
  -DCMAKE_INSTALL_PREFIX="$install_dir" -DBUILD_SHARED_LIBS=ON \
  -DBUILD_LIST=core,imgproc,imgcodecs \
  -DBUILD_TESTS=OFF -DBUILD_PERF_TESTS=OFF -DBUILD_EXAMPLES=OFF \
  -DBUILD_opencv_apps=OFF -DBUILD_opencv_python3=OFF -DBUILD_JAVA=OFF \
  -DWITH_IPP=OFF -DWITH_ITT=OFF -DWITH_OPENCL=OFF -DWITH_TBB=OFF \
  -DWITH_EIGEN=OFF -DWITH_LAPACK=OFF -DWITH_FFMPEG=OFF -DWITH_GSTREAMER=OFF \
  -DWITH_JPEG=ON -DBUILD_JPEG=ON -DWITH_PNG=ON -DBUILD_PNG=ON -DBUILD_ZLIB=ON \
  -DWITH_OPENEXR=OFF -DWITH_TIFF=OFF -DWITH_WEBP=OFF -DWITH_OPENJPEG=OFF \
  -DWITH_JASPER=OFF -DWITH_AVIF=OFF -DWITH_JPEGXL=OFF \
  -DOPENCV_DOWNLOAD_PATH="$cache_dir/downloads"
cmake --build "$build_dir" --parallel "$jobs"
cmake --install "$build_dir"

cmake -S "$project_dir/vision" -B "$cache_dir/vision-rv1106" \
  -DCMAKE_TOOLCHAIN_FILE="$project_dir/cmake/rv1106.cmake" \
  -DSCARA_TOOLCHAIN_ROOT="$toolchain_dir" \
  -DOpenCV_DIR="$install_dir/lib/cmake/opencv5" -DCMAKE_BUILD_TYPE=Release
cmake --build "$cache_dir/vision-rv1106" --parallel "$jobs"

package_dir="$project_dir/output/rv1106/scara-vision-v1"
mkdir -p "$package_dir/lib" "$package_dir/licenses"
cp "$cache_dir/vision-rv1106/scara_vision" "$package_dir/"
for module in core imgproc imgcodecs; do
    cp -L "$install_dir/lib/libopencv_$module.so.500" "$package_dir/lib/"
done
"$toolchain_dir/bin/arm-rockchip830-linux-uclibcgnueabihf-strip" \
    "$package_dir/scara_vision" "$package_dir"/lib/*.so.500
cp "$project_dir/tools/capture.sh" \
    "$project_dir/tools/camera-mis5001.sh" "$package_dir/"
cp "$project_dir/tools/runtime-usage.txt" "$package_dir/README.txt"
cp "$source_dir/LICENSE" "$package_dir/licenses/OpenCV-LICENSE.txt"
cp "$source_dir/3rdparty/libjpeg-turbo/LICENSE.md" "$package_dir/licenses/libjpeg-turbo-LICENSE.md"
cp "$source_dir/3rdparty/libjpeg-turbo/README.ijg" "$package_dir/licenses/libjpeg-README.ijg"
cp "$source_dir/3rdparty/libpng/LICENSE" "$package_dir/licenses/libpng-LICENSE.txt"
cp "$source_dir/3rdparty/zlib/LICENSE" "$package_dir/licenses/zlib-LICENSE.txt"
