# 完整 RV1106 项目的构建与部署

新入口是 `build-robot.sh`、`deploy-robot.ps1`、`run-robot.sh`。旧 `build-opencv.sh`、`deploy.ps1` 和 `capture.sh` 属于历史 lesson，勿用于本整机包。

以下构建命令在 Ubuntu/WSL Bash 执行；ADB 命令在能访问板子的 Windows PowerShell 执行。先进入本项目根目录。缓存与安装前缀均可替换，不需要改工程源码或系统 OpenCV。

## 1. 固定工具链和 OpenCV

Ubuntu 所需工具：`build-essential cmake git curl unzip python3`。工具链来自 [LuckfoxTECH/luckfox-pico](https://github.com/LuckfoxTECH/luckfox-pico/tree/824b817f889c2cbff1d48fcdb18ab494a68f69d1)，固定提交 `824b817f889c2cbff1d48fcdb18ab494a68f69d1`；不是任意系统 `arm-linux-gnueabihf-g++`。

```bash
export SCARA_PROJECT_DIR="$PWD"
export SCARA_CACHE_DIR="$HOME/.cache/scara-vision"
mkdir -p "$SCARA_CACHE_DIR"

# 新目录执行一次：只取交叉工具链子目录。
git clone --filter=blob:none --no-checkout --depth 1 \
  https://github.com/LuckfoxTECH/luckfox-pico.git "$SCARA_CACHE_DIR/luckfox-toolchain"
git -C "$SCARA_CACHE_DIR/luckfox-toolchain" sparse-checkout init --cone
git -C "$SCARA_CACHE_DIR/luckfox-toolchain" sparse-checkout set \
  tools/linux/toolchain/arm-rockchip830-linux-uclibcgnueabihf
git -C "$SCARA_CACHE_DIR/luckfox-toolchain" fetch --depth 1 origin \
  824b817f889c2cbff1d48fcdb18ab494a68f69d1
git -C "$SCARA_CACHE_DIR/luckfox-toolchain" checkout --detach \
  824b817f889c2cbff1d48fcdb18ab494a68f69d1
export SCARA_TOOLCHAIN_ROOT="$SCARA_CACHE_DIR/luckfox-toolchain/tools/linux/toolchain/arm-rockchip830-linux-uclibcgnueabihf"
"$SCARA_TOOLCHAIN_ROOT/bin/arm-rockchip830-linux-uclibcgnueabihf-g++" --version

git clone --filter=blob:none --no-checkout --depth 1 \
  https://github.com/opencv/opencv.git "$SCARA_CACHE_DIR/opencv-5.0.0"
git -C "$SCARA_CACHE_DIR/opencv-5.0.0" fetch --depth 1 origin \
  40738fb16ceddb5fb3fea747585f7ce6abb0605b
git -C "$SCARA_CACHE_DIR/opencv-5.0.0" checkout --detach \
  40738fb16ceddb5fb3fea747585f7ce6abb0605b
export SCARA_OPENCV_SOURCE="$SCARA_CACHE_DIR/opencv-5.0.0"
export SCARA_OPENCV_ROOT="$SCARA_CACHE_DIR/install-rv1106"
```

[OpenCV 5.0.0 固定源码](https://github.com/opencv/opencv/tree/40738fb16ceddb5fb3fea747585f7ce6abb0605b) 使用下列最小配置。`features`、`flann` 是这些模块自动引入的依赖。板端不需要 highgui；界面由电脑端提供。

```bash
cmake -S "$SCARA_OPENCV_SOURCE" -B "$SCARA_CACHE_DIR/build-rv1106" \
  -DCMAKE_TOOLCHAIN_FILE="$SCARA_PROJECT_DIR/cmake/rv1106.cmake" \
  -DSCARA_TOOLCHAIN_ROOT="$SCARA_TOOLCHAIN_ROOT" \
  -DCMAKE_INSTALL_PREFIX="$SCARA_OPENCV_ROOT" \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_STANDARD=17 -DBUILD_SHARED_LIBS=ON \
  -DBUILD_LIST=core,imgproc,imgcodecs,objdetect,geometry \
  -DBUILD_TESTS=OFF -DBUILD_PERF_TESTS=OFF -DBUILD_EXAMPLES=OFF \
  -DBUILD_opencv_apps=OFF -DBUILD_opencv_python3=OFF -DBUILD_JAVA=OFF \
  -DWITH_IPP=OFF -DWITH_ITT=OFF -DWITH_OPENCL=OFF -DWITH_TBB=OFF \
  -DWITH_EIGEN=OFF -DWITH_LAPACK=OFF -DWITH_VTK=OFF \
  -DWITH_FFMPEG=OFF -DWITH_GSTREAMER=OFF -DWITH_V4L=OFF \
  -DWITH_JPEG=ON -DBUILD_JPEG=ON -DWITH_PNG=ON -DBUILD_PNG=ON -DBUILD_ZLIB=ON \
  -DWITH_OPENEXR=OFF -DWITH_TIFF=OFF -DWITH_WEBP=OFF -DWITH_OPENJPEG=OFF \
  -DWITH_JASPER=OFF -DWITH_AVIF=OFF -DWITH_JPEGXL=OFF
cmake --build "$SCARA_CACHE_DIR/build-rv1106" --parallel 4
cmake --install "$SCARA_CACHE_DIR/build-rv1106"
```

首次安装后保留源码中的许可证；构建脚本会复制 OpenCV、libjpeg-turbo、README.ijg、libpng、zlib 的许可文件。依赖安装目录附带的其他第三方许可也会保留。

## 2. RKNN 头文件和板端链接库

头文件固定来自 [RKNN Model Zoo v1.6.0](https://github.com/airockchip/rknn_model_zoo/tree/eaa94d6f57ca553d493bf3bd7399a070452d2774)，直接下载地址如下。链接库从当前板子读取，**不要用下载的同名库替换板上系统运行库**。

Windows PowerShell：

```powershell
$Serial = '填写 adb devices 中的序列号'
$Sdk = Join-Path $env:USERPROFILE '.cache\scara-robot-rknn'
New-Item -ItemType Directory -Force "$Sdk\include", "$Sdk\lib" | Out-Null
Invoke-WebRequest -Uri 'https://raw.githubusercontent.com/airockchip/rknn_model_zoo/eaa94d6f57ca553d493bf3bd7399a070452d2774/3rdparty/rknpu2/include/rknn_api.h' -OutFile "$Sdk\include\rknn_api.h"
adb -s $Serial pull /oem/usr/lib/librknnmrt.so "$Sdk\lib\librknnmrt.so"
Get-FileHash "$Sdk\lib\librknnmrt.so" -Algorithm SHA256
```

本板已验证库版本是 RKNN 1.6.0，SHA-256 为 `d6259f5f8537acc15b0c3686010109bf4182e654281aad180a44bfd73ce929cc`。不同值不自动判定损坏，但意味着运行库组合不同，不能沿用既有验收结论。构建前在 WSL 中把上面输出的 `$Sdk` Windows 路径转换成 Linux 路径：

```bash
export SCARA_RKNN_ROOT="$(wslpath '粘贴上面Sdk变量的完整Windows路径')"
```

也可以在原生 Linux 建同样的 `include/`、`lib/` 目录，下载该头文件并用 Linux ADB 读取板端库。工程只用它完成链接；发布包不带 `librknnmrt.so`，运行时使用 `/oem/usr/lib` 中已有的版本。

## 3. 已验证的皮卡鱼和模型输入

引擎固定为 [Pikafish-2023-03-05](https://github.com/official-pikafish/Pikafish/tree/4762ca3c77c7f8ccbf204974cf23c5ca3e8c2783)。可以直接使用交付的引擎目录；从干净环境构建时：

```bash
deps="$HOME/.cache/scara-robot-deps"
export SCARA_ENGINE_DIR="$deps/engine-2023-03-05"
mkdir -p "$SCARA_ENGINE_DIR"
git clone --filter=blob:none --no-checkout --depth 1 \
  https://github.com/official-pikafish/Pikafish.git "$deps/pikafish-2023-03-05"
git -C "$deps/pikafish-2023-03-05" fetch --depth 1 origin \
  4762ca3c77c7f8ccbf204974cf23c5ca3e8c2783
git -C "$deps/pikafish-2023-03-05" checkout --detach \
  4762ca3c77c7f8ccbf204974cf23c5ca3e8c2783
make -C "$deps/pikafish-2023-03-05/src" -j4 all ARCH=armv7-neon COMP=gcc \
  COMPCXX="$SCARA_TOOLCHAIN_ROOT/bin/arm-rockchip830-linux-uclibcgnueabihf-g++" \
  EXTRACXXFLAGS='-mcpu=cortex-a7 -mfpu=neon-vfpv4 -mfloat-abi=hard'
cp "$deps/pikafish-2023-03-05/src/pikafish" "$SCARA_ENGINE_DIR/"
cp "$deps/pikafish-2023-03-05/Copying.txt" "$SCARA_ENGINE_DIR/"
git -C "$deps/pikafish-2023-03-05" archive --format=tar.gz \
  -o "$SCARA_ENGINE_DIR/source-4762ca3.tar.gz" 4762ca3c77c7f8ccbf204974cf23c5ca3e8c2783

curl -fL 'https://github.com/official-pikafish/Pikafish/releases/download/Pikafish-2023-03-05/Pikafish.2023-03-05.zip' \
  -o "$deps/Pikafish.2023-03-05.zip"
entry=$(unzip -Z1 "$deps/Pikafish.2023-03-05.zip" | awk '/(^|\/)pikafish\.nnue$/ {print; exit}')
test -n "$entry"
unzip -p "$deps/Pikafish.2023-03-05.zip" "$entry" > "$deps/pikafish.nnue.zip"
printf '%s  %s\n' 9bed5ed4f2f356d361c859728c68b1f9103fa982e5d9fe658b38f71dcbcec0a9 "$deps/pikafish.nnue.zip" | sha256sum -c -
unzip -p "$deps/pikafish.nnue.zip" pikafish.nnue > "$SCARA_ENGINE_DIR/pikafish.nnue"
printf '%s  %s\n' 2e9280f355cd4f092fcb63c21477692100afaf9ba9868a10c9c8d5b8fc6d1d1f "$SCARA_ENGINE_DIR/pikafish.nnue" | sha256sum -c -
```

NNUE 必须在电脑端预解压；不能把 ZIP 内容直接交给小内存板加载。引擎构建日期会进入程序横幅，重编译不保证等于已交付二进制 SHA。对应源码 `source-4762ca3.tar.gz` 放在运行包旁，GPL 许可放包内，不把第三方整个仓库复制进课程工程。

模型是另一份固定输入：取已训练并转换的 14 类 RV1106 RKNN 文件，类别顺序 `kabnrcpKABNRCP`。当前交付模型 SHA-256 为 `c843fac9714d97ca3b478193fb98557fef3146ca4c216a98380295a87c693943`。训练、转换、来源和适用范围见 [model/SOURCES.md](model/SOURCES.md)，不能用任意 YOLO/COCO 权重代替。

## 4. 构建完整运行包

```bash
cd "$SCARA_PROJECT_DIR"
bash tools/build-robot.sh /path/to/xiangqi14-s.rknn "$SCARA_ENGINE_DIR"
```

默认构建目录是 `$HOME/.cache/scara-robot-rv1106`；默认包目录是 `output/rv1106/scara-robot`。`SCARA_BUILD_DIR`、`SCARA_BUILD_JOBS`、`SCARA_BUILD_TESTING` 可覆盖，默认并行4、编译测试ON。测试只构建，不在交叉构建电脑上直接运行 ARM 程序。

第三个参数可改变包目录，但必须位于 `SCARA_PACKAGE_ROOT` 中（默认项目 `output`）。已有包会在新包组装完成后替换；任意现有非运行包目录会拒绝覆盖。不要把训练原件、私人文件或其他源码放进生成包。

包内包含 `scara_robot`、`config/robot.json`、`lib/`、`models/`、`engine/`、`camera-mis5001.sh`、`run-robot.sh`、许可证和 `SHA256SUMS`。模型复制为配置所需的固定文件名。`BUILD-INFO.txt` 记录预期上游提交、实际编译器版本及链接时 RKNN 库哈希；它不代替模型/真板验收。

## 5. 上传和手动运行

在替换同名应用目录前停止旧应用。若旧 `engine/` 或 `lib/` 是指向别处的符号链接，先保留原有数据，解除该链接或选择新的独立应用目录；部署脚本会拒绝沿链接覆盖旧引擎。

```powershell
.\tools\deploy-robot.ps1 -Serial '填写板子序列号' `
  -Package '.\output\rv1106\scara-robot' -Destination '/root/scara-robot'
```

脚本验证本地哈希，只上传当前包并设置权限。老 ADB 的主机退出码不可靠，所以还会执行板端 `sha256sum -c SHA256SUMS`，必须收到独立的 `SCARA_DEPLOY_OK` 才报告成功。它不会启动程序。

手动从 ADB/SSH 控制台运行：

```sh
sh /root/scara-robot/run-robot.sh
# 或提供另一个主程序配置文件：
sh /root/scara-robot/run-robot.sh config/robot.json
```

运行脚本把 `/oem/usr/bin` 加入 PATH，库搜索顺序为包内 `lib`、`/oem/usr/lib`、原有路径。未加载时会加载本板 `/oem/usr/ko/rknpu.ko`；相机模块仍由主程序的 `Camera.start()` 调用配套脚本。不会修改开机脚本、系统 RKNN 库或网络配置，也不会预先占用摄像头。

网络地址由使用者按本机网口环境配置。板子重启后若 MAC 改变，原 DHCP 租约可能不再匹配；检查地址和连接后再启动 SSH/网页查看器。构建/部署/运行脚本均不自动修改地址或 DHCP。
