# Pikafish 2023-03-05 / RV1106

复用用户指定的历史已验证版本；精确来源与SHA-256见`manifest.json`。目录包含ARMv7/uClibc可执行程序、**预解压的**NNUE、GPL许可证和对应源码包。它不是Windows可执行程序。

历史来源为本仓库`2604457`中的`output/rv1106/pikafish-validation/2023-03-05/`。源码提交为`4762ca3c77c7f8ccbf204974cf23c5ca3e8c2783`，上游[发布页](https://github.com/official-pikafish/Pikafish/releases/tag/Pikafish-2023-03-05)。没有替换为较新的大型网络。

`board-results.json`是2026-10-08的历史RV1106记录，不是本次与当前相机/NPU同时运行的新验收。该记录约90MiB常驻、Threads=1、Hash=4MiB。当前整机内存及通信监督仍需重新检查。

在匹配的RV1106 Linux上核对哈希后，在本目录启动`./pikafish`；UCI封装见`chess/pikafish.*`，要求提供此目录。合法走法使用本版本的`go perft 1`扩展，思考使用`position fen <起始FEN> moves <完整已完成历史>`及`go movetime`。不要只给当前FEN丢弃历史。

源码复现：解开`source-4762ca3.tar.gz`，在其`src`目录使用匹配uClibc工具链：

```sh
make -j4 all ARCH=armv7-neon COMP=gcc \
  COMPCXX=/path/to/arm-rockchip830-linux-uclibcgnueabihf-g++ \
  EXTRACXXFLAGS='-mcpu=cortex-a7 -mfpu=neon-vfpv4 -mfloat-abi=hard'
```

完整许可证见`Copying.txt`。本次只归档固定工件与接入代码，没有启动真实棋局或机械臂。
