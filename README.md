# SCARA 中国象棋机器人

课堂上层使用 C++17、OpenCV 5.0.0、RV1106 RKNN 和 Pikafish。棋盘统一用 **（行，列）**：行 0–9 从黑方底线走到红方底线，列 0–8 从红方视角左到右。上位机显示数字坐标，Pikafish 的 UCI 边界和历史棋谱保留其协议编码。

## 项目入口

| 任务 | 入口 |
| --- | --- |
| 人工选择测试、脚本与 Codex 直接运行 | [台架菜单](bench/START.cmd) · [编号、命令和日志](bench/README.md) |
| 课堂流程、各层职责及下一轮验收 | [上层复现与重构](docs/上层复现与重构.md) |
| 教师版 BMS 对照和课堂写法 | [课堂代码写作规范](docs/课堂代码写作规范.md) |
| 阅读 C++ | [三点标定与归格](vision/geometry.cpp) · [整局、一手棋与吃子](app/game.cpp) · [取子、落子、盘外弃子](motion/plan.cpp) · [RV1106 设备接入](app/rv1106.cpp) |
| R13 固定装配与重新标定 | [带图零件表、安装、三点和 90 点](docs/cad/R13装配与标定.md) |
| R10 棋盘固定基准 | [装配核对和俯视图](docs/cad/R10棋盘固定装配核对_2026-11-23.md) |
| 驱动板与历史实机证据 | [轴参数表](firmware/src/config.c) · [初版验收记录](docs/初版验收与跨设备交接_2026-11-19.md) · [原交接](temp/交接.md) |

## 从这里运行

在项目根目录运行 `bench\01-build.cmd` 构建 Windows 台架。双击 `bench\START.cmd` 可选测试。命令行用 `bench\20-grid90.cmd --plan` 查看 90 点定位，用 `bench\21-pick90.cmd --plan --start 3 4` 规划从第 3 行第 4 列开始的整盘取放，用 `bench\40-game-sim.cmd --all` 运行 C++ 对局回归。

固定后用 `bench\15-site.cmd 实测表.txt --install` 生成本机 C++ 数值表，再运行 `bench\01-build.cmd`。坐标、工作高度和观察位都随程序编译。没有另选本机表时，构建采用仓库已有的实测数值。[Setup_game](app/installation.hpp) 中直接填写这些数值。`bench\50-game.cmd` 打开的面板从内存接收棋局和相机画面。同一面板进程支持“接续上局”，退出面板进程会清空棋谱。

## 运行边界

- `app/`、`vision/`、`chess/`、`motion/` 是学生阅读的 C++ 主线。相机、OpenCV、RKNN 模型与运行库、Pikafish 和 NNUE 是实机所用的资源。中文棋名由 OpenCV 5 的内置 uni 字体直接绘制。棋局、合法走法和预览均在内存中传递。
- Windows 已走通的整机路径通过 [USB-CAN/ADB 台架](bench/field) 调用现场设备。Python 3.13 和测试脚本属于这条台架路径。Linux 原生入口的四个 `Drive_*` 硬件函数仍待接入与实机验收；现有板端 v3 接受成段绝对目标。原生执行器的周期参考不能直接逐周期重发为 v3 MOVE。
- 当前固件包为 [9fc6a5de](images/common-20261123-9fc6a5de/joint-app-manifest.json)。J1～J3 实板回读过此版本，J4 有同版产物，本次实板状态仍为 `41f870bf`。J1～J3 正常运动母线门槛 40 V，实际停机与联锁原因可从 CAN 状态读出。
- `tests/` 存放仿真与回归，`output/bench/` 存放本机运行结果，`docs/evidence/` 保存选定的验收记录。2026-11-16 的完整历史交接和压缩包索引见[重构交接](docs/重构交接_2026-11-16.md)。
