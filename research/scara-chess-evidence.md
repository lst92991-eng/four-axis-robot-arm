# SCARA 下棋方案补充证据

检索日期：2026-10-08。以下均为只读源码检查，未编译、运行或连接实物。物理行包含空行与注释；非空行只剔除空白行，仍包含注释。逐文件数据见 `scara-chess-code-counts.json`。本批源码快照在系统临时目录 `scara-root-research-20261008`，未加入项目生产目录。

| 项目与版本 | 栈与用途 | 实测范围和体量（文件数；物理行 / 非空行） | 限制与教学判断 |
| --- | --- | --- | --- |
| [rpd123/SCARA](https://github.com/rpd123/SCARA) 对应 [chess-robot](https://github.com/rpd123/chess-robot/tree/a825da104e38a01662fd944ea9ec19043232cec5)，提交日期 2025-10-22 | Raspberry Pi / Windows、Python、OpenCV、NumPy、串口、Stockfish；SCARA 改造机械设计与软件分仓 | `version2` 7 个应用/标定脚本：1,575 / 1,388；另 `ChessBoard.py` 1,360 / 1,109；排除其他版本、旧 robotmove、testport | 只统计一个版本，避免历史版本重复。ChessBoard 文件声明 GPL；仓库旧目录有 GPLv3，当前应用与固件许可覆盖须逐项确认。物理设计是 Pybot 改型，不能将其轴数直接等同用户的四运动轴。 |
| [rpd123/Arduino-Firmware](https://github.com/rpd123/Arduino-Firmware/tree/17b176ce2fa3b05e2faf8ca5e265c5719b489a7e)，2023-12-30 | Arduino C++，串口、插补、归零、限位、夹爪；README 要求显式开启 `SCARA` 配置 | `robotArm` 除文件自称不使用的 `inverse_kinematics.cpp`：25 文件，1,774 / 1,520；不含 Arduino 核心和库 | 可以参考运动与取放分工；多夹爪、多几何选项并非本课都要保留。未发现顶层 LICENSE。不是纯四轴完整闭环范本。 |
| [dongvua90/ChessMainboard_Scara](https://github.com/dongvua90/ChessMainboard_Scara/tree/7ed03bec4358474a340271a3184e80a7b7cd69bc)，2022-01-26 | STM32F401CC、C、HAL/CMSIS、FreeRTOS、AS5600、I2C、步进运动；下位机工程 | `Core` 35 文件 6,940 / 5,940，含 Cube 生成代码和适配库，**不能视为纯自有代码**；`Drivers` 69 文件 104,944 / 90,419；`Middlewares` 30 文件 29,332 / 25,767 | 实例说明 HAL 与 RTOS 会让总行数远超业务量。只包含控制板，视觉、棋力不在此仓库；源码中 motor_j1/j2/j3 不证明四个运动轴。未发现顶层 LICENSE。 |
| [MostafaOkasha/ChessMate](https://github.com/MostafaOkasha/ChessMate/tree/aa4bc1d021e552daef21714f33f99daedd46b475)，2019-08-25 | SCARA 国际象棋；Python、OpenCV、python-chess/旧 chess.uci、Stockfish、neuralchessboard、boto3/S3、G-code | 根目录与 `lsh` 排除实验 `potato.py`，11 文件 796 / 599；排除 `scrap_files`，其中仅 `get-pip.py` 就有 20,890 物理行 | 值得参考“已知初始局面+颜色/占用变化推断着法”。实际有云存储、旧 API、绝对路径和未随库提供的依赖；不能把 599 非空行当成可独立运行总量。未发现顶层 LICENSE。 |
| [AvQ1301/chinese_chess_bot_307](https://github.com/AvQ1301/chinese_chess_bot_307) | README 描述 Raspberry Pi/PC、Arduino Mega、Python/OpenCV、YOLOv8、Pikafish、PlatformIO，MIT | 未做整仓计数。主固件文件 blob `d44c359ee0d7367b2f963027654e10ad6ef902f2` 719 字节 | [固件 main.cpp](https://github.com/AvQ1301/chinese_chess_bot_307/blob/main/firmware/src/main.cpp) 的电机、夹爪、归零、解析和执行均仍标 TODO。属于部分实现，不能依据 README 当作完整可复现机器人。 |

GitHub 仓库页面中的磁盘 `size` 不等于源码行数。本批统计后又按文件职责划分范围，保留原始逐文件数字，避免用第三方库、下载脚本和备份文件夸大教学工作量。统计范围并非许可证归属证明。
