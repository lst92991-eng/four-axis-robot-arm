# 实机与离线测试入口

人在 Windows 上双击 [START.cmd](START.cmd)。自动化从任意目录调用同一组编号 CMD。编号按职责固定，日志中的格点一律写成 **（行，列）**。黑方底线为行 0，红方底线为行 9，红方视角从左至右为列 0–8。示例中的 `--start 3 4` 表示第 3 行第 4 列。

| 编号 | 用途 | 直接运行 |
| --- | --- | --- |
| [00](00-check.cmd) | 被动读取 CAN 的构建号、电压、角度、参考和故障。单次等帧窗口 1 秒 | `00-check.cmd`，四轴首装后用 `00-check.cmd --four` |
| [01](01-build.cmd) | 一次构建台架 DLL、C++ 格点、视觉回放与模拟对局程序 | `01-build.cmd` |
| [10](10-flash.cmd) | **空白板 SWD 首装**。一块 CMSIS-DAP 接一块实际标好轴号的板 | `10-flash.cmd J2` 校验；`10-flash.cmd J2 --write` 烧录并读回 |
| [11](11-update.cmd) | 已有本项目引导程序的板，经 Candlelight 更新共用应用，回读节点和版本 | `11-update.cmd J1 J2 J3 J4` 校验；加 `--write` 后按提示重启板电源 |
| [12](12-camera.cmd) | 取现场相机的一张新原图，显示分辨率 | `12-camera.cmd --url rtsp://板子地址/live/0 --show` |
| [13](13-electrical.cmd) | 新板单轴电角标定，保存并回读该轴零点与方向 | `13-electrical.cmd J2` 查看；`13-electrical.cmd J2 --run` 才通电标定。已有有效标定的板可跳过 |
| [14](14-reference.cmd) | 三轴失能且实物已对准已知标记时，写回并逐轴回读机械参考 | `14-reference.cmd --angles J1角 J2角 J3角` 查看；实物已对准保存的观察姿态时用 `14-reference.cmd --at-park --write` |
| [15](15-site.cmd) | 把已实际对中的三组像素和关节角、Z 示教值制成本机安装记录 | `15-site.cmd --template 测量.txt`；填好后 `15-site.cmd 测量.txt` 演算，加 `--install` 生成本机 C++ 表，再运行 `01-build.cmd` |
| [20](20-grid90.cmd) | **实机定位**。空吸头蛇形扫 90 个交点，约在已示教棋子顶面上方 2 mm 记录，再全高横移，结束回观察位 | `20-grid90.cmd`；`20-grid90.cmd --plan` 离线规划 |
| [21](21-pick90.cmd) | **实机取放**。空盘仅放一颗样子在指定起点，逐格搬运，最后把棋子送回起点、机械臂回观察位 | `21-pick90.cmd --start 3 4`；离线加 `--plan` |
| [30](30-vision-replay.cmd) | 用保存的图像、NPU 输出检查 C++ 预处理、解码和检测结果 | `30-vision-replay.cmd` |
| [40](40-game-sim.cmd) | 运行正式 C++ main，使用模拟相机、引擎与驱动 | `40-game-sim.cmd` 演示两回合；`40-game-sim.cmd --all` 完整场景回归 |
| [50](50-game.cmd) | 打开本机象棋面板，点击开局后才进入实机。人机与自对弈使用 DLL 中同一份安装几何 | `50-game.cmd`；`50-game.cmd --mock` 可先练界面 |

## 工作站只准备一次

台架使用 Windows x64、Python 3.13、OpenCV 5.0.0 C++ SDK、CMake、Ninja 和 Visual Studio C++ 工具。将 SDK 的 CMake 目录设为 `OpenCV_DIR`。台架 Python 需要 NumPy、OpenCV Python 及 Candlelight 所需的 PyUSB/libusb，模型推理实际走 RV1106。已装环境可以用 `SCARA_PYTHON` 指定解释器，项目的 `.venv` 和当前实验机的已装环境也能直接识别。烧录和测试脚本运行在工作站；原生 C++ 对局程序的依赖见[复现流程](../docs/上层复现与重构.md)。

SWD 使用雅特力 OpenOCD 的 `interface/cmsis-dap.cfg` 和 `target/at32m416xx.cfg`。它可以在 PATH 中，也可以通过 `--openocd` 指定。脚本校验选定 HEX、轴记录和应用 CRC，核对 MCU，编程后校验写入内容并比对最后 4 KiB 电角标定区。四块新板的电角零点分别在各自实板上标定，已有标定随板保留。默认包为 [9fc6a5de](../images/common-20261123-9fc6a5de/joint-app-manifest.json)，可用 `--package` 选择教师验收的其他同格式版本。J1～J3 有此版板上回读，J4 的此版完成离线镜像校验，本次状态帧回读为 `41f870bf`。

现场使用 ADB 时可用 `SCARA_ADB` 指定工具路径，多块 RV1106 用 `ANDROID_SERIAL` 选板，多只 USB-CAN 用 `SCARA_CAN_SERIAL` 选适配器。对局把 `config/site/installation.hpp` 编入程序。CMake 可用 `SCARA_INSTALLATION_HEADER` 指定另一份实测 C++ 表。独立的 20/21 巡点工具仍可通过 `--site` 指定 `.site` 或旧 JSON。单机同时只由一个控制程序持有 USB-CAN。

## 每项测试的约定

- 一个编号只负责一项测试。CMD 定位工作目录和解释器，测试体调用正式 C++ 的格点与逆解、共用的 CAN 和 H 桥实现。新增测试先在本表登记编号、输入、实物动作和完成条件。
- 测试代码按“取得输入、生成动作、执行、记录”顺序写。角度、毫米和时间在变量名或接口上标单位。动作入口提供离线规划，硬件访问放在执行阶段；故障保留节点、详情和最后已完成的格号。诊断命令在日志首行标明真实、模拟或规划。50 对局面板直接使用内存和进程管道。
- `20` 和 `21` 自己读取一次本次 CAN 状态，在同一连接完成后续动作。硬件通信期间的心跳和到位反馈随动作进行。无需先跑 `00`，也无需翻历史日志。异常直接报告节点、故障或缺少哪份现场测量。
- `--plan` 使用规划数据，不连接电机或 H 桥。本机还未示教时会明确使用 [历史离线夹具](reference/game.json)。实际运动使用已经示教的本机坐标和高度。J2/J3 选解按当前姿态与 CAD 可达区执行，J4 保持。
- 编号诊断及回放命令打印自己的 `output/bench/runs/<时间>-<编号职责>/result.jsonl`。到位点逐条落盘。退出码 0 为完成，1 为运行失败，2 为参数错误，130 为人工中断。实机 `21` 的到位记录与 H 桥应答证明动作指令完成，实际棋子是否落稳由现场观察或对局视觉确认。
