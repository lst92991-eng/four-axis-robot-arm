# 棋类机器人扩展方向：开源证据与体量抽样

调研日期：2026-10-08（Asia/Shanghai）。本文件是全项目调研的扩展棋类分报告。用户的 SCARA 硬件已确定，以下其他机械结构仅用于比较软件职责与复杂度，不提出更换硬件。检索不能证明覆盖全部开源项目；只有列出的证据可核验。

## 计数方法

对 7 个仓库做浅克隆、静态读取，没有安装依赖、运行第三方脚本、编译或连接机器人。固定提交见下表。用 `git ls-files -z` 限定跟踪文件；按 UTF-8 解码（非法字节替换）后 `splitlines()` 统计物理行，`line.strip()` 非空统计非空行。**非空行包含注释和文档字符串，不等于去注释代码行。**

通用统计脚本选择 `.py/.c/.cpp/.h/.hpp/.ino`，因此原始 JSON 的全仓源码数包含测试、工具和随仓依赖。下表进一步明确应用范围，不能把各行不同比较范围简单相加或排名。README、图片、模型、二进制、配置和 CMake 没有计入源码行；它们可能是运行必需输入，未计入行数不表示不需要交付。应用体量和整套依赖体量必须分开。

复测脚本：[measure-chess-extensions.py](measure-chess-extensions.py)；逐文件结果：[chess-extensions-measurements.json](chess-extensions-measurements.json)。第三方快照位于 `C:\Users\lst\AppData\Local\Temp\scara-chess-research-20261008`，不留在项目源码树。复测命令：

```powershell
python .\research\measure-chess-extensions.py "$env:TEMP\scara-chess-research-20261008" .\research\chess-extensions-measurements.json
```

## 实测代表

| 项目 | 选定范围 | 文件数 | 物理行 | 非空行（含注释） | 技术栈与许可证事实 |
|---|---|---:|---:|---:|---|
| [Raspberry-Chess](https://github.com/yizhepku/Raspberry-Chess) | 全部 5 个 `.py` | 5 | 608 | 500 | 树莓派、Dobot、Python、OpenCV、pydobot、python-chess、Stockfish；仓库 GPL-3.0 |
| [LSS Chess-Robot](https://github.com/EDGE-tronics/Chess-Robot) | `ArmControl.py/ChessLogic.py/Interface.py/VisionModule.py` | 4 | 1,385 | 1,195 | 4DoF LSS 臂、Python、OpenCV、Stockfish、python-chess、FreeSimpleGUI；仓库 GPL-3.0 |
| LSS 随仓驱动 | `lss.py/lss_const.py`，与应用分开 | 2 | 537 | 450 | LSS 串口驱动；文件头为 LGPL-3.0 |
| [OpenChessRobot](https://github.com/renchizhhhh/OpenChessRobot) | `src/open_chess_robot/scripts/**/*.py`，含训练/评估/工具 | 71 | 10,538 | 8,963 | Python、ROS Noetic、MoveIt、Franka、ZED、PyTorch、Stockfish；可选 LLM/TTS；仓库 MIT，外部依赖独立 |
| [Star-Robot 中国象棋](https://github.com/Star-Robot/chinese-chess-robot) | `chessplayer/**/*.c/cpp/h/hpp`，剔除 `common/json.hpp` | 42 | 5,163 | 4,481 | C++11、OpenCV、RKNN、YOLOv7-Tiny、TinyROS、UCCI、yaml-cpp、ALSA/FFmpeg；根许可证 BSD-3-Clause |
| Star-Robot TinyROS | `tinyros` 的 C++/头文件 | 22 | 2,090 | 1,762 | 自制通信中间件；README 的“2112 行”仅针对 TinyROS，不是完整机器人 |
| Star-Robot 随仓 JSON 依赖 | `chessplayer/src/common/json.hpp` | 1 | 20,842 | 17,413 | nlohmann/json 3.6.1，文件头 MIT；单头文件不等于项目自研体量 |
| Star-Robot 视觉训练代码 | `yolov7/**/*.py` | 33 | 12,017 | 10,069 | 训练/转换一并计入；不能把根 BSD 标签直接推广至所有随仓第三方组件 |
| [Sunfish](https://github.com/thomasahle/sunfish) | 仅 `sunfish.py` | 1 | 662 | 589 | Python 国际象棋搜索引擎、位置评分、MTD-bi、UCI；GPL-3.0 |
| [Moonfish](https://github.com/walker8088/moonfish) | 仅 `moonfish.py` | 1 | 654 | 568 | Sunfish 改出的 Python 中国象棋引擎；GPL-3.0 |
| Moonfish UCCI 路径 | `moonfish.py/moonfish_ucci.py/tools.py` | 3 | 1,054 | 908 | UCCI 包装会导入 `tools.py`，不能只报核心文件即称完整入口 |
| [python-chess](https://github.com/niklasf/python-chess) | `chess/*.py` 完整包，不含仓库测试/示例 | 8 | 15,546 | 12,537 | Python 规则、棋谱、引擎协议、残局库等；GPL-3.0-or-later；不是搜索引擎 |

`python-chess` 单独的规则核心 `chess/__init__.py` 为 4,412/3,577 行；引擎接口 `chess/engine.py` 为 3,114/2,526 行。应用调用它很短，但不能说整套系统只有调用处几十行。

| 仓库 | 固定提交 SHA |
|---|---|
| yizhepku/Raspberry-Chess | `89b6ad54300f7fa9fc854c9f123f00cf2f04725a` |
| EDGE-tronics/Chess-Robot | `ae8b41e6c88e06cb916f2e0bb924aaf459a9cabd` |
| renchizhhhh/OpenChessRobot | `264d4e1a3d76fb069dca686d545d4bb52e00424a` |
| Star-Robot/chinese-chess-robot | `07ce94891768f307b8fe4abe504dc05d277f51bd` |
| thomasahle/sunfish | `e396948acbd80cdcbad3dccfb07e878fba9df8f4` |
| walker8088/moonfish | `52c44c3f2d187ad239a2c36325f9f224138fed53` |
| niklasf/python-chess | `0c6bdaccfcb3b09cdbe94b0f81e948cb2a356015` |

## 与教学需求直接相关的观察

**Raspberry-Chess 是最值得先读的轻量应用例子。** 它只有入口、视觉、棋局差分、机械臂、异常 5 个文件。机械臂应用层 109 行，规则与棋力由成熟库提供。连续多帧棋盘一致后才接受变化，展示了不用神经网络也能完成受控场景的思路。但它不是完整的工业质量实现：README 与 `positions.py` 明示升变未完成；静态检查 `arm.py` 可见所有 `is_capture()` 都搬走目标格棋子，未单独处理吃过路兵被吃子位于其他格的情况。不可照抄后声称国际象棋特殊规则全部物理执行正确。[固定版机械臂代码](https://github.com/yizhepku/Raspberry-Chess/blob/89b6ad54300f7fa9fc854c9f123f00cf2f04725a/arm.py)、[固定版差分逻辑](https://github.com/yizhepku/Raspberry-Chess/blob/89b6ad54300f7fa9fc854c9f123f00cf2f04725a/positions.py)。

**LSS 说明 GUI 很容易超过运动业务本身。** 4 个应用文件里 `Interface.py` 就有 819 物理行，`ArmControl.py` 为 299 行，视觉为 119 行，棋局为 148 行。首版教学应把棋盘显示、标定、开始/停止做清楚，不必加声音、计时、棋谱管理等全套界面功能。项目 README 明确提示臂展、格子间距和夹爪碰邻子的限制；这些仍要按用户现有装配验证。[官方 README](https://github.com/EDGE-tronics/Chess-Robot)。

**OpenChessRobot 是研究复杂度样本，不适合作为最小教学骨架。** 它有 ROS/MoveIt、深度识别、研究数据采集、评估和可选语音互动；10,538 行口径包含这些附属脚本，不是声称其最小对弈入口需要全部代码。[官方仓库](https://github.com/renchizhhhh/OpenChessRobot)。

**Star-Robot 的“TinyROS 两千行”不能误读为整机两千行。** 应用约五千物理行，再加两千行通信层、约一万二千行视觉训练代码与二万余行 JSON 依赖。对教学四轴平面任务，先从单程序与清晰函数开始，只有真有跨进程/跨设备的需求再加对应通信，不必先搭消息总线。[官方仓库](https://github.com/Star-Robot/chinese-chess-robot)。

**Sunfish 的“111 行引擎”不是可直接比较的工程口径。** 当前实测可读 `sunfish.py` 662 物理行；完整仓库有测试、NNUE、调参、UCI/UI 工具等。压缩、删注释后追求行数少不适合用户的可读性要求。它适合讲搜索，不代表完整规则裁判；官方 README 还列出 50 步和棋规则限制。[官方 README](https://github.com/thomasahle/sunfish)。

## 另外 3 个相关方向（未做 LOC 实测）

| 项目 | 角色和技术栈 | 本次体量证据与适用边界 |
|---|---|---|
| [Maicarons/chch-robot](https://github.com/Maicarons/chch-robot) | 中国象棋系统；Python、ONNX、Pikafish、Flask、STM32 TCP；GPL-3.0 | 树包含视觉、规则管理、机器人、Web、板端固件、相机服务、测试、VitePress 文档，属于多模块系统；未给出猜测行数。可参考状态稳定与完成应答，不必照搬全套部署。 |
| [diegocepedaw/lasergo](https://github.com/diegocepedaw/lasergo) | 围棋；Python/OpenCV、GNU Go、Arduino 舵机激光指示；项目 MIT，随附 GNU Go 另有许可证 | 根目录有 10 余个 Python 文件，另有实验代码/数据/Arduino 程序，未实测 LOC。它用激光指落点，由人代放棋子，**不是自动抓取/提子机械臂**。 |
| [lightvector/KataGo](https://github.com/lightvector/KataGo) | 围棋引擎；C++ 推理/GTP、Python 训练，支持多种计算后端 | 属于完整强引擎与训练系统，未实测 LOC。若以后要强围棋，可作为外部进程，不应把训练系统搬入教学主线。其自有内容为 MIT 型许可，外部组件独立，见 [LICENSE](https://github.com/lightvector/KataGo/blob/master/LICENSE)。 |

这里明确保留“未实测”，没有把 GitHub 仓库磁盘大小当源码大小。尝试补充 API 树元数据时遇到匿名限流，因此没有用空响应生成零行结论。

## 容易混淆的候选

- [Chessmate](https://github.com/ladyHufflePuff/Chessmate) 有 Yahboom Dofbot、树莓派、OpenCV、python-chess、Stockfish 和 Notebook，但 README 明示仅学校项目、禁止未经授权使用/修改/分发。属于公开可见代码，不能当作可自由复用的开源基线。
- [Adi-gyt/chess-robot-arm](https://github.com/Adi-gyt/chess-robot-arm) 展示 ROS 2、OpenCV 帧差、Stockfish 与 Pico 串口控制；根目录没有发现许可证，GitHub 元数据 license 为空。只能作为思路证据，不能默认为 MIT/GPL。
- Niryo 方向有 [ChessBot 官方学院报告](https://academy.niryo.com/pluginfile.php/628/mod_data/content/515/Rapport_de_stage_Eliot_BERTHOMIER_M1_SIME_2025_Projet_ChessBot.pdf) 与 [CNN-for-Accurately-Detecting-Chess-Board-State](https://github.com/Harikeshav-Shekar/CNN-for-Accurately-Detecting-Chess-Board-State)。后者 MIT、Python/PyTorch/EfficientNet/ArUco，README 说明它是更大 Niryo Ned2 项目的视觉子项目；不能因此把其根目录几个脚本当完整机械臂系统体量。
- [WSChessRobot](https://github.com/Yuxuan10086/WSChessRobot) README 称围棋，但模块名为 `gobang_ai` 并称 negamax。仅 README 不能确认具备围棋气、提子、劫与终局计分，不应据此称已实现完整围棋。

## 五子棋扩展到其他棋类：最少应保留的边界

这是根据以上源代码与任务需求作出的设计推论，不是现有项目实现承诺。不要建立插件注册、抽象工厂或多级继承。首版可以只有少量普通函数和结构体，按职责分文件。

1. **棋局只使用行列/棋子身份。** 胜负、合法性、轮到谁、AI 落点留在棋局模块；不要在 AI 中混入毫米、串口、吸盘操作。棋盘尺寸用明确配置，首版只实现五子棋规则，之后新增真实第二棋类时再改入口分支。
2. **棋盘坐标到机械坐标独立。** 标定、棋盘原点/方向、间距、取子区、弃子区、接近高度放在现有机械臂模块或一份配置中。无需为不同棋类重复运动学和通信。
3. **业务着法先转成有顺序的搬运，再执行。** 最小通用动作是 `搬运(来源, 目的)`，来源/目的可以是棋盘位置或取子区/弃子区。第一版仅落实“取子区→落点”，未来按下表扩展搬运序列；无需预建通用动作语言。
4. **识别结果和已确认棋局分开。** 相机得到的是观测，存在手遮挡/反光/误识别，先做稳定性与合法性检查。动作没执行成功前，不能把计划落点当真实落点提交；有故障就停在可理解的状态，人工恢复，不盲目重发整个搬运。

| 棋类/事件 | 实际需要的操作变化 | 不能用什么简化代替 |
|---|---|---|
| 五子棋 | 取子区→空落点 | 首版若采用无禁手自由规则，必须明示；不能同时宣称完整连珠竞赛规则 |
| 围棋 | 放一子后，可能提走多个位置；另有停着、劫/禁入点和终局计分 | 只改棋盘为 19×19、只检测连五，不能叫围棋 |
| 中国象棋 | 普通移动：起点→终点；吃子：目标→弃子区，再起点→终点 | 只识别颜色可能不足以从任意局面重建棋子种类；可先依赖已知开局与合法走法差分 |
| 国际象棋普通吃子 | 与上同；另需棋子高度/夹取条件 | 平面圆片的操作参数不能未经验证直接用于立体棋子 |
| 王车易位 | 一步棋对应两次搬运 | 不能固定“一步棋只搬一个子” |
| 吃过路兵 | 被吃子的位置不同于目标格 | 不能一律清空目标格 |
| 升变 | 更换棋子类型，需备用棋子或人工确认 | 不应为了首版兼容预做自动换子库；未来可先定义明确人工处理步骤 |

现阶段最有价值的兼容是“棋局不依赖机械坐标、运动不依赖五子棋规则、一步棋允许对应顺序操作”，而不是提前实现多棋种大框架。硬件尺寸、到达范围、夹取/吸取方式、回零与限位，待已收到的装配文件完成 CAD 几何解析及实际控制器信息核对后确定。
