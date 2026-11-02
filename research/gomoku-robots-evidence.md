# SCARA 下棋课程：五子棋及轻量机械臂公开项目调查

调查日期：2026-10-08（Asia/Shanghai）。这是公开可发现项目的广覆盖样本，不能宣称穷尽互联网。用户硬件已确定，已提供 SolidWorks 装配压缩包；机械输入核查另见 `hardware-input-notes.md`。下文比较软件和教学结构，不替用户更换机械结构。

## 口径及复查材料

- 浏览项目主仓库 README、源文件及作者说明；默认分支提交 SHA 和最后提交时间通过 GitHub REST API 读取。时间仅代表可见提交，不证明仍有维护或工程质量。
- 下载均在 `%TEMP%/scara-gomoku-research-20261008`，没有运行任何第三方程序。项目目录仅保留本文、元数据、逐文件计数和计数脚本。
- 表中「物理行 / 非空行」均包含注释，不是去注释 SLOC；计数包括仓库已提交的演示、测试、旧版本、训练代码及厂商头，不能称为原创业务代码量。排除 `.git`、`__pycache__`、`node_modules`、`Debug`、`Release`、`build` 目录。图片、权重、音频、CAD、二进制、文档及构建配置不计入此源码行数；其交付和运行成本仍然存在。
- Notebook 只抽取 code cell 源码，排除 JSON 格式和输出内容。文件扩展名白名单以及完整排除规则见 [count_gomoku_sources.py](./count_gomoku_sources.py)。源码清单及原始字节 SHA-256 见 [gomoku-source-counts.json](./gomoku-source-counts.json)。这些是调研样本记录，不是用户项目的冻结基准。
- 仓库元数据与完整提交 SHA 见 [gomoku-repository-metadata.json](./gomoku-repository-metadata.json)。GitHub `size` 是仓库存储信息，不拿它当代码量或程序运行内存。
- 没有真机复现、编译或完整行为测试。README 的完成性描述与代码静态检查分别表达。没有 LICENSE 的项目仅归为“公开源码”，不直接认定具备开源再分发许可。

## 重点比较

| 项目 | 技术栈与机构 | 已提交源码体量：文件数；物理行 / 非空行 | 可见最后提交；许可证 | 对当前项目的价值及局限 |
|---|---|---:|---|---|
| [TooAboil/FiveInARowRobot](https://github.com/TooAboil/FiveInARowRobot) | **四自由度 SCARA**；工业相机；Python、OpenCV、pygame、串口；另有 MFC C++ 控制程序 | 21；**2,247 / 1,981** | 2018-11-24；未见 LICENSE | 关联度最高，课程项目，10×10 棋盘；棋盘视觉、优先级落子、查表运动、播报。依赖既有机械控制器，不能把上位机体量当完整 SCARA 固件体量。 |
| [ese-519/GomoBot](https://github.com/ese-519/GomoBot) | Hexbot/Hexarm 吸盘臂；PC Python、OpenMV MicroPython、OpenCV C++、mbed LPC1768、UART | 9；**1,220 / 1,013** | 2018-12-15；未见 LICENSE | 视觉—棋局—落子闭环清楚，主界面是终端。包含重复 OpenMV 脚本；驱动现成臂，并非从电机底层实现。 |
| [52HZ-GGG/Gobang-Human-Machine-Game-System](https://github.com/52HZ-GGG/Gobang-Human-Machine-Game-System) | Dobot 吸盘；Atlas 200I DK A2；Python、OpenCV、NumPy、ipywidgets、pydobot、pyserial、Jupyter | 1 Notebook；**1,461 / 1,261** | 2026-05-18；MIT | 单文件演示链完整，9×9 五子棋、贪心/Minimax+αβ、视觉时间滤波、标定与取放。教学时可以拆成几个直观模块；不应因开发板名自动增加 NPU/深度学习课程。 |
| [muhuahu/Dofbot_GoBang](https://github.com/muhuahu/Dofbot_GoBang) | Dofbot 舵机串联臂；树莓派 Python/OpenCV+ESP32 MicroPython；Wi-Fi/socket | 6；**1,194 / 1,044** | 2025-04-12；未见 LICENSE | 很小的上下位机样例；ESP32 使用大量逐点舵机姿态。静态发现 `ConvertUtil`、`SocketIoutil` 导入对应文件未在仓库，复现不完整；算法名按作者称为 A*，未验证其博弈正确性。 |
| [Robin-WZQ/Gobang-Chess](https://github.com/Robin-WZQ/Gobang-Chess) | 树莓派 ArmPi 串联臂；Python/OpenCV、αβ、TensorFlow/Keras 模型、PyTorch、厂家 ArmIK/HiwonderSDK、socket | 34；**3,479 / 2,951** | 2023-04-16；MIT | 中文视觉/博弈/控制三单元；含 `lab3/train`、`lab3/test` 等重复与实验。厂家依赖不在当前源码量内，且说明把 socket 网络通信称为串口，教材不能照抄。 |
| [KITTIYOJIANG/gomoku-robot](https://github.com/KITTIYOJIANG/gomoku-robot) + [gomoku-arm-controller](https://github.com/KITTIYOJIANG/gomoku-arm-controller) | STM32 舵机臂的 Python 上位机；OpenCV、NumPy、PyQt5、pyserial、pytest；启发式浅搜索 | 47+9；**7,255+407 / 6,039+307** | 2026-08-29 / 2026-05-19；均未见 LICENSE | 规则/视觉/动作边界清楚，包含 GUI、棋谱、回放、标定、mock、测试和多个原型；README 明说真实硬件闭环仍待联调。所谓 arm-controller 是 Python 控制包，没有 STM32 C 固件，不能误记成已交付完整下位机。 |
| [hri5-boardgame-robot/Koch_Gomoku_robot](https://github.com/hri5-boardgame-robot/Koch_Gomoku_robot) | Koch 串联臂、Dynamixel；Python、PyTorch、MCTS、MuJoCo、mink、OpenCV | 20；**3,202 / 2,650** | 2025-02-03；MIT | 含视觉标定、抓取、仿真、训练与模型；虽源码不大，但神经网络、仿真、IK 依赖的教学负担高于表面行数。适合进阶扩展，非最小首版。 |
| [Agenicy/Gomoku-With-Robot-Arm](https://github.com/Agenicy/Gomoku-With-Robot-Arm) | TinkerKit Braccio；Python、Keras/AlphaZero/MCTS、串口、GUI；Unity 2019.4.3f1 C# 前端 | 33；**6,190 / 5,165** | 2020-10-01；MIT | 包含训练与两套交互入口，作者说明机械臂运行需要修改端口与脚本，且操作复杂；不适合以“代码最少”直接套用。 |
| [omegaliii/gomoku_robot](https://github.com/omegaliii/gomoku_robot) | Baxter；ROS 1/catkin、MoveIt、Python、OpenCV | 14；**2,012 / 1,635** | 2019-01-17；未见 LICENSE | 棋盘视觉、AI 和机械臂节点分开；README 还描述非法落子检测。源码只有两千行，但借助 ROS/MoveIt/Baxter 外部栈，迁到教学 SCARA 没有体量优势。 |
| [raghavansh/Tic-Tac-Toe-playing-4dof-robotic-arm](https://github.com/raghavansh/Tic-Tac-Toe-playing-4dof-robotic-arm) | 4DOF 舵机串联臂（并非已证实 SCARA）；Arduino Firmata；Python/OpenCV、pyfirmata、NumPy、imutils、sympy；Minimax | 2；**1,096 / 992** | 2022-06-28；未见 LICENSE | 自有 `code.py` 只有 **273 / 250**；另 **823 / 742** 是 StandardFirmata。适合理解最小闭环，不可把 273 行与自写电机固件的项目直接比较。 |

## 最直接的 SCARA 样例拆解

[TooAboil/FiveInARowRobot，提交 9d1fa5cb](https://github.com/TooAboil/FiveInARowRobot/tree/9d1fa5cb9a4ac91d80077e78a177cad4bf6e794a) 的 Python 主链只有五个文件：

| 文件 | 物理行 / 非空行 | 职责 |
|---|---:|---|
| `python/main.py` | 130 / 107 | 拍照、等待人落子、调用决策和执行、播放语音 |
| `python/ImgProcess.py` | 201 / 176 | 棋盘图像处理与状态识别 |
| `python/game.py` | 515 / 478 | 棋局和优先级决策 |
| `python/SolveAngle.py` | 122 / 122 | 查表得到机械臂控制量 |
| `python/serial_commu.py` | 80 / 66 | 串口通信 |
| **主链合计** | **1,048 / 949** | 不含测试、厂家库和底层电机固件 |

完整计数还含 C++ MFC UI、Pcomm/WinIo/Kld 头和 Python 实验程序；已排除 `Cpp/Debug/Pcomm.h` 的重复。Python 主链没有通用插件、分布式服务或深度学习，证明简单分工足以表达下棋闭环。它依赖固定棋盘、固定标定及既有驱动，不能推导出用户的完整 SCARA 教材也只需一千行。

源码 `main.py` 导入 OpenCV/pygame，调用查表和串口；声音来自 `audition/*.mp3`，更准确的表述是播放预录片段，而不是额外接云端 TTS。机械参数、COM 口、相机裁剪区域均硬编码，需按用户装配和实测参数重新做清晰配置。

## 其他覆盖及排除原因

1. [zzh112119/gomoku_game](https://github.com/zzh112119/gomoku_game)，SHA `7bd09db668da0cc39d3deb3db4bb7ed4f4bbb9f5`，2018-12-13，8 源文件 **1,135 / 940**；与 ese-519/GomoBot 属同一课程作品谱系，不重复当作两套独立架构。mbed `main.cpp` 仅 12 行，是 OpenMV 串口转发，不是机械臂运动控制器。作者 [Devpost 项目说明](https://devpost.com/software/gomokubot) 描述完成取放和人机对弈，并记录过规划路径在无障碍场景中反而增加误差与时间；这是该项目经验，不能推广为所有机械臂都不需要轨迹规划。
2. [Ciaran-byte/FiveChess](https://github.com/Ciaran-byte/FiveChess)，SHA `7de3fb27235ddcaa5c3f7b79e0c342d30e03b6cc`，2020-11-06。3DOF 串联臂，C++/OpenCV，上位机与部分机械资料公开。作者在 [项目说明](https://damodev.csdn.net/68243dfca5baf817cf4bc84e.html) 明确说下位机误删，机械资料也有缺失；不可列成可完整复现套件。未实测行数，API 语言字节为 C++ 90,229、C 265；字节不等于行数。未见许可证。
3. [KanghwaSisters/24-2-Omok](https://github.com/KanghwaSisters/24-2-Omok)，SHA `f5d530d907157c7d8955efcd4d870e7124c7993c`，2025-03-25。4DOF 吸盘臂、Arduino Uno/Servo、Python、OpenCV、PyTorch、Flask、Notebook，AlphaZero 式训练。README 提及 4090 上训练 20 小时（作者给定配置下的描述）；不适合作为本课必修依赖。未实测 LOC；API Python 字节 103,226、C++ 17,759，Notebook 原文件 1,309,379 字节，不能将 notebook JSON 体积当源码量。未见许可证。
4. [wholol/Tic-Tac-Toe-Robot-Arm](https://github.com/wholol/Tic-Tac-Toe-Robot-Arm)，SHA `b9c3dd9996df6f68dc3a45ce47b3b6f9a7220c33`，2020-03-09。Python FABRIK 仿真、Minimax 和 Arduino C++，3 源文件 **1,028 / 790**；README 标注 In Progress，不能当完成机器人。未见许可证。
5. [OSU-AIMS/tic-tac-toe](https://github.com/OSU-AIMS/tic-tac-toe)，SHA `33768e68941356f1b44fad6117cf907bb244ed9a`，2022-06-01，Apache-2.0。面向 Motoman MH5L 六轴工业臂、ROS Melodic、Python 2.7.17；不属于轻量 SCARA 首选。未实测行数，API Python 120,060 字节，不与本表行数混比。
6. [Logan-Martin/Tic-Tac-Toe-Delta-Robot-Arm](https://github.com/Logan-Martin/Tic-Tac-Toe-Delta-Robot-Arm)：课堂 Delta 三舵机+Arduino Metro+9 按钮作品，主要是两人按键选择落点。README 自述取子、输赢判定等问题，不能当已完成机器 AI 人机博弈系统。没有实测 LOC。
7. [执念执战 Gitee 项目入口](https://gitee.com/zhinianzhizhan) 及 [SCARA XY 控制文章](https://www.cnblogs.com/zhinianzhizhan/p/17360467.html)：Python+PyQt+CoppeliaSim 的下棋机器人前期仿真。作者明确未完成插值及全部逆解分支；更适合作为运动学辅助资料，非完整机器人。
8. [Miq26/gomoku-thesis-proj](https://github.com/Miq26/gomoku-thesis-proj)：论文中存在该链接，但调查时 API 返回 404；不补猜技术栈或规模。

## 给主线程的选型结论（条件性）

1. 课程首版可采用五个清楚职责：棋盘状态/规则、五子棋决策、棋盘识别、棋格到机械坐标的标定、机械动作与通信。TooAboil 的五文件主链最值得学习组织方式；不是建议照搬老硬件接口。
2. 许多“几百行机器人”把难点交给现成机械臂 SDK、Firmata、ROS/MoveIt 或厂商控制器。体量比较必须同列底层来源、标定方式、视觉和 AI 依赖，不能只挑应用行数最少的仓库。
3. 首版决策用规则评分和浅层搜索足够讲清流程；PyTorch/AlphaZero、Unity、ROS/MoveIt 都不是完成五子棋闭环的必需项。是否用 Python/OpenCV 上位机加 MCU C 控制，等用户确认计算平台与电机/控制器后再定。
4. 兼容后续棋类的合理边界是稳定的棋盘坐标和“取/放/移/移除”动作；棋类只负责合法性、胜负、落子或移动决策。第一版写直接函数即可，无需提前做插件注册表、抽象工厂、消息总线或多进程服务。
5. 五子棋和井字棋都主要是新增棋子；围棋需额外提子，象棋/国际象棋需移动和吃子，且视觉要分辨棋子身份。不要为了未来兼容，把这些功能都提前塞进五子棋版本；但避免通信协议永远只能表达一个 `row,col` 落点。
6. 用户已确定四轴机构，不能因为别人的工程小就更换成机械结构不同的 Dobot、Baxter、Braccio 或 Dofbot。硬件相似性、软件完整性和教学复杂度分别评估。

## 检索路径与局限

使用中文/英文组合检索：五子棋机械臂、五子棋机器人、OpenMV、STM32、SCARA gomoku、gomoku robot arm、tic tac toe robot arm、Gitee 下棋机器人，并结合 GitHub 仓库搜索发现的候选。排除纯线上五子棋 bot、与棋类无关的机械臂、仅营销演示而没有源码的产品。对代表项目固定 SHA 静态下载；没有把 fork、改名或课程内同源仓库算成独立方案。后续 GitHub 公共 API 触发限流，Ciaran、Kanghwa、OSU 的源码树未进一步下载，因此保留未实测而不补造行数。
