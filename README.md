# 按逐行手写方式整理的当前版本（2026-11-14）

用户这次纠正的是**代码书写方式**。本轮对照原电机代码的 `pid.c`、`wheel_control.c`，实际整理了动作构造、配置读取、坐标计算、棋局字符转换和引擎读写。共三轮，不用改名、减行或测试通过来代替手写体验。

- [先看取放步骤 Pick_plan](motion/plan.cpp)：先说明变量和单位，逐字段设置 pickup、travel、release、pulse，最后按顺序加入动作数组。不再靠 `{MoveZ, {}, ...}` 的位置猜参数。
- [再看回合流程](app/game.cpp)：准备动作、开局确认、人类回合和机器人回合仍按原流程执行。
- [配置](app/game_config.cpp)与[文件读取](vision/io.hpp)：先声明接收数据，再用 `readStorage(path, file)`、`readBytes(path, bytes)` 写入；不在一行中套多个对象构造。
- [坐标处理](vision/geometry.cpp)：先写检测中心，再算格点、最近交点和基座XY；投影结果通过 `project(matrix, point, result)` 写出。
- [引擎通信](chess/pikafish.cpp)：具名的毫秒数、读写字节数，代替嵌套 `decltype`、模板和类型转换；短读短写和资源释放仍保留。
- [全部C++完整阅读稿](output/game-style-20261114/round-03/cpp-code.txt) · [仅C++源码包](output/game-style-20261114/round-03/cpp-code.zip) · [源码与验证包](output/game-style-20261114/game-application-source.zip)。

本轮有效行数 **2465 → 2758**，增加293行，主要来自逐字段赋值、声明中间数据和分开写消息字段；没有新增业务功能，也不是进一步压缩。全部CPP、头文件、内部实现和仿真后端都计入。六个已认可视觉文件仍为408行，保持原哈希。文件执行协议也保留，本轮没有替用户选择新的硬件接口。

三轮均通过17组CTest、60项Python检查；保存图像/张量/ONNX回放和4份几何输出与起点一致。改前改后完整程序的15场景状态和动作内容一致，仅排除运行编号、时间戳和票据编号。证据见[逐项自审](docs/evidence/2026-11-14/game-style-20261114/self-review.json)、[程序对照](docs/evidence/2026-11-14/game-style-20261114/program-equivalence.json)、[实际行数](docs/evidence/2026-11-14/game-style-20261114/final-metrics.json)。没有新增准确率样本，也没有连接硬件；Linux真实进程、ARM/NPU和整机验收仍是后续事项。

仍需课堂讲解的知识包括引用、vector、OpenCV矩阵、相机线程同步和Linux进程/管道。它们没有被藏到另一层或移出统计；本次去掉的是密集、隐式的写法，不能声称这些知识本身消失了。

以下为此前版本记录，旧行数和旧读取接口以本节及当前源码为准。

---
# 完整主程序的手写复核版（2026-11-14，上一轮）

本轮重新对照原电机代码的参数出入、集中声明和顺序初始化写法，当前全部自有C++为**2465有效行**，比上一版2472行净减7行。主要改善数据流和重复处理，不把这个数字说成大幅压缩，也不替用户断言已经完全符合课堂手写体验。

- [当前主程序](app/game_main.cpp) · [回合流程](app/game.cpp) · [全部C++完整阅读稿](output/game-lesson-20261114/round-05/cpp-code.txt) · [源码与验证包](output/game-lesson-20261114/game-application-source.zip)。
- 命令变为 `Operator_read(phase, prompt, command)`，通过输出参数写回；两种IO共用Command_parse，退出集中处理。
- 主流程能直接读到 Runtime_open、Vision_observe、Engine_query、Robot_execute 和 Match_report，各自职责不再全部叫IO。
- 删除仅转发参数的 Session_acceptHuman、FixedBoard_assign。直接调用Game_acceptHuman与assignPieces，没有挪到别处的替代包装。
- 示教数值直接写到j2Deg、j3MotorDeg和capturePlaceMm；预览与消息共用原子发布实现。
- Session_plan先把步骤计算/合并完毕，再锁定走法；不再复制整份棋局和历史。原pending、合法走法、视觉确认和恢复行为保留。

本轮17组CTest、60项Python检查通过；还分别运行了改前和改后的可执行程序，在同样15种场景中比较完整棋局状态和全部动作内容，结果一致（时间戳和每次运行的票据编号除外）。原视觉六文件、ROI、模型与几何输出不变。见[前后程序对照](docs/evidence/2026-11-14/game-lesson-20261114/program-equivalence.json)和[行数/变更记录](docs/evidence/2026-11-14/game-lesson-20261114/final-metrics.json)。

执行接口的偏好问题尚未答复，本轮维持原文件请求/回执兼容。真正更明显的结构删减是改成直接C++硬件函数接口；没有把未获答复当作同意删除已交付协议。真实SPI-CAN/H桥与实机验收仍未进行。

以下为此前完整主程序与固定核心的说明，旧函数名和行数以本节为准。

---
# 完整对弈主程序（2026-11-14）

**第一目标已经补齐：可执行的C++主程序贯通整个对弈循环。** 当前自有C++总2472有效行，包含所有实现和头文件；相对之前1800行核心新增672行。新的硬件执行器和实机验收仍待你后续推进，主程序不会伪造真实执行成功。

## 先看与先运行

- [完整主入口 game_main.cpp](app/game_main.cpp)：75有效行，初始化、查询引擎、区分双方回合、结束与清理。
- [回合流程 game.cpp](app/game.cpp)：131有效行，初局确认、玩家走棋、机器人取放与失败恢复。
- [配置读取](app/game_config.cpp)、[实机IO](app/game_live_io.cpp)、[执行请求/状态输出](app/game_files.cpp)、[仿真IO](app/game_sim_io.cpp)。
- [全部C++阅读稿](output/game-main-20261114/round-02/cpp-code.txt) · [仅C++源码包](output/game-main-20261114/round-02/cpp-code.zip) · [完整源码与验证包](output/game-main-20261114/game-application-source.zip)。

本机直接跑完整仿真，不需要板子、相机、模型或引擎：

```powershell
.\build\commit-verify\scara_game_sim.exe tests/fixtures/game-app/simulation.json
```

它使用和实机目标相同的main/回合逻辑，模拟两轮走棋和机器人吃子，然后退出。终端明确标记SIMULATION，结果在 [status.json](output/game-main-simulation/status.json)，已确认历史应为 `a3a4 a6a5 a4a5 a9a5`。这是真实主程序的离线运行，感知、引擎回复和机械结果是场景夹具，不是实际硬件验收。

## 主程序已经处理的流程

初始化固定90点、机械参考和示教高度 → 操作者确认进入观察位 → 新图识别与开局确认 → 玩家走完并移开手后回车 → 确认唯一合法变化 → 引擎应手 → 搜索后重新观察 → 生成并发布取放步骤 → 等待已停稳的执行结果 → 回观察位后重新识别 → 吻合才提交历史 → 下一回合。

普通搬子、吃子、红黑方选择、任意已确认起始盘面、无合法走法、认输/和棋和重复局面人工裁定均有对应流程。执行失败或盘面不符时不盲目重发；retry只观察，accept需要人工确认且盘面吻合，restored必须看到完整复位盘面才清pending，已有历史不清空。当前不支持退出进程后自动续局。

相机/NPU与Pikafish分时使用，搜索和运动时不保持旧视觉资源。预览是最近一次识别的双画面，固定2560×720，识别ROI/模型/预处理不变。状态文件包含记录时间和帧号，不把状态时间当作硬件心跳。

## 你后续对接的两个点

1. **硬件执行器**：读取工作目录的motion-request.json，按有单位的动作数组执行，完成并停稳后回传同ticket的motion-reply.json；处理取消并去重，真实执行器拒绝simulated=1。也可以在IO_execute内直接调用你的驱动库，主循环不需要改。
2. **实机验证**：填入真实观察位和吃子区，接入SPI-CAN/H桥，完成板端依赖/构建以及连续走棋、释放、吃子、异常恢复验收。

接口细节、启动配置和恢复命令见 [完整对弈与执行器协议](docs/棋局与视觉接口_2026-11-09.md)。[game.example.json](config/game.example.json)中的null是未示教占位，不能直接当现场数据；仿真配置也不能用来控制真实机械臂。主程序不实现SPI-CAN或H桥引脚驱动。

Linux+RKNN构建启用SCARA_WITH_RKNN后生成scara_game，安装的bin/run-game.sh先校验原v3模型SHA256再启动。使用新程序时停用旧YOLO测试服务、保留ISP/RTSP供图。只读预览页是 [game-preview.html](tools/game-preview.html)，复制到工作目录为index.html后，可用普通静态HTTP服务查看；操作在终端完成。

## 新增代码与验证

| 新增文件 | 有效行数 |
| --- | ---: |
| game_main.cpp | 75 |
| game.cpp / game.hpp | 167 |
| game_config.cpp | 67 |
| game_files.cpp / game_files.hpp | 129 |
| game_live_io.cpp | 120 |
| game_sim_io.cpp | 96 |
| Session恢复与状态补充 | 18 |
| 新增合计 | **672** |

这些都计入2472行，没有只展示75行入口而隐藏其调用实现。仿真后端也在总数中；只读HTML、构建/启动脚本、测试和已有Python上位机另列。

当前通过17组CTest、60项原Python检查，其中包括15种真实main可执行程序的场景。另检查旧/未停稳回执、失败及取消、实际IO包装的资源生命周期。视觉六文件哈希不变；16张保存图、152个历史NPU结果、原ONNX CPU的501个结果以及4份固定几何输出保持一致。原64枚棋子核对保持一致，不把重复回放当新样本。

证据见 [本轮自审](docs/evidence/2026-11-14/game-main-20261114/self-review.json)、[行数与源码清单](docs/evidence/2026-11-14/game-main-20261114/final-metrics.json)、[计划与状态](docs/evidence/2026-11-14/game-main-20261114/plan.json)。真实Linux引擎进程、ARM交叉构建、NPU和整机动作未验收；Windows真实IO包装检查使用外部API替身。

源码包已解到空目录重新构建，17组CTest、60项Python检查及保存图像/几何回放再次通过；45个应用源文件、18个CPP编译单元与已验证快照一致。见[交付包验证](docs/evidence/2026-11-14/game-main-20261114/package-verification.json)。

---

下面保留此前1800行固定核心的说明。当前完整程序、接口、总行数与交付入口以上述新增章节为准。

# SCARA 中国象棋机器人

**当前固定棋盘版共1800行自有C++，包含全部CPP、头文件和内部实现。** 本轮以 `6bd2287` 的2975行为起点，实际减少1175行。首次像素/基座标定、YOLO、棋局/UCI、逆解、取放计划和落子确认均保留；不再逐帧自动找棋盘。机械参考和Z高度读取已有实测/示教文件。

## 直接阅读源码

[1800行C++专用阅读稿](output/fixed-minimal-20261113/round-08/cpp-code.txt) · [仅C++源码包](output/fixed-minimal-20261113/round-08/cpp-code.zip) · [全部语言源码](output/fixed-minimal-20261113/round-08/code.txt) · [六个视觉文件完整文本](output/fixed-minimal-20261113/round-08/vision-code.txt) · [源码、构建与测试材料包](output/fixed-minimal-20261113/fixed-application-source.zip)。

| 顺序 | 源码入口 | 处理的数据 | 有效行数，含头文件 |
| --- | --- | --- | ---: |
| 1. 取图、识别、绘框 | [vision_main.cpp](app/vision_main.cpp)、[capture.cpp](vision/capture.cpp)、[yolo_npu.cpp](vision/yolo_npu.cpp)、[yolo.cpp](vision/yolo.cpp) | BGR原图 → 固定ROI的RGB输入 → NPU张量 → 类别/分数/原图检测框 → 预览图 | 408 |
| 2. 首次标定、四角建格 | [geometry_tool.cpp](app/geometry_tool.cpp)、[geometry.cpp](vision/geometry.cpp)、[board.cpp](vision/board.cpp) | 采点与独立检查点 → 像素到基座映射；确认四角 → 90个交点 | 547 |
| 3. 固定交点坐标 | [fixed_board.cpp](vision/fixed_board.cpp) | 初始化时保存90个基座XY；日常根据行列查表 | 48 |
| 4. 棋局和引擎 | [board.cpp](chess/board.cpp)、[pikafish.cpp](chess/pikafish.cpp) | 已确认盘面 → 起始FEN和完整历史 → 引擎合法走法与应手 | 542 |
| 5. 逆解和取放 | [plan.cpp](motion/plan.cpp) | 源/目标XY及示教Z → J2/J3目标角、升降与释放步骤 | 161 |
| 6. 一回合的顺序 | [session.cpp](chess/session.cpp) | 识别 → 格点棋子 → 三帧稳定 → 走法/取放计划 → 新盘面确认 | 94 |

先看 `vision_main.cpp` 的 `source`、`input`、`heads`、`pieces`、`preview`。用户已经认可的直接调用顺序和Camera接收写法保持原样；六个视觉文件的字节哈希与本轮开始时相同。再沿Session中的调用阅读棋局与取放。

## 固定流程怎么运行

1. 首次记录不少于4组像素/基座XY配对，再用独立采点验误。`calibratePlane`计算映射，`savePlane`保存。`loadPlane`统一读取新采点或已保存标定，并复算、验误。
2. 按红方视角给出四个格线交点：红方左下、红方右下、黑方右上、黑方左上，对应a0、i0、i9、a9。`boardFromCorners`生成90点；人工确认朝向后，`FixedBoard_setup`一次缓存XY。
3. 每个新识别帧通过 `Session_observe` 分配棋子。格点层只保留棋子身份和一个有效标志，连续三个不同有效帧一致后才交给棋局。YOLO原有置信度仍用于阈值和绘框。
4. `Pikafish::legalMoves(game, legal)`、`think(game, timeMs, choice)`直接接收已确认的Game，发送其起始FEN和完整走法历史。只有查询成功后才使用输出；引擎返回的走法格式在接收处检查。
5. `Session_plan`准备普通搬子或先清走被吃棋子的计划，`Session_confirm`在动作成功且新盘面符合预期后才提交历史。结果未确认时不自动重发。和棋/认输通过 `Game_finish`提交明确结果。

**固定90点是保存的布局，不代表当前画面已自动定位。** 只移动棋盘时须重新输入并确认四角；相机或基座位置、焦距改变时重新标定映射。重新设置布局后调用 `session.stable.reset()`，再观察新的稳定盘面。坐标查表与帧号无关；识别帧仍必须有编号，重复帧不能推进稳定计数。

布局字段见 [离线格式示例](tests/fixtures/fixed-app/setup.json)。它是测试夹具，不应直接当作现场坐标。相机标识及1280×720尺寸集中在 `vision/geometry.hpp`；当前只接受已独立验误的原始像素标定，含非空镜头畸变参数的配置会被拒绝。

```sh
scara_geometry_tool calibrate measured-pairs.json plane-map.yml
scara_geometry_tool inspect fixed-board.json grid.json
```

两个命令都只处理文件，不连接机械臂。旧双画面入口改用 `--layout`读取固定布局，不再使用 `--orientation-reference`。启动时仍显式保留已验证的识别ROI：

```sh
python temp/yolo_live/dual_preview.py --board <RV1106_IP> --geometry-exe <scara_geometry_tool路径> --layout <现场固定布局.json> --out output/dual-fixed --roi 0 0 1280 720 --inference-roi 300 70 650 600
```

此处是调用格式，本轮没有连接或启动板端服务。预览文字已改为“固定90点已加载”，不能拿它作为自动定位成功的证据。

## 实际删掉了哪些内容

- 自动边框/格线搜索、旋转判向、河界方向模板及对应工具入口。
- 重复运行的机械参考四种方向搜索；当前直接读取已验证的机械参考文件，没有伪造新参考。
- 多相机/多检测坐标格式适配、识别日志导入到标定工具的旁路。
- 每个固定坐标携带的重复帧号等字段、未使用的格点置信度副本和错误列表。
- 把自己生成的FEN再次反向解析的过程、独立视觉转棋局转发库、重复的合法走法列表检查。任意起始盘面仍可用 `Game_reset`设置；不再提供外部FEN文本导入。

代码实际从当前源码删除，旧版保存在Git和已有快照中，没有挪到Python或未计数文件中隐藏。新代码仍保留真实数据入口、SDK/管道失败、资源回收、棋局确认和不可达目标处理；没有删除固件保护。

## 同一口径的行数

NLOC由Lizard统计，排除空行和注释；所有头文件、辅助函数及参数表都计入。页面脚本计入上位机，非脚本HTML另计20行。

| 范围 | 本轮起点 | 当前 |
| --- | ---: | ---: |
| 自有C++应用 | 2975 | **1800** |
| Python上位机、取放、演示与页面脚本 | 2184 | 2138 |
| Python电机标定/调参/升级工具 | 740 | 740 |
| 旧实时视觉服务 | 841 | 835 |
| USB-CAN适配，另列 | 302 | 302 |

1800行不是整仓库所有语言的总数，不含测试、第三方OpenCV/RKNN/Pikafish源码、模型权重或未来SPI-CAN实现。[逐文件计数与哈希](docs/evidence/2026-11-13/fixed-minimal-20261113/final-metrics.json)与[计划/逐轮记录](docs/evidence/2026-11-13/fixed-minimal-20261113/plan.json)可直接核对。此前2975行版本的记录仍在 `upper-review-20261113`。

## 验证范围

本轮完成基线及8轮自查、改写和离线回归，每轮有独立源码快照与本地Git提交。当前通过14组CTest、60项Python检查，包含模拟HTTP控制、重复帧处理、坐标缓存失败恢复、UCI协议、取放顺序和棋局提交。

16张保存图的RGB输入及完整标注PNG逐字节相同；152个历史NPU检测、原v3 ONNX CPU重新推理的501个结果相同。既有两张实拍64枚棋子的类别/位置核对以及保存v3 NPU的32枚核对保持一致。没有新增训练或准确率样本，也没有更换、转换模型。

固定流程的标定文件、90点JSON、叠加图及俯视图共4份输出与精简前逐字节一致。98组逆解结果与旧Python一致；48600组搬子数据摘要保持一致。14400组合成输入比较格点身份和有效性；已删除的格点置信度字段不再计作当前输出，YOLO检测分数仍在视觉回放中比较。第6轮首次检查有一条旧动态帧号测试未同步，失败日志保留；迁移为当前识别帧编号约定后全量通过。

交付包已解到全新目录，重新配置和编译，14组CTest与60项Python检查通过；新编译程序对16张图、4份固定几何输出的回放也一致。37个应用源文件与第8轮快照相同，12个CPP均按C++17编译，六个视觉文件哈希保持本轮起点值。证据见[源码包验证](docs/evidence/2026-11-13/fixed-minimal-20261113/package-verification.json)与[九份快照核验](docs/evidence/2026-11-13/fixed-minimal-20261113/artifact-audit.json)。

这些是离线一致性检查。未进行ARM交叉构建、Linux真实引擎进程、真实NPU连续运行或整机运动验收。当前C++核心产生计划，实际执行入口仍为现有Python上位机/演示；正式SPI-CAN和H桥硬件尚未接入。

## 构建与继续检查

自有C++使用C++17、OpenCV **5.0.0**，需要core、imgproc、imgcodecs、geometry及videoio。相机检查需要FFmpeg解码支持。开发终端中配置并构建：

```sh
cmake -S . -B build/commit-verify -G Ninja -DOpenCV_DIR="<OpenCV5安装目录>/build" -DBUILD_TESTING=ON
cmake --build build/commit-verify
ctest --test-dir build/commit-verify --output-on-failure
python -m unittest discover -s tests -p "test_*.py"
```

本次14组CTest包括RKNN模拟ABI/主入口检查，配置时另传实际SDK头文件目录 `RKNN_INCLUDE_DIR`；未提供时不构建这两组。Windows需要OpenCV运行库可加载。CMake导出的编译数据库已接入 `.vscode/settings.json`。

现有Python检查环境为Python3.13.6、NumPy2.5.4、cv2 4.13.0、ONNX Runtime1.31.1、pyusb1.3.1和libusb-package1.0.30.0。Python旧服务不冒称使用OpenCV5。连续审查使用 `tools/fixed_review.py`，打包使用 `tools/package_upper.py --review fixed-minimal-20261113 --round 8`；它们使用本机SDK与冻结数据，完整ONNX复核另需manifest所列同哈希模型。

RV1106工具链在 `cmake/rv1106.cmake`。原生视觉构建另加 `SCARA_WITH_RKNN=ON`并提供RKNN头文件/库，安装后使用 `tools/run-vision.sh`校验v3模型再启动。此入口只输出识别JPEG，尚未替代原浏览器服务。本轮未部署，也未连接CAN、ADB或GPIO；底层固件、模型、第三方引擎及原现场配置保持不变。

## 现场资料与历史

机械参考与实机过程见 [续做交接](docs/续做交接_2026-11-09.md)、[坐标与下棋记录](docs/项目收敛与下棋闭环_2026-11-09.md)、[当前棋局接口](docs/棋局与视觉接口_2026-11-09.md)。以下带日期段落只记当时事实，其中旧减速比、构建号和未标定状态不覆盖后续确认结果。

## 2026-11-04 实机验收记录

- J1～J4 已依次经 SWD 首装并读回校验，末尾 4 KiB 标定区均保持不变；J2 已使用 25:1 配置重新生成镜像。烧录记录见 `temp/flash-logs/`。
- 开启此前未开启的终端电阻后，四轴在 500k/2M CAN FD/BRS 下通信通过：60 秒 PING 阶段每轴 273/273 应答，共 1092 次；无应答超时、拒绝、重复应答、节点复位或 CAN 错误帧。
- 全程每轴收到 1382 帧 v3 STATUS，故障为 0、编码器有效、功率关闭，构建 ID 与本地对应镜像匹配。电角零点仍为 -1，本次未进行使能、运动或 CAN OTA 验收。
- 原始报文与结果：`temp/comm-logs/20261104-164948-frames.jsonl`、`temp/comm-logs/20261104-164948-summary.json`。

### 现场现象、处理与结论

1. 首次连接 SWD 时，各板复位入口读到无效向量（PC 为 `0xFFFFFFFE`），部分连接报告异常锁死。按用户要求跳过原固件备份，按扇区覆盖程序区；烧录后读回校验通过，应用向量表位于 `0x08004000`，系统毫秒计数正常，未标定使能门控为 0。末尾标定区在同次烧录会话中前后比较一致，没有保存原固件备份。
2. 16:45 首轮 CAN 接收连续 8 秒未收到任何节点 STATUS；用户随后确认当时驱动板未上电。另做一次外部 PING 诊断，无节点应答并出现位填充错误、接收错误被动等错误报告；USB-CAN 的 64 字节 FD/BRS 内部回环通过。内部回环只证明适配器内部收发链路可用，不等于外部总线已正常。
3. 16:48 在用户报告四板已上电后复测，仍无有效 STATUS，出现位填充、格式错误和接收错误被动。用户随后发现终端电阻未开启。没有实际执行单板隔离测试，也没有修改固件或 CAN 位速率。
4. 用户开启终端电阻后，16:49 开始的复测通过：先被动接收 8 秒，再做约 60 秒 PING，含收尾总计约 69.64 秒。各轴 273 次 PING 全部接受，共 1092 次；每轴收到 1382 帧 STATUS，J1 另收到 3457 帧诊断数据。无 CAN 错误帧、异常格式、应答超时、拒绝、重复应答或节点复位。记录中的 USB 接收轮询超时是短时间窗口内暂无数据，不能当作 PING 应答超时。
5. 本次直接证据是：供电并开启终端电阻后，四轴总线在既定 500k/2M 下恢复通信。未实测终端数量、H/L 间电阻或波形，不能把这些列为已验证项目。后续接线时应先确认驱动板供电和两端终端配置。

| 节点 | 构建 ID | PING 应答 | 平均 / 最大主机往返时间 | 最大 STATUS 接收间隔 |
| --- | --- | --- | --- | --- |
| J1 | `0bffcca9` | 273/273 | 14.94 / 40.17 ms | 82.86 ms |
| J2 | `17a105f6` | 273/273 | 12.76 / 35.66 ms | 82.72 ms |
| J3 | `e72bb310` | 273/273 | 10.47 / 35.56 ms | 83.50 ms |
| J4 | `cca804d2` | 273/273 | 8.24 / 33.43 ms | 85.94 ms |

这些时间包含 Windows、USB 和程序轮询开销，不作为 CAN 线上的硬实时延迟指标。全部 STATUS 均为 v3、无故障、编码器有效、功率关闭；配置未标定，与电角零点 -1 的限制一致。通信通过不代表电角标定、运动、OTA 或断电恢复验收完成。

### 本次工具环境与证据

- SWD：CherryUSB CMSIS-DAP，雅特力 OpenOCD Windows V2.1.0，SWD 500 kHz。
- CAN：Candlelight 2.5 / WeActStudioV1，序列号 `37813482333551062`；Python 3.13 独立虚拟环境，离线安装 pyusb 1.3.1、libusb-package 1.0.30.0、importlib_resources 7.1.0。
- J1/J3/J4 使用交接包预编译镜像；J2 按 25:1、零点 -1 使用本机 GCC 13.3.1 重编译。镜像与烧录记录已纳入本项目。
- 失败记录：`temp/comm-logs/20261104-164512-summary.json`、`20261104-adapter-diagnostic.json`、`20261104-164832-summary.json`；成功记录见上文。首次被动接收原始日志为空，是因为没有收到报文。
- 复测工具：`temp/check_can.py --serial 37813482333551062 --seconds 60`；仅发送 PING，先确认全部节点 STATUS 与未使能状态。`ping-sequence.json` 为本机运行状态，不纳入 Git；移到新环境后复测前复位板端，使请求序号与板端状态重新对齐。

## 从这里开始

- 此包包含 C11 固件源码、厂商库、历史镜像、CAN FD 工具和相机脚本。
- [固件参数](firmware/src/config.c)集中维护 `four_axis_config[4]` 和 `six_axis_config[6]`。四轴数组按 J1～J4 存 CAN 节点、角度换算、方向、行程、速度、加减速、三环增益、电流与电压限制等。J2 为 20:1，J3 的 CAD 带轮齿数也在表中。电角标定继续从各板 Flash 读取。六轴数组保存旧项目的电机型号尺寸、标称减速比及已有实测记录，六轴控制仍使用它原来的 StepFOC 应用和协议。
- 旧 Emm 主机与新 v3 CAN FD 要分开验证。首次验证用单块板、支持 FD/BRS 的适配器和 500k/2M。优先选已固定、无垂直负载的板；完成升级和断电恢复验证后再扩到多板。

## CAN FD 按节点更新

1. 首次用 DAPLink 备份各板原有 Flash，尤其末尾 4 KiB 标定区。按目标节点选择 `images/Jn-first-install.hex`，用按扇区擦除方式烧录和读回。勿整片擦除。随后确认 v3 STATUS 能读到，功率未开启。
2. Windows x64 使用 Python 3.11、支持 FD 的 Candlelight 适配器及 WinUSB。关闭占用适配器的上位机。如包内有 `wheels`，可离线安装，否则去掉 `--no-index --find-links wheels`。
3. 先运行离线检查，再执行带 `--write` 的命令。等提示后复位目标板即可，无需接 DAPLink。节点地址必须唯一，镜像节点必须与命令一致；两块板地址相同会同时收到更新。升降机构先承托。

```powershell
py -3.11 -m pip install --no-index --find-links wheels pyusb==1.3.1 libusb-package==1.0.30.0
py -3.11 temp/ota.py update 1 images/J1.ota
py -3.11 temp/ota.py --candlelight update 1 images/J1.ota --write
```

有多个适配器时，在 `update` 前加 `--serial 序列号`。Linux 可用 `python3 -m pip install python-can`，将 SocketCAN 配成 500k/2M FD 后，换用 `python3 temp/ota.py --channel can0 update 1 images/J1.ota --write`。升级完成后从 v3 STATUS 核对节点、版本和功率状态。现有 RV1106 是否具有可用的 CAN FD 主机接口待确认。

引导区起始 `0x08000000`，应用起始 `0x08004000`，应用校验记录 `0x0801E000`，末尾 4 KiB 从 `0x0801F000` 起保留。引导期间关闭功率桥。擦写前先撤销应用启动许可，完整 CRC32 和向量检查通过后最后写入有效标记。断电或写入失败时，引导保留并可按同一命令完整重传。单槽没有旧应用回滚，CRC32 用于完整性校验，实验总线限定为可信设备。

四轴应用编译一次，输出 `joint-app.ota` 和共用构建清单。更新工具按指定节点为这份应用附加目标轴记录。首装 HEX 另带节点对应的引导程序；现有板可沿用原引导执行 OTA。以下命令构建并离线校验，不发送更新。

```powershell
py -3.11 temp/build.py --gcc "C:/你的工具链/bin/arm-none-eabi-gcc.exe" --output output/joint-app
py -3.11 temp/ota.py update 3 output/joint-app/joint-app.ota
py -3.11 temp/ota.py update-group output/joint-app/joint-app.ota --nodes 2 3 4
```

## Z 轴回零取证

J1 实验镜像每 20 ms 发送多圈计数、速度、目标/实际位置、Iq 给定/实测、q 轴 PI 电压请求、母线电压和状态。q 轴 PI 电压请求用于分析，量纲为 mV，实际绕组电压还受矢量和 PWM 限幅影响。CSV 的采样速率适合观察持续堵转，短冲击峰值可能落在两帧之间。

```powershell
py -3.11 temp/ota.py --candlelight capture 1 z-travel.csv --seconds 60
```

采集命令只读。运动控制主机与采集程序不应同时打开同一个 Windows USB 适配器；实测时由接手方把 CSV 记录接入已有 v3 控制会话，或用第二个适配器接入同一总线采集。J1 编译配置必须按丝杆可用行程设置角度上下限，通用的 ±180° 只覆盖一圈。速度 30°/s 指电机 5 rpm，此时反电动势较弱。判读先看编码器速度是否在给定仍移动时下降、跟踪误差是否增长、Iq 是否持续升高，同时用实物位置确认事件。

第一步手动对齐底部安全标记并设零，再向上点动到顶部安全标记。记录两端计数、实测毫米距离，计算每电机转位移 = 实测位移 ÷ ((顶部计数 − 底部计数)绝对值 ÷ 16384)。保存软行程。端点撞击和上电自动回零暂未实现；异物卡住与到端都能造成堵转，不能仅用一次电流或速度突变决定机械零点。后续如要验证接触检测，应先用已知位置和可控接触条件获取对照数据。上电未定位时用人工参考建立零点。

## 历史 MIS5001 相机方案（不用于当前 SC3336）

把两个 `camera-*.sh` 放到 RV1106 同一个持久目录，例如 `/oem/usr/bin/scara-camera`。在板端执行：

```sh
cd /oem/usr/bin/scara-camera
sh camera-keepalive.sh start
tail /tmp/scara-camera-keepalive.log
```

使用项目已有 MIS5001 驱动加载顺序、`/dev/video11`、1280×960 NV12 和板上 IQ 文件。保活模式持续采集并丢弃图像数据，进程退出后隔 3 秒重启。要上电自启，在确认该镜像的启动机制后，将 `sh /oem/usr/bin/scara-camera/camera-keepalive.sh start` 加入其启动脚本。接下棋视觉程序前先用 `sh camera-keepalive.sh stop` 释放设备，由视觉程序自己的连续采集接管。尚未在目标 RV1106 部署。

## 交接顺序

已完成：四轴 SWD 首装、读回与标定区保留核对、四轴 CAN FD 通信验收。后续：单板 CAN FD 完整升级与中断恢复验收 → 逐轴确认电角与机械参数 → J1 从已知标记做短行程验证和取证 → RV1106 相机验证与视觉程序接入。

厂商资料：[AT32M416](https://www.arterychip.com/en/product/AT32M416.jsp) · [TI 步进电机无传感堵转检测](https://www.ti.com/lit/an/slvaei3a/slvaei3a.pdf)

## 2026-11-04 RV1106 刷机与相机接入

- 用户提供的 BoomPi 交接包 SHA-256 与附带校验文件一致，343 个清单文件核对通过。镜像 SHA-256：`bab699164e84db9cbaec85d982a6491c512fdaf50a407cda9231ba474a2e0109`。
- 当前板 ADB 序列号 `abc8f01242c327ec`。补装签名有效的 Rockusb 5.12 驱动后，首次 UF 在下载引导后的 Test Device 阶段失败；重新枚举稳定后第二次成功，日志含 Download Firmware Success 与 Upgrade firmware ok。
- 当前电脑有线 IP 为 `192.168.31.164/24`，板端 eth0 通过现有路由器取得 `192.168.31.233/24`，双向 ping 各 3/3。没有照搬交接包历史 192.168.55.* 地址，也没有修改电脑上网网卡。地址可能在重启后变化。
- 两种传感器均按用户要求探测：SC3336 在 `4-0030` 读取到 ID `OV00cc41` 并绑定；MIS5001 在 `4-0031` 探测失败 `-22` 且未绑定，已卸载未使用模块。当前实际出图的是 SC3336。
- 已验证 H.264 RTSP `rtsp://192.168.31.233/live/0` 为 1280×720、30 fps；一帧实景显示完整中国象棋棋盘，红方在画面右侧。启动日志存在传感器资源/AIQ 告警，不能称无告警运行。尚未做内参、畸变、棋盘坐标或机械臂基座标定。
- 用户选择在当前 Ubuntu 虚拟机预览（当前 NAT IP `192.168.189.129`）。用户已提供 Ubuntu ffplay 成功显示实时画面的截图，并确认对焦完成。脚本和摄像头现象证据在 `temp/rv1106-logs/`。

板子重启后，在板端前台运行（脚本已上传；本次仅语法校验，当前推流是交接包命令逐步启动的会话）：

```sh
sh /userdata/scara-camera/camera-sc3336.sh
```

在 Ubuntu 图形桌面的终端打开预览：

```sh
ffplay -rtsp_transport tcp -fflags nobuffer -flags low_delay -framedrop -window_title SC3336-Camera-Preview -x 960 -y 540 rtsp://192.168.31.233/live/0
```

保持板端推流会话运行；停止推流在其原会话按 Ctrl+C，退出播放器按 q。仅看画面不需要把虚拟机切为桥接；用户截图已确认当前 NAT 虚拟机能够直接播放该 RTSP 地址。相机型号已经确认，不再沿用旧 MIS5001 保活脚本。

## 下一阶段筹备

见 [项目筹备与验收计划](docs/项目筹备与验收计划.md)。使用用户已有 YOLO，不开展训练或微调；模板参数未标定，不用于运动。

## 2026-11-04 YOLO模型包接收

已接收用户现成YOLOv5s/ReLU 14类模型，优先使用包内RV1106 INT8 RKNN，无需训练或暂行重转换。18份交付文件哈希及数据包结构校验通过；当前板端RKNN 1.6.0运行库哈希与历史验证环境相同。当前相机精度和现板推理尚未验收，电机仍禁止使能。详见[模型接收记录](docs/YOLO模型接收记录.md)和`config/model-manifest.json`。

## 2026-11-04 YOLO实时测试

已按用户要求运行现成RKNN：电脑解码/预处理/绘框，真实RV1106 NPU推理，板端提供`http://192.168.31.233:8088/`识别预览。参考test22复测277框全部匹配；当前实物棋盘有明显漏检和错分类，不能用于自动走棋。模型未训练、未重转换，未发送电机命令。临时测试使用主机OpenCV4.13，不等于板端OpenCV5正式方案已完成。详情见[实时测试说明](docs/YOLO实时测试.md)。


## 2026-11-06 视觉坐标对接代码

已新增 C++17 / OpenCV 5.0.0 的纯几何模块与离线工具：平面像素到基座XY标定、独立点误差验证、棋盘自身格线定位、人工四点与印刷特征判向、90交点、YOLO归格和目标XY输出。使用说明见 [视觉坐标对接](docs/视觉坐标对接.md)。

构建入口为根目录 CMakeLists.txt，程序 scara_geometry_tool；配置模板位于 config/vision/。代码不链接电机/CAN，不会自动运动。真实标定点尚未采集，示例模板保持无效；棋盘移动只更新网格映射，不重新覆盖基座平面映射。


## 2026-11-06 电磁铁 MOS 吸放验证

LSD-P25/20 的24V/80mA版本，经双N-MOS模块控制。最新网表确认 U7物理11脚为GPIO1_B0（当前Linux GPIO40），U7物理9脚为GND。更换模块后，用户确认高电平吸住、低电平关闭正常；结束保持低输出，无自动重开，未移动机械臂。旧模块疑似故障，未拆检定因；新模块准确关闭电流、温升及释放延时仍未测量。详细接线、过程、寄存器和原始证据见[电磁铁MOS接线与验收](docs/电磁铁MOS接线_2026-11-06.md)。

归档范围、远程分支和提交前检查见[2026-11-06归档说明](docs/归档说明_2026-11-06.md)。

## 2026-11-06 四轴手动上位机

浏览器点动界面位于 `manual_control/`，Windows 本机地址 `http://127.0.0.1:8090/`：J1～J4 正反转、按住运行、松开减速保持、独立 0.1～30°/s 速度、全轴停止、实时反馈与日志。启动方法和当前电角配置阻塞见[四轴手动上位机](docs/四轴手动上位机.md)。真实四轴当前通信正常，但仍回报电角配置未就绪，页面未发送真实运动命令。

当晚后续：**J1 已通过 CAN 更新为 `d26b98f4`，完成基本电角标定并保存（raw_zero=10088、direction=+1），当前位置设为机械0°，故障0、功率关闭。** 手动页面已恢复供用户点动；J2～J4 未更新。实测过程与边界见 [J1标定记录](docs/J1电角标定接入_2026-11-06.md)。
