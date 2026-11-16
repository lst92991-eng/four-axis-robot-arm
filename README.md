# SCARA 中国象棋机器人：直接C++执行接口

跨设备续做请从[重构交接](docs/重构交接_2026-11-16.md)开始，里面有 Git 分支、历史材料下载和回归验收约定。

2026-11-15按用户选择，机械执行改为直接函数调用。当前全部自有C++为 **2679有效行**，包括头文件、内部实现、仿真后端和待接入硬件接口；比上一版2758行减少79行。动作JSON请求、回执、取消文件和轮询已从源码删除，预览图片与棋局状态仍保留供显示。

## 先看这些代码

| 阅读顺序 | 文件 | 处理的数据 |
| --- | --- | --- |
| 1. 主循环与双方回合 | [game_main.cpp](app/game_main.cpp)、[game.cpp](app/game.cpp) | 初始化、玩家确认、引擎应手、执行和落子复核 |
| 2. 图像识别 | [vision_main.cpp](app/vision_main.cpp)、[capture.cpp](vision/capture.cpp)、[yolo_npu.cpp](vision/yolo_npu.cpp)、[yolo.cpp](vision/yolo.cpp) | 相机原图→固定ROI输入→NPU张量→类别/分数/原图位置→预览 |
| 3. 坐标与稳定观察 | [geometry.cpp](vision/geometry.cpp)、[fixed_board.cpp](vision/fixed_board.cpp)、[session.cpp](chess/session.cpp) | 首次标定、四角90点、棋子归格、连续三帧和基座XY |
| 4. 棋局与引擎 | [board.cpp](chess/board.cpp)、[pikafish.cpp](chess/pikafish.cpp) | 已确认盘面、起始FEN、完整历史、合法走法和应手 |
| 5. 逆解和取放 | [plan.cpp](motion/plan.cpp) | 已保存机械参考、示教Z、源/目标XY→按顺序执行的PickStep数组 |
| 6. 硬件接入位置 | [execute.hpp](motion/execute.hpp)、[execute.cpp](motion/execute.cpp) | 直接执行动作数组，以及停止；SPI-CAN/H桥接这里 |

[全部C++阅读稿](output/game-direct-20261115/round-01/cpp-code.txt) · [仅C++源码包](output/game-direct-20261115/round-01/cpp-code.zip) · [源码与验证包](output/game-direct-20261115/game-application-source.zip)。保持先声明、说明单位、逐字段赋值和顺序调用的手写风格。六个已认可视觉文件的字节保持不变。

## 后续硬件只接两个函数

```cpp
void Motion_execute(const std::vector<PickStep> &steps,
                    const volatile std::sig_atomic_t &running,
                    bool &success);
bool Motion_stop();
```

`Motion_execute`同步执行整个数组，全部完成并停稳后才写 `success=true`。`running`变0时停止，不继续下一步；失败不会自动重发。`Motion_stop`在普通函数上下文调用，真实确认停稳才返回true。信号回调只关运行标志和相机，不进行阻塞硬件I/O。

**当前 execute.cpp 尚未实现真实SPI-CAN/H桥，两个接口明确返回失败。** 这是待硬件接入的位置，不能直接用它跑实机下棋。补齐实现后，主循环、棋局、坐标和取放计划不需要改变。`Robot_execute`先关闭相机/NPU，再调用它；实际执行成功之后，主程序仍要看到新三帧盘面与预期一致才提交走法。详细约定见[对弈与执行接口](docs/棋局与视觉接口_2026-11-09.md)。

## 现在可以离线运行完整对局

```powershell
.\build\commit-verify\scara_game_sim.exe tests/fixtures/game-app/simulation.json
```

同一main和业务流程链接仿真IO，场景包含两轮走棋和一次机器人吃子，完整历史为 `a3a4 a6a5 a4a5 a9a5`。状态保存在 `output/game-main-simulation/status.json`。仿真把动作调用参数单向打印为SIM_CALL/SIM_STEP/SIM_RETURN，供离线核对，不生成或读取动作往返文件，也不加载真实相机/NPU/引擎/电机。

普通搬子、吃子、双方轮次、任意已确认初始盘面、无合法走法、认输/和棋、重复局面人工裁定和失败恢复仍保留。玩家走完并移开手后回车；retry只观察，accept仍需盘面吻合，restored只在完整复位后解除待确认状态。当前不支持重启进程自动续局。

实机需要在 `config/game.example.json` 填入实际观察位、吃子区和已有标定/示教文件。模板null和仿真夹具不能当真实坐标。浏览器只读显示最新 `preview.jpg` 与 `status.json`，思考和运动期间相机暂停，终端负责交互。

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

## 本轮验证与行数

17组CTest、60项Python检查通过。原文件通道检查已退役，改为验证直接接口的执行结果、取消、停止失败和未接入实现；真实IO包装仍验证相机/NPU/引擎的资源顺序。旧版和新版15种完整对局场景的棋局状态和动作内容一致，明确排除已删除的ticket和文件传输格式，不把它们算作需要保持的行为。

16张保存图的输入/绘制像素、152个历史NPU检测、原v3 ONNX CPU的501个检测以及4份固定几何输出保持一致。没有新增准确率样本；原64枚实拍棋子的读图核对不变。当前未验证ARM交叉构建、Linux真实引擎进程、RV1106 NPU连续运行或机械臂实机动作。

全部自有C++按Lizard计2679有效行，空行和注释不计，所有CPP/HPP和辅助函数均计入。其他语言另列：原Python上位机/演示2138行、标定调参工具740行、旧实时视觉835行、USB-CAN适配302行；测试、第三方库、模型及未来SPI-CAN实现另计。没有将旧执行协议挪到未计数文件里。

证据：[逐项自审](docs/evidence/2026-11-15/game-direct-20261115/self-review.json) · [前后程序对照](docs/evidence/2026-11-15/game-direct-20261115/program-equivalence.json) · [源码行数与哈希](docs/evidence/2026-11-15/game-direct-20261115/final-metrics.json)。此前的精简和手写风格修改保留在Git及对应output/审查快照中。

## 构建与继续检查

C++17、OpenCV **5.0.0**，需要core、imgproc、imgcodecs、geometry及videoio；相机检查需要FFmpeg解码支持。

```sh
cmake -S . -B build/commit-verify -G Ninja -DOpenCV_DIR="<OpenCV5安装目录>/build" -DBUILD_TESTING=ON
cmake --build build/commit-verify
ctest --test-dir build/commit-verify --output-on-failure
python -m unittest discover -s tests -p "test_*.py"
```

本机17组CTest包含Windows POSIX替身及RKNN模拟ABI/主入口检查，配置时传 `RKNN_INCLUDE_DIR`；未提供RKNN头文件时少两组。Windows需要OpenCV运行库可加载，编译数据库已接入VS Code。已有Python离线/旧服务cv2为4.13.0，不冒称使用OpenCV5。

完整审查使用 `tools/game_review.py`，本轮系列参数为 `--series game-direct-20261115`；打包使用 `tools/package_upper.py --review game-direct-20261115 --round 1`。回放需本机SDK、冻结夹具及原哈希的v3 ONNX。

RV1106工具链为 `cmake/rv1106.cmake`。Linux原生构建启用 `SCARA_WITH_RKNN=ON` 并提供RKNN头文件/库后生成scara_game；硬件执行仍需补齐execute.cpp。`tools/run-game.sh`校验原v3模型再启动。单独视觉入口仍使用 `tools/run-vision.sh`，启动新程序前停用重复占用NPU的旧测试服务。本轮没有部署、连接CAN、ADB或GPIO。

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
