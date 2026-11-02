# 旧版 SCARA 中国象棋仿真

2026-10-10。当前工作目录：Ubuntu RV 虚拟机 `/home/st/scara-sim`。
本机 VSCode 已通过 `ubuntu-codex` 连接此目录。Windows 的 `simulation/` 保留源码镜像；完整依赖、贴图和证据随运行包交付。

## 先运行

在 VSCode 的远程终端执行：

```sh
cd /home/st/scara-sim
./run.sh red
./run.sh black
```

颜色表示**机械臂控制哪一方**；红方总是先走。也可修改 `config.json` 的 `robot_side`，不传参数时使用它。
默认演示20个半回合，前两步特意安排双方吃子，后续由真实 Pikafish 搜索。一个半回合是一方走一步。`max_plies` 是演示长度上限，达到它不会冒充胜负。

画面在 `output/camera.jpg`、`output/vision.jpg`、`output/scene.jpg`，棋局在 `output/status.json`，终局或错误在 `output/result.json`。
用 `view.html` 查看原图、识别框和俯视图；图像每一步更新，俯视图是几何示意，不是CAD动态渲染。

## 从运行包重新构建

运行包面向 Ubuntu 22.04 x86_64，包含已验证的 OpenCV 5.0.0、模型、引擎和贴图。
需要系统已有 CMake、g++；准备贴图的 Python 工具另外需要 NumPy、Pillow，直接运行不需要 Python。

```sh
tar -xzf scara-sim-2026-10-10.tar.gz
cd scara-sim
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release \
  -DOpenCV_DIR="$PWD/deps-opencv-5.0.0/lib/cmake/opencv5"
cmake --build build -j4
./run.sh red
```

`run.sh` 只进入项目目录、设置所带库的搜索路径并启动程序。CMake用 `EXACT` 拒绝其他 OpenCV 版本。
运行包不是用户指定的冻结基准，也没有执行 Git 提交、推送或旧工程清理。

## 顺着代码阅读

`sim.cpp` 按参数和坐标、成像识别、CAN虚拟节点、运动估计/取放、主对局顺序展开。
`yolo_cpu.cpp` 是桌面 ONNX 适配，与现有板端 `vision/detector.cpp` 共用 `detector.hpp` 的接口约定。
`chess/` 直接复用现有棋局和 Pikafish 接口，未再实现一套象棋规则。

本次主文件约721行（包含成像、虚拟对象和控制），CPU适配83行，独立调试68行；计划主文件约700行，额外部分保留了净空/碰撞/超时和错误结果输出。源码按普通语句展开，没有用压行或宏隐藏步骤。以上不计复用的棋局/引擎代码、依赖库、模型和素材；它是完整桌面仿真体量，不是最终板端体量。已删除不需要的BFS绕路和自动生成配置步骤。

实际链路是：渲染像素→模拟NV12→OpenCV5转BGR→检测4个ArUco→透视矫正→RGB→同一真实YOLOv5权重→映射90交点→Pikafish合法走法/搜索→虚拟CAN搬子→复拍确认。
检测器不会读取渲染器里的棋子标签。取放会保留真实模拟位置误差，不把棋子强行吸附到理想交点。

## 一份配置

| 参数 | 本次值与来源 |
|---|---|
| 结构 | 旧版195＋275mm，由既有CAD轴线测量得到；不使用275＋275mm候选新机 |
| 节点 | 1=Z、2=肩、3=肘、4=腕；节点1=Z由用户指定，其余为仿真映射，实机核对地址 |
| 减速 | 肩50由用户确认；肘3为GT20/GT60设计意图的假设；腕1为直接输出假设 |
| Z | 电机与丝杆直接联轴，转数比1:1；`screw_lead_mm=4`是待替换的导程假设 |
| 棋具 | 25mm磁性棋子是用户采购规格；6mm高度、30mm格距、400×430mm外板均为仿真假设 |
| 棋盘 | 基座坐标中心(-170,200)mm，角度130°；不是已确定的实物摆位 |
| 相机 | MIS5001 B名义VFOV56.5°，1280×960，棋盘中心正上方450mm；无畸变针孔假设 |
| 抬升 | 从棋子顶面接触位置向上10mm；6mm高棋子在本模型中余4mm净空，未验证实物 |
| 动力参数 | 600RPM、1200RPM/s、15ms伺服滞后均为演示假设，无负载转矩/结构柔性辨识 |

相机在上述机位完整覆盖外板和四角标记，25mm棋子约50个原始像素。相机水平/垂直名义视场和4:3输出不完全匹配，本仿真只用VFOV与方形像素建立一致内参。
早先626mm假设机位下棋子约36像素，已观察到误分类，不再作为本次默认值。450mm也不能直接转成打印支架高度；实际有效视场、畸变、焦点和文字清晰度仍须实拍核对。

旧机CAD碰撞证据是4°采样，程序逐1°检查规划线、逐20ms检查实际虚拟关节。
此摆位90个交点、停车点、15个收纳点之间5,565对直线均通过采样约束检查，因此不保留额外绕路规划器；不通过时直接停止。
原CAD采样只覆盖当时的Z/R和已建模实体，不能据此称连续运动、传动总成及整个实体环境已通过碰撞认证。

丝杆证据见原工程 `research/mechanism/assembly-evidence.json` 的 `findings.Z`：模型是无螺旋/螺距特征的简化体。
工程图预览和STEP也未给出导程，本次COM重读遇到已有SolidWorks实例不可连接。**不能由直径16mm、长度500mm或联轴器比例推导导程。**
当前假设4mm/转下，抬升10mm对应2.5转；更换导程后由同一换算自动更新。

## 最少的CAN协议

只实现手册V1.0.5第42～45、55、60、72页所需子集：扩展ID `(node<<8)|packet`，固定校验6B。

| 功能 | 内容 |
|---|---|
| `FD` | X梯形绝对位置、加减速度、同步缓存；15字节载荷拆两帧，每帧重复功能码 |
| `FF 66 6B` | 广播同步启动 |
| `36 6B` | 读取电机位置；X固件返回带符号的0.1°，不能按Emm的65536/圈解码 |
| `FD 02 6B` / `FD 9F 6B` | 接收成功/动作到位，按四轴分别跟踪；等待有超时 |

代码里没有USB、真实SocketCAN发送或刷固件操作。虚拟节点按实际报文字段运行梯形轨迹，包含CAN分包和四节点反馈，但未建模总线仲裁、带宽、掉帧或厂家内部控制器。

## EKF和调试隔离

状态是肩角、肘角、两轴角速度。测量包括电机侧编码器角度与**合成末端XY位置传感器**；非线性测量是FK，雅可比写在更新步骤中。
滞后、背隙play算子、零偏、编码器量化/噪声分别在虚拟对象中出现。EKF结果用于停稳后的有限次数位置修正，不直接使用真值控制。
仅靠减速器输入侧编码器不能观测输出背隙；实机若没有独立末端位置测量，不能原样宣称实现了这里的运动EKF，也不能声称估计误差下降就是机械振动消失。

`debug.cpp` 只观察CAN、运动状态、阶段与放子误差，不参与控制或随机采样。
关闭它：

```sh
cmake -S . -B build-nodebug -DSIM_DEBUG=OFF \
  -DOpenCV_DIR="$PWD/deps-opencv-5.0.0/lib/cmake/opencv5"
cmake --build build-nodebug -j4
LD_LIBRARY_PATH="$PWD/deps-opencv-5.0.0/lib" ./build-nodebug/scara_sim red
```

清洗时先关闭并移走 `debug.cpp`，再删除 `debug.hpp` 的观察回调调用和CMake调试选项；核心没有另一套调试专用控制路径。正常控制台只给最终摘要或一条错误。

## 完整对局复现与验证范围

`assets/replay.json` 保存真实Pikafish搜索产生的161步合法走法及红胜结果，是回归用棋谱，不是识别标签。
重放时仍通过引擎逐步验证合法性、真实YOLO逐帧识别及实际虚拟取放；没有用棋谱代替识别。默认配置只固定前两步，仍会调用引擎搜索。

```sh
cp config.json output/config-before-replay.json
python3 - <<'PY'
import json
with open('config.json') as f: c=json.load(f)
with open('assets/replay.json') as f: c['scripted_moves']=json.load(f)['history'].split()
c['max_plies']=400
with open('config.json','w') as f: json.dump(c,f,ensure_ascii=False,indent=2)
PY
./run.sh red
./run.sh black
cp output/config-before-replay.json config.json
```

已核对执红、执黑的完整161步重放，带噪20步、无额外噪声20步、关闭调试，以及不合法颜色、覆盖不足、抬升净空不足的拒绝路径；数字见运行包 `output/checks.json` 和 `output/final-*/`。
带噪20步中，原始XY估计RMSE约0.447mm，EKF约0.256mm，14次放子最大误差约0.483mm。这是指定噪声和合成测量下的模拟结果。
完整161步中，执红95次取放、最大误差1.583mm；执黑90次取放、最大误差1.466mm。较长对局会积累抓取偏心，20步指标不能代替整局指标。

YOLO权重沿用已训练的14类YOLOv5s/ReLU，本次没有重新训练。贴图只从原训练集裁剪；所选贴图3、合成画面和重放棋谱不是独立测试集，更不等于用户实拍通过。
已发现一处训练标签将实际“相”标为“帅”，仅在贴图准备中排除并记录，未静默改动训练包或模型。
其他贴图、较远机位及部分自由对弈盘面已出现身份误分类，程序会在棋局核对失败时停止；当前证据不保证任意棋具、任意盘面均成功。正式相机/选定棋具数据仍需单独验收。

## 接到实物时替换什么

1. `renderCamera()`的模拟输入换为MIPI/ISP NV12帧；保留OpenCV定位、矫正和识别结果核对。
2. `yolo_cpu.cpp`换现有RKNN适配及ARM构建，模型路径换成已验证RKNN；RV1106不能运行这里的x86引擎或ONNX桌面适配。板端还需沿用已有的小内存生命周期安排，NPU释放后再启动引擎。
3. 虚拟`send()`/`receive()`换实际CAN收发；实测FWType、单位、地址、归零/限位、使能和停止后才能发运动指令。真实IO未接入本程序。
4. `magnet()`换实际电磁铁IO和抓取确认，EKF的合成XY测量换真实传感观测；核对减速比方向、导程、板到基座标定和实际净空。

这几处是明确的替换位置，不是实机已验证的即插即用承诺。后续仍按项目阶段效果确认、教学精简、回归、审核及授权提交推进。
