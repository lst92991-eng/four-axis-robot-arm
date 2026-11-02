# 原始SolidWorks机械臂中国象棋仿真

2026-10-10。当前工作目录：Ubuntu RV 虚拟机 `/home/st/scara-sim`。
本机 VSCode 已通过 `ubuntu-codex` 连接此目录。Windows 的 `simulation/` 保留源码镜像；完整依赖、贴图和证据随运行包交付。

## 先运行

### Ubuntu桌面交互（当前推荐入口）

在RV虚拟机的桌面终端执行：

主机CUDA服务与SSH转发当前已启动。Windows重启后先在本机PowerShell执行一次：

```powershell
& C:/Users/lst/Desktop/scara/simulation/start-gpu.ps1
```

再在Ubuntu执行下面的一条sh。VMware客体没有NVIDIA直通，不能仅给OpenCV设置CUDA标志就当作GPU推理。

```sh
cd /home/st/scara-sim
./run-interactive.sh black
```

参数表示机械臂颜色：`black`时你执红先走，`red`时机械臂执红先走。用户已明确本次使用**仿真相机**，不采集实物MIS5001。
当前界面是两个独立可拖动窗口：SolidWorks三维整机和YOLO棋盘。直接在YOLO棋盘中拖动你方棋子并释放，三维机械臂会执行应手；窗口标题栏也可拖动，分别调整大小和位置。原图与纯矫正帧继续保存在output中，界面只显示这两个窗口。
你方走棋后，Pikafish根据完整已确认历史搜索应手，四个虚拟CAN节点控制机械臂吸放/吃子，停车复拍后才提交这一手。交互模式不自动替你走棋，也不使用批量演示中的固定棋谱。

- 空格：暂停/继续虚拟运动，图像预览继续。
- `N`：暂停时推进一个20ms运动采样。
- `Esc`或关闭窗口：退出仿真。
- 三维窗口右键旋转、滚轮缩放、`R`恢复视角。

三维本体来自用户原始195＋275mm装配：153个组件、289,016个三角面。每个组件的网格经过原装配Transform2变换，并与CAD包络核对；分为固定、Z滑台、肩、肘、丝杆五个显示组，随四节点虚拟状态运动。相机也从同一三维场景离屏成像后进入NV12/OpenCV/YOLO链路，不使用绘图标签代替识别。
Z滑台沿模型的实际行程显示，肩/肘轴对应现有FK/IK。原CAD没有完整末端电磁铁，显示用20mm长、12mm直径简化件；棋盘厚14mm也是假设，板面选择为CAD Z=1000mm。这些尺寸等待实物核对。原CAD被抑制的传动子装配保持抑制；没有擅自修改或保存原CAD。
这是带实际CAD外形的运动学/伺服误差仿真，不是完整刚体动力学、接触力或电机转矩求解。紧固件显示分组不作为新连续碰撞认证，丝杆转动采用逻辑Z和假设导程。

三维绘制、OpenGL相机取图和交互留在主线程；图像预处理和GPU请求最多只有一个后台作业，不排队。落子严格复拍前先等待后台任务完成。YOLO的模型、输入张量、NMS均在宿主RTX5060的CUDA上，未连接GPU会停止，不回退CPU。预览使用独立噪声，不消耗运动传感器随机序列。吸放100ms和停稳200ms仍是有意停顿。
YOLO画面上方显示CUDA后端和GPU处理时间；`output/gpu.json`保存验证信息，`output/graphics.json`保存三维渲染器。当前实测SVGA3D；检测到llvmpipe/softpipe会拒绝启动。GPU分配器预算上限为本机显存的25%，这是分配上限，不是独占预留2GB。OpenCV几何、小规模滤波、棋规和Pikafish仍在CPU正常执行。
使用已登录的Ubuntu桌面会话启动；普通SSH未设置显示环境时不会自动得到窗口。三维使用固定raylib5.5.0/OpenGL，YOLO窗口使用GTK3，图像算法严格使用OpenCV5.0.0。图形显示需要桌面或测试显示服务器，批量回归也需要OpenGL上下文。

2026-10-12修复引擎等待中的界面刷新：先读取已经缓冲的完整行，等待新数据时才处理取消；引擎期间GUI刷新额外间隔50ms。避免逐行重绘耗尽合法走法/搜索超时预算，没有放宽原超时或更改棋规。修复只作用于此桌面副本。

### 批量回归入口

在 VSCode 的远程终端执行：

```sh
cd /home/st/scara-sim
./run.sh red
./run.sh black
```

颜色表示**机械臂控制哪一方**；红方总是先走。也可修改 `config.json` 的 `robot_side`，不传参数时使用它。
默认演示20个半回合，前两步特意安排双方吃子，后续由真实 Pikafish 搜索。一个半回合是一方走一步。`max_plies` 是演示长度上限，达到它不会冒充胜负。

画面在 `output/camera.jpg`、`output/rectified.jpg`、`output/vision.jpg`、`output/scene.jpg`，棋局在 `output/status.json`，终局、退出或错误在 `output/result.json`。
`view.html`只看三维和YOLO的静态记录；实际可旋转三维、拖棋和移动窗口的入口是Ubuntu桌面的两个窗口。

## 从运行包重新构建

Ubuntu运行包面向22.04 x86_64，包含OpenCV5.0.0、raylib5.5、引擎、CAD网格和贴图。需要已有CMake、g++、OpenGL/X11开发环境；GPU依赖与权重在另一个主机配套包中，C++不再加载ONNX。

```sh
tar -xzf scara-cad-sim-2026-10-10.tar.gz
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
`yolo_gpu.cpp`是短小的GPU客户端，OpenCV准备RGB640输入；`gpu_server.py`在WSL的CUDA环境运行同一权重。原CPU适配移到本机`output/simulation-review/yolo_cpu_reference.cpp`仅保留基准证据，不参与构建。
`x42s.cpp/.hpp`负责共享X协议、四轴梯形规划和真实SocketCAN驱动；`motion_filter.cpp/.hpp`负责可接真实反馈的KF/EKF。详细设计、厂家代码分析、测试与实机边界见项目主文档`docs/X42S驱动分析与实现报告.md`，Ubuntu包内也带`docs/`副本。
`chess/` 直接复用现有棋局和 Pikafish 接口，未再实现一套象棋规则。

同一C++入口继续维护视觉、运动与棋局顺序。`cad_view.cpp/.hpp`只负责实际CAD网格、离屏相机、三维显示和鼠标射线；两个消费者共享这一套场景。`export_cad.py`是Windows离线只读导出工具，Linux运行不需要SolidWorks。GPU服务只推理图像，不拥有棋局或运动状态；它属于桌面配套，后续RV1106直接使用RKNN/NPU，不部署这个Python服务。

CAD导出需要本机已安装的SolidWorks COM技能及NumPy，原装配只读打开后执行`python export_cad.py`。资产的来源文件哈希、每个组件的组别、包络误差和网格哈希在`assets/arm/manifest.json`中。图形依赖固定[raylib 5.5源码](https://github.com/raysan5/raylib/tree/5.5)，遵循其自带LICENSE；运行包带头文件和静态库，未把训练框架或图形示例复制进业务代码。

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
工程图预览和STEP也未给出导程。2026-10-10用户打开SolidWorks后已恢复COM；再次只读检查候选装配内丝杆/螺母的特征、方程和全部自定义属性，仍未发现导程，既有未保存标志保持原样。**不能由直径16mm、长度500mm或联轴器比例推导导程。**
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

状态是肩角、肘角、两轴角速度。测量包括电机侧编码器角度与**合成末端XY位置传感器**；非线性测量是FK。共享滤波模块只有编码器时明确是KF；只有新鲜、通过残差检查的独立XY进入时才是EKF。实机不可把FK(编码器)伪装为独立XY。
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

1. `cadCamera()`/`formatCamera()`的模拟输入换为MIPI/ISP NV12帧；保留OpenCV定位、矫正和识别结果核对。
2. `yolo_gpu.cpp`及桌面加载接口换现有RKNN适配和板端头文件，加载已验证RKNN；不把WSL服务部署到RV1106。板端还需沿用已有的小内存生命周期安排，NPU释放后再启动引擎。
3. 使用已经写出的`X42Can`真实收发，核对FWType、内核、单位、地址、零位/限位、心跳、使能和停止后才连接真实运动。当前sh不会打开真实CAN口。
4. `magnet()`换实际电磁铁IO和抓取确认，EKF的合成XY测量换真实传感观测；核对减速比方向、导程、板到基座标定和实际净空。

这几处是明确的替换位置，不是实机已验证的即插即用承诺。后续仍按项目阶段效果确认、教学精简、回归、审核及授权提交推进。

## 主机GPU环境复现

本机复用WSL2的`Ubuntu-24.04-D`和`/home/lst/.cache/scara-xiangqi-train`，Python3.12、Torch2.7.1+cu128、TorchVision0.22.1+cu128。主机配套包`scara-sim-gpu-runtime-2026-10-10.tar.gz`保留已应用兼容补丁的固定YOLO源码、许可证和原始portable.pt；不包含巨大的venv或重新训练。

在同款WSL的干净环境准备时：

```sh
mkdir -p ~/.cache/scara-xiangqi-train
tar -xzf scara-sim-gpu-runtime-2026-10-10.tar.gz -C ~/.cache/scara-xiangqi-train
python3.12 -m venv ~/.cache/scara-xiangqi-train/venv
~/.cache/scara-xiangqi-train/venv/bin/pip install \
  torch==2.7.1 torchvision==0.22.1 --index-url https://download.pytorch.org/whl/cu128
~/.cache/scara-xiangqi-train/venv/bin/pip install numpy==2.2.6 opencv-python==5.0.0.93 \
  Pillow==11.1.0 PyYAML==6.0.2 pandas==2.2.3 matplotlib==3.10.1 seaborn==0.13.2 \
  scipy==1.15.2 setuptools==69.5.1 requests==2.32.3 tqdm==4.67.1 psutil==6.1.1
```

这些是已验证组合的安装说明，本轮没有删除既有环境另做全量重装。随后在Windows执行`start-gpu.ps1`；如果WSL名称、Linux用户名或SSH别名不同，传入`-Distro`、`-GpuRoot`、`-Vm`即可。脚本只启动回环服务和到已有虚拟机的SSH转发，不安装开机任务或开放防火墙。

GPU与原CPU同一矫正帧的28个棋子身份/交点一致；连续20次GPU适配往返平均约47ms（含传图），GPU本体约6～7ms，原CPU约99ms。首帧建CUDA上下文及空闲唤醒明显更慢，不能用稳态时间承诺每一帧。实际记录见`output/simulation-review/gpu-benchmark.json`。
