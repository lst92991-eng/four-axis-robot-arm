# Z轴500 RPM实测与项目最小驱动

日期：2026-10-13。本轮只提取已有记录、编写C++模块和离线验证，没有新增实机运动；高频转速日志任务仍暂停。

## 1. 可以确认的结论

**当前Z轴在本次装配与供电条件下，上升、下降都实测达到过500 RPM。500 RPM不是已经测出的极限，也不是持续运行额定值。**

从3个原始日志提取14段F6速度测试，共1144个运动期间采样点：

| 项目 | 结果及解释 |
|---|---|
| 目标 | 电机侧500 RPM，约8.333转/秒 |
| 达到过500 RPM | 11段；另3段较早停止，未等到达速 |
| 结束方式 | 13段有停止确认，1段故障终止；停止确认不等于整段性能达标 |
| 最高采样值 | 523 RPM，不能当作额定或极限转速 |
| 首次采样达到500 RPM | 各段约6.35～7.01秒 |
| 加速度 | Emm acc=1：每12.75ms改变1RPM，约78.43RPM/s；0→500RPM理论约6.375秒 |
| 当前采样间隔 | 中位约0.109秒，约9Hz；不是高频采集，可能漏掉更短的波动 |
| 位置反馈 | 电机侧编码器累计位置，65536计数/圈；不等于滑台毫米或独立负载测量 |

时间来自主机日志，约9Hz采样；“首次达到”指首次采样观察到，不是驱动内部精确达速时刻。

加速后统计统一取“起动至少6.375秒、第一条FE停止指令之前”的采样，未对尾段做静默剔除。第1、2、5～10、14段在这个窗口内全部位于475～525 RPM，各段中位数497～500.5 RPM。第12、13段存在异常尾部，单独保留：

- 第12段：上升，后期降到17 RPM、相电流2694mA，然后有停止确认。
- 第13段：下降，后期0 RPM、相电流3050mA、状态`0x05`（使能位为1、堵转位为1），软件故障停止。
- 不能仅凭这些日志认定触碰了哪一个机械限位，也不能把异常归因于电机额定能力。具体负载质量、丝杆导程、当时机械位置仍未形成完整测量记录。

对走棋时间有用的换算：若实测导程为`p mm/转`，500 RPM对应`8.333×p mm/s`。当前p未知，不填毫米速度。短距离抬升可能还在加速段，不能用“距离÷500RPM对应线速度”代替完整动作计时。

## 2. 已核对的连接与协议

| 项目 | 当前有效事实 |
|---|---|
| 轴 | Joint1/M1/CAN节点1＝Z升降，CAD电机L60-1＋联轴器＋丝杆 |
| 方向 | 用户实测A/方向0/CW为上升，B/方向1/CCW为下降；只适用于当前接线与配置 |
| 硬件 | 实物丝印X42S_V2.0B，不能外推另外三轴 |
| 驱动模式 | Emm闭环，选项回读`1A 00 06 6B`；当前整数RPM输入 |
| CAN | 500kbit/s、经典扩展帧，节点1单包ID=`0x100`，固定校验`6B` |
| 恒速 | F6按下时只发送一次，正常运行只查询反馈，不周期重发运动目标 |
| 停止 | FE立即停止，确认ACK及连续零转速；不失能、不清故障 |
| 失联保护 | 临时500ms，不存Flash；低速软件静默测试通过，非独立硬件急停 |
| 其余三轴 | CAD对应肩L60-2、肘L60-3、腕L40-1；实物CAN地址仍未验证 |

当前CAN单包数据（已经去掉串口命令中的地址字节）：

```text
上升500RPM： ID=0x100，扩展帧，F6 00 01 F4 01 00 6B
下降500RPM： ID=0x100，扩展帧，F6 01 01 F4 01 00 6B
立即停止：  ID=0x100，扩展帧，FE 98 00 6B
读转速：    ID=0x100，扩展帧，35 6B
临时500ms： ID=0x100，扩展帧，68 38 00 00 00 01 F4 6B
```

速度应答`35 sign hi lo 6B`按整数RPM解析；位置应答`36 sign b3 b2 b1 b0 6B`按`计数×360/65536`解析。不能套用X固件的0.1°单位。

精确`0x20000004＋8个00`兼容为旧candleLight空状态通知，不算有效电机应答；其他CAN错误、格式错误和超时仍报错。断电曾造成ACK错误；未屏蔽ACK、Bus-off或协议错误。

## 3. 项目代码与教学顺序

用户明确要求源文件与头文件形式的项目代码。现已将前一版`emm_can.*`迁移为下列模块，旧实现仅保存在本次修改前快照中，不保留两套有效驱动。

| 文件 | 职责 | 有效行数 |
|---|---|---:|
| [can.hpp](../robot/can.hpp) / [can.cpp](../robot/can.cpp) | `robot::Can`：打开已有SocketCAN接口，发送一次、限时接收、通信错误与资源释放 | 15＋64 |
| [motor.hpp](../robot/motor.hpp) / [motor.cpp](../robot/motor.cpp) | `robot::Motor`：Emm报文、应答匹配、反馈解码、准备/启动/读取/停止 | 31＋120 |
| [motor_test.cpp](../app/motor_test.cpp) | 阶段1入口：参数→CAN→Motor→准备→启动一次→反馈→停止 | 59 |

四个驱动文件共**230有效行、261物理行**，比前一版192有效行增加38行，用于明确CAN/电机边界、节点寻址和停止失败锁定。有效行不计空行及独立注释。入口另计，没有接口基类、工厂、后台线程或多轴调度框架。

教学按`can.hpp → can.cpp → motor.hpp → motor.cpp → app/motor_test.cpp`阅读。CAN层不解释电机指令；Motor层不直接调用socket/ioctl。`Can`须先构造、后析构，同一总线上所有调用串行执行；其他程序不能同时控制同一电机。本阶段只验过实物节点1，构造支持地址并不代表其余三轴已验收。

```cpp
#include "robot/motor.hpp"

robot::Can can("can0");     // 打开接口，不发送电机命令。
robot::Motor motor(can, 1); // 指定节点，构造不发帧。
motor.prepare();           // 接入检查，临时500ms失联停机，不存Flash。
// 按下时只调用一次：motor.start(目标有符号RPM);
// 运行中约每100ms：auto state = motor.read();
// 松开或结束时：motor.stop();
```

| Motor接口 | 行为 |
|---|---|
| `prepare()` | 核对Emm、校准、使能、静止和24V台架电压；设置并回读临时失联停机 |
| `start(signedRpm)` | 非零、幅值≤3000；发送一次F6，acc=1；运行中再次start拒绝；3000只是输入范围 |
| `read()` | 电机侧累计角度deg、整数RPM、相电流mA及状态；有效查询也维持心跳 |
| `stop()` | FE停止并确认连续3次零速；不失能、不清硬件故障；未确认停止则抛异常并锁定读取 |

当前节点1正值上升、负值下降；换线、配置或传动后复核。仍保留100ms应答期限、34秒台架上限、2800mA台架阈值及驱动故障检查，不恢复已取消的目标转速偏差保护。34秒由调用read时检查，应用必须持续读取；调用停顿依赖500ms驱动失联停机。应答缺失不盲重发F6；已经发出即按可能运动处理并请求停止。异常后不继续喂旧运动心跳；调用方显示错误，显式核对停止后才重新prepare。析构只能尽力停止，不能替代stop的成功结果。

Can只兼容精确旧candleLight空状态通知，不让NOTICE或无关帧延长请求期限。协议没有事务序号，本模块要求单控制者、关闭非请求式高速上报；不提供多客户端应答分发。

本阶段实现恒速运动及反馈，未增加位置定位、多轴同步、回零、校准、使能、EKF或毫米换算，也未把真实运动接入原棋局/仿真主循环。Windows测试面板保留独立入口。

## 4. 构建、调用与验证边界

**500RPM实物证据仍来自Windows/Python面板。以下C++使用Linux SocketCAN，已离线验证和交叉编译，尚未接RV1106真实CAN。**

Linux/WSL下在项目根目录构建，只需C++17、CMake和Linux头文件，无需OpenCV/RKNN：

```sh
cmake -S robot -B output/motor-build
cmake --build output/motor-build -j2
./output/motor-build/scara_motor_test
```

产物为`libscara_motor.a`和`scara_motor_test`。无参数只显示用法，不打开CAN。根项目也通过`add_subdirectory(robot)`复用同一库与入口，根项目的其他模块仍要求原OpenCV依赖。

在真实Linux接口、500kbit/s及节点1已经核对之后，调用格式为：

```sh
# 静止接入检查与反馈；prepare可能写临时心跳参数，不发运动命令。
./output/motor-build/scara_motor_test can0

# 只有显式给出RPM与时长才运动；以下参数由操作者按实际行程填入。
./output/motor-build/scara_motor_test can0 SIGNED_RPM SECONDS
```

`SIGNED_RPM`是有符号整数，`SECONDS`为1～34整数。程序只发一次启动命令，约每100ms读取反馈，到时或Ctrl+C/SIGTERM停止并确认；默认不演示500RPM。程序不配置CAN速率、不自动使能。操作系统强制终止或总线断开时，软件不能保证收到停止确认。

RV1106继续使用项目固定工具链，从项目根目录执行：

```sh
cmake -S robot -B output/motor-arm \
  -DCMAKE_TOOLCHAIN_FILE=../cmake/rv1106.cmake \
  -DSCARA_TOOLCHAIN_ROOT=/home/lst/.cache/scara-vision/luckfox-toolchain/tools/linux/toolchain/arm-rockchip830-linux-uclibcgnueabihf
cmake --build output/motor-arm -j2
```

本次通过WSL GCC13及固定GCC8.3 Cortex-A7/hard-float/uClibc的CMake构建，均加`-Werror`；ARM产物为32位EABI5可执行程序及静态库。板端历史记录为CONFIG_CAN未启用，必须另行核对当前内核CAN_RAW、CAN_DEV、CAN_GS_USB、USB Host及实际can0收发；交叉编译不等于这些前置条件完成。

测试沿用现有`tests/main.cpp`和[协议测试](../tests/motor_tests.cpp)。不设`SCARA_TEST_CAN_INTERFACE`时跳过；离线复现只在新建隔离网络命名空间中使用vcan0：

```sh
mkdir -p output/motor-check
printf 'void testMotor();\nint main(){testMotor();}\n' > output/motor-check/main.cpp
g++ -std=c++17 -O2 -Wall -Wextra -Wpedantic -Werror -pthread -I. \
  robot/can.cpp robot/motor.cpp tests/motor_tests.cpp output/motor-check/main.cpp \
  -o output/motor-check/test
unshare -Urn sh -c 'ip link add vcan0 type vcan && ip link set vcan0 up && SCARA_TEST_CAN_INTERFACE=vcan0 ./output/motor-check/test'
```

已通过：标准/扩展帧收发及超时、非法帧拒绝、虚拟节点寻址、构造不发帧、模式与心跳检查、不存Flash、正负反馈、F6仅一次、FE确认、假节点心跳停止、缺ACK/缺反馈/堵转/格式及真实CAN错误后的停止与锁定，停止ACK缺失后拒绝继续读取。未运行全项目ctest或真实电机。

本次修改前快照、构建产物、文件哈希和回归记录位于`output/motor-can-refactor-2026-10-13/`。此前`output/emm-can-check/`属于合并版本的历史验证。

## 5. 提取证据

目录：[500 RPM提取结果](../output/hardware-validation/500rpm-extract-2026-10-13/)。

- `measurements.json`：统计定义、每段结果、原始文件SHA-256。
- `segments.csv`：14段逐段摘要，保留异常尾段及短动作。
- `samples.csv`：1144个原有采样点，可直接画波形；未新增高频采集。
- `sources/`：本次读取的3份原始日志快照，避免原面板继续追加后混淆统计范围。

未提交或推送；原Python面板、旧X固件仿真驱动及历史证据均保留。

## 6. 四个聊天同步所得的接口约定

经用户授权，已向01需求调研与技术选型、03算法、04中国象棋技术栈调研、05绘制机械臂工作区投影父聊天同步，并取得四方只读回复。原文保存在本次证据目录`thread-sync.json`。

- 01：保持`robot/`与短阶段入口；迁移现有实现，先单轴，不提前接棋局或加入通用框架。依旧等用户审核及提交授权。
- 03：算法`Joints`顺序为肩、肘、Z、腕，而节点顺序为Z、肩、肘、腕，后续逐项映射。算法使用关节侧rad/mm及rad/s、mm/s，电机反馈不能直接代入。F6恒速不能直接执行位置轨迹。历史Vacuum命名不代表当前使用气路。
- 04：沿用固定Cortex-A7/uClibc工具链；真实RV1106 USB Host/CAN/gs_usb尚未通过，历史CONFIG_CAN未启用，需要接板时复核。
- 05：现用仍是195＋275mm旧机；Z直联1∶1、导程未知；CAD 0～327.5mm配合范围不是实机软限位。50∶1属于肩轴，不能给Z反馈除以50。

上述信息已体现在代码职责和本说明中。下一项硬件验证是RV1106真实CAN通信与C++单轴功能，不将当前离线通过扩大为实机阶段完成。
