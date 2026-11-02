# SCARA / 四轴机械臂软件平台调研证据

调研日期：2026-10-08（Asia/Shanghai）。范围是软件复用和教学阅读成本；用户硬件已经确定，本文不建议更换硬件。所有程序只下载、读取和统计，未安装项目依赖、未执行项目程序、未编译、未连接机械臂。功能介绍与源码观察不等于实机验证。

## 结论

最值得借鉴的形态是“小型桌面程序 + 串口协议 + 简单 MCU 固件”。它能把棋盘、规则、AI 与运动执行分开，通常不需要 ROS、MoveIt、Web 后端或者通用插件框架。Marcel 的 C# + Arduino 实例，以及 HowToMechatronics 的 Processing + Arduino 示例，比通用 CNC 固件更接近教学规模；但短代码存在原点搜索阻塞、异常处理不完整等真实缺口，不能把行数最少直接当作最适用。

必须分开看：①学生编写与阅读的应用代码；②自动生成的 UI 与复制进来的组件；③运动固件继承的完整框架；④下载依赖；⑤配置、CAD、文档与媒体。只列 GitHub 仓库大小或主语言百分比不能回答教学工作量。

## 八个相关设计 / 平台

| 项目 | 已核实的技术栈与控制方式 | 对本项目可借鉴部分 | 限制与教学成本 | 许可证证据 |
|---|---|---|---|---|
| [HowToMechatronics SCARA](https://howtomechatronics.com/projects/scara-robot-how-to-build-your-own-arduino-based-robot/) | Arduino Uno / C++、CNC Shield、4 个步进轴加独立夹爪舵机；Processing GUI，串口，正逆运动学和示教回放 | 坐标到关节、机械参数、手动控制、抓取动作顺序；四运动轴与夹爪分别控制的结构接近目标 | 是抓放/示教项目，尚需棋盘识别、规则、AI、回合管理；源下载页存在锁定下载界面，本次未获得完整原包，故不报源码总行数 | 教程公开、提供源下载入口，但本次未核实完整源码包许可证，不能直接标成 MIT/GPL |
| [Marcel3245/SCARA-robot-V2](https://github.com/Marcel3245/SCARA-robot-V2) | C# WinForms / .NET Framework 4.7.2，System.IO.Ports；Arduino C++ + **MobaTools**；4 步进轴、独立舵机、4 限位 | 串口命令、坐标控制、回零、路径录制、关节状态显示 | 小而直观；仓库没有 .csproj/.sln；回零存在无超时 while 循环；不能仅凭 README “可靠”认定适合课堂批量复现 | 仓库未发现 LICENSE 或源文件许可证声明，待确认 |
| [techyg/Pybot](https://github.com/techyg/Pybot)（JJrobots 固件存档） | Arduino C++ / SAMD21 DEVIA M0，ESP8266 UDP、USB 串口、自写定时器步进控制；3 步进轴 + 手腕舵机 + 夹爪舵机；VL53L0X | 紧凑的逆解、轨迹、遥测分工；板端实现可作为参考 | 原项目资源维护已中断，存档作者明确没有原 Python App；底层定时器代码绑定 SAMD21，不是可直接移植到任意 MCU 的通用驱动 | 主 .ino 文件头明确写 GPL v2；不能把全部历史资料视为同一许可 |
| [ayuugoyal/scara](https://github.com/ayuugoyal/scara) | Arduino Mega + AccelStepper；Next.js 14、React 18、TypeScript、Tailwind/shadcn；Express 4 + serialport 13；HTTP→串口 | 多层控制面板怎样接串口，以及 UI 组件膨胀对体量的影响 | 实际固件只有单电机相对步进命令，未发现 SCARA 逆解和回零；README 串口格式与实际程序不同；不应当作完整机械臂规划系统 | backend/package.json 声明 ISC；前端和固件未见统一 LICENSE，不能把后端声明外推全仓 |
| [jared-hughes/grbl-scara](https://github.com/jared-hughes/grbl-scara) | 基于 Grbl 0.9 的 AVR C 固件，G-code/串口；平面双连杆 + 可选线性 Z；把直线拆段并逆解 | 成熟 CNC 解析、缓冲和加减速的思路，SCARA 线段细分与可达范围参数 | 2023-01-27 已归档；继承 GRBL 的体量不等于 SCARA 修改量；没有证据表明它已经提供独立第四手腕运动轴 | COPYING：GPL-3.0-or-later |
| [bdring/FluidNC 的 Scara 分支](https://github.com/bdring/FluidNC/tree/Scara) | ESP32 / C++、PlatformIO、YAML 配置，G-code/串口/Wi-Fi/WebUI；SingleArmScara.cpp 包含两连杆和第四坐标的末端朝向补偿 | 可复用 G-code 控制接口、分段、软限位和关节/末端朝向关系 | **不是 main 默认具有的能力**；Scara 分支回零代码仍有人工处理注释及占位风格日志，需要专门验证；架构采用继承/工厂/配置框架，阅读成本显著高于小固件 | LICENSE：GPL-3.0-or-later；个别文件另有声明 |
| [MarlinFirmware/Marlin 2.1.x](https://github.com/MarlinFirmware/Marlin/tree/2.1.x) | C++ / Arduino / PlatformIO，众多 MCU/HAL；G-code；MORGAN_SCARA、MP_SCARA 运动学 | 正逆运动学、回零/限位与运动规划的工程参考；已有控制板兼容时可评估作为底层固件 | 原生定位为 3D 打印固件，大量配置、宏、板型、UI、加热功能；不能把 SCARA 打印机支持等价为完整四轴棋子抓放能力，手腕/夹爪仍须核对 | GPL-3.0-or-later |
| [ROBOTIS OpenMANIPULATOR SCARA Friends](https://emanual.robotis.com/docs/en/platform/openmanipulator_x/friends/#openmanipulator-scara) | OpenCR + Arduino C++、Processing、DYNAMIXEL SDK，XM430 总线电机；官方示例组织 | 结构化运动学、轨迹和总线执行器接口的教学参考 | 官方表格的 **4 DOF = 3 DOF + 1 End-Effector**，不等于用户标准四运动轴；与当前未知控制器兼容性未定。SCARA Friends 的 Arduino/Processing 路线不应和整个 ROS 2 主仓库混为一谈 | [OpenCR](https://github.com/ROBOTIS-GIT/OpenCR/blob/master/LICENSE)、[Processing](https://github.com/ROBOTIS-GIT/open_manipulator_processing/blob/master/LICENSE) 为 Apache-2.0；第三方依赖分别核查 |

补充排除：普通 [gnea/grbl](https://github.com/gnea/grbl) 的 Arduino Uno / ATmega328P 三轴 CNC 固件不能直接当作四轴 SCARA 固件。搜索命中 [derFrickler/FluidNC_scara](https://github.com/derFrickler/FluidNC_scara) 时，其默认 main（09fd5726…）也没有 SCARA 运动学文件；仓库名称不是功能证据。两者仍可作为通信/规划参考，但不单独算已可用的四轴 SCARA 方案。

## 实测体量与口径

全部用 `git ls-files` 枚举当前快照，按文件原文统计物理行、去掉纯空白行后的非空行。**非空行仍包含注释、宏和花括号，不叫有效 LOC。** 未执行 cloc。固定源扩展名为 `.c .h .cpp .hpp .cc .ino .cs .ts .tsx .js .jsx .mjs .css .py .pde`；不计 JSON/YAML/XML 配置、锁文件、Markdown、图片、视频、字体、CAD、二进制及 Git 元数据。排除这些不代表它们在运行时不需要；本表是源码阅读规模，不是完整交付包统计。

第三方依赖未下载安装，Arduino 核心、AccelStepper、MobaTools、SAMD 平台、Node npm 依赖、.NET 运行库、PlatformIO 外部库等均不在表中。FluidNC 与 Marlin 的所列源码目录可能内嵌上游库，所以只称“源码子树”，不冒充纯自有业务代码。

| 快照与范围 | 文件数 | 物理行 | 非空行（含注释） | 边界说明 |
|---|---:|---:|---:|---|
| Marcel：`main/*.ino` | 7 | 428 | 364 | 应用固件，不含 MobaTools |
| Marcel：手写 `*.cs` | 4 | 1,518 | 1,309 | 桌面应用，不含 Designer、resx |
| Marcel：`*.Designer.cs` | 3 | 1,581 | 1,561 | 设计器生成 UI，独立列出 |
| Pybot：根目录 3 个 ino + Configuration.h | 4 | 2,106 | 1,889 | 固件一份，包含自写驱动/网络/运动学 |
| Pybot：`PyBotArm_v1/` 复制件 | 4 | 2,106 | 1,889 | 四文件逐个 SHA-256 相同，不应重复算学习量 |
| ayuugoyal：Arduino 固件 | 1 | 194 | 162 | 仅单电机命令和电磁铁控制 |
| ayuugoyal：`backend/src` | 7 | 361 | 308 | Express/串口应用 |
| ayuugoyal：前端应用/辅助/CSS | 10 | 734 | 642 | 不含组件库目录和构建代码 |
| ayuugoyal：`frontend/components/ui` | 46 | 4,702 | 4,222 | shadcn 风格组件脚手架，未证明每个都在业务入口使用 |
| ayuugoyal：构建配置代码 | 3 | 107 | 104 | mjs/ts 配置；其他 JSON/YAML 未计 |
| grbl-scara：`grbl` 固件（不含 examples） | 51 | 8,992 | 7,669 | **含继承的整个 GRBL 主体**，不是 SCARA 净增 |
| grbl-scara：主机发送示例 | 2 | 204 | 171 | Python 示例，不计入固件 |
| grbl-scara：上传入口示例 | 1 | 29 | 22 | Arduino 上传壳，不计入固件 |
| FluidNC `Scara`：`FluidNC/src` | 294 | 38,023 | 32,224 | 固件源码子树，不含外部平台依赖和测试 |
| 其中 `SingleArmScara.cpp/.h` | 2 | 336 | 260 | 运动学部分；**不能把 336 行当可独立运行固件** |
| FluidNC `main`：`FluidNC/src` | 349 | 49,914 | 42,854 | 对照：本次 SHA 中不含 Scara 类 |
| Marlin `2.1.x`：`Marlin/src` | 2,082 | 434,311 | 381,011 | 含板型引脚、HAL、字体/显示资源与库，不能视为课程代码 |
| 其中 `module/scara.cpp/.h` | 2 | 360 | 285 | 运动学模块，需要其余框架支撑 |

可直接比较的“小型应用”合计：Marcel 手写主机 + 固件 = **1,946 物理行 / 1,673 非空行**；ayuugoyal 固件 + 后端 + 前端应用/辅助 = **1,289 / 1,112**，但额外还有 **4,702 / 4,222** 的 UI 组件；Pybot 固件一份 = **2,106 / 1,889**，而上位机代码并未包含。三者覆盖的实际功能不同，不能按行数排序得出质量名次。

## 固定的测量提交

| 仓库/分支 | 提交 SHA | 提交日期 |
|---|---|---|
| ayuugoyal/scara | `03107d1672bbbf4ee171f7af15af8e24ad5b0342` | 2025-04-28 |
| Marcel3245/SCARA-robot-V2 | `c738f807c93e473611eb657c4ab56d2119687391` | 2025-08-02 |
| techyg/Pybot | `93027a9b25e9c328932457bbcc26b0427b93a90a` | 2024-08-04 |
| jared-hughes/grbl-scara | `bc03358810650e20241de7c707b6ca225295bc74` | 2021-03-28 |
| bdring/FluidNC main | `4a18985947e36e253d7234d2e940af2bec1c1d49` | 2026-10-07 |
| bdring/FluidNC Scara | `28a9b815d4affbd45e40224d7f192bf605cd7090` | 2025-12-25 |
| MarlinFirmware/Marlin 2.1.x | `47d0617f9cc6e20e146c2579b858403465450aed` | 2025-03-01 |
| derFrickler/FluidNC_scara main（排除核验） | `09fd57261d65e2bf7d60834c7f3af60a2e0c569f` | 2024-01-24 |

测量清单及每文件 SHA-256：`scara-platforms-metrics.json`。测量脚本：`measure_scara_platforms.py`。源快照位于本机临时目录 `C:/Users/lst/AppData/Local/Temp/scara-research-platforms/`，以 `git -c core.autocrlf=false clone --depth 1` 获取；Marlin 显式取 `2.1.x`，FluidNC Scara 显式取 `Scara`。这只是调研样本版本记录，不是用户项目冻结。

## 重要源码证据

1. [Marcel main.ino](https://github.com/Marcel3245/SCARA-robot-V2/blob/c738f807c93e473611eb657c4ab56d2119687391/main/main.ino)：使用 MobaTools，不是 README 泛称 Arduino 就可推断的 AccelStepper。`homecoming.ino` 中等待限位的循环没有看到超时路径；适合借鉴模块职责，不宜未经修正直接交付学生。
2. [ayuugoyal 固件](https://github.com/ayuugoyal/scara/blob/03107d1672bbbf4ee171f7af15af8e24ad5b0342/arduino_scara/arduino_scara.ino)：实际电机格式为 `motor_index,steps,direction,speed`，README 描述 `M{index} position speed`；固件有 `1000000 / speed` 且未见除数为零检查。README 所说“安全特性/位置跟踪”不足以证明完整系统。
3. [Pybot 主文件](https://github.com/techyg/Pybot/blob/93027a9b25e9c328932457bbcc26b0427b93a90a/PyBotArm_v1.ino)：文件头给出 SAMD21/DEVIA M0、ESP8266、GPL v2；`Robot.ino` 直接使用 TC3/TC5/TCC2 定时器，`Network.ino` 解析 USB/UDP 消息。
4. [grbl-scara README](https://github.com/jared-hughes/grbl-scara/blob/bc03358810650e20241de7c707b6ca225295bc74/README.md)：双连杆、可选 Z、Grbl 0.9、默认 2 mm 分段都有明确说明。
5. [FluidNC Scara 运动学](https://github.com/bdring/FluidNC/blob/28a9b815d4affbd45e40224d7f192bf605cd7090/FluidNC/src/Kinematics/SingleArmScara.cpp)：两连杆长度参数、`elbow_motor` 区分关节处电机与皮带耦合、第四坐标朝向补偿；`kinematics_homing()` 仍含人工关闭电机注释，初始化假设手臂完全伸展。需要用用户装配与控制器核对，不能宣称开箱可用。
6. [Marlin SCARA 运动学](https://github.com/MarlinFirmware/Marlin/blob/47d0617f9cc6e20e146c2579b858403465450aed/Marlin/src/module/scara.cpp)：Morgan/MP 两种运动学分支及正逆解明确存在，但只是通用打印固件中的模块。

## 对课程软件设计的启示（分析判断）

- 先固定上层下棋软件与下层运动控制的边界；棋类改变不应要求改脉冲发生器、回零和串口驱动。推荐只约定坐标移动、取放和状态查询的少量动作。
- 不把运动学参数写进五子棋规则。五子棋的交叉点、象棋的交叉点、国际象棋的格子中心，应先转成棋盘坐标，再经一次统一标定转换到机器人坐标。
- 兼容棋类的最小价值点是棋盘尺寸/落子点、局面和合法落子规则、决策函数、动作序列；未来移动和吃子可以把同样的取放动作组合起来。当前不必提前做插件注册、抽象工厂或全棋类继承树。
- 教学代码可以只有数千行，但短固件对异常路径缺少处理的情况很多。急停、软限位、不可达坐标、回零超时、动作完成确认和通信失联不能为了缩行省略。
- 现阶段只选择软件结构，不决定具体平台和轴映射。SolidWorks 装配压缩包已收到；完成装配几何解析并核对现有控制板/电机驱动接口后，再确定角度定义、传动耦合、机械零位、Z 方向和末端工具控制。

## 调研范围限制

这是公开互联网可找到的代表性平台与近似设计清单，覆盖紧凑固件、桌面主机、Web 主机、CNC 框架、商用执行器教学例程；不是对“全部类似开源项目”的穷尽证明。HowToMechatronics 和 ROBOTIS SCARA 未做全仓代码计量；所有实例都未以用户硬件做编译或实机测试。许可栏只记录已看到的声明，未进行法律结论判断。
