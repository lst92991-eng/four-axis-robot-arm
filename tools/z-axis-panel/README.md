# Z轴恒速性能测试面板

## 双击启动

1. 在项目根目录双击 **启动Z轴上位机.bat**，不需要打开终端或管理员权限。
2. 启动器检查本机Python 3.11 x64、Tk和USB依赖；缺失时自动安装到本用户LocalAppData。Release离线包自带安装器和wheel；源码启动缺失时从python.org与PyPI下载，Python安装器执行前校验官方SHA-256。不需要管理员安装Python，不修改PATH或驱动固件。
3. 窗口启动后检查连接，并临时设置/回读500ms驱动失联停机（不存Flash），成功显示“已连接节点1”。默认目标50RPM，处于手动待机，不自动使能或运动。
4. 输入目标RPM→静止时点“应用转速”→按住上升A/下降B。松开、失焦、空格、Esc或红色CAN停止按钮请求停止。
5. 锁定后可点击 **清除面板锁定 / 重新连接**。按钮会等待旧任务完全退出，再重连、请求停止并核对静止状态、驱动配置；不清硬件故障、不自动使能、不恢复原运动。真实故障仍存在时会再次锁定并保留原因。
6. 正常退出用窗口右上角X；程序先完成停止处理。再次双击BAT时，如果原窗口还在，会显示原窗口而不创建第二个控制进程；不会修改原窗口的目标RPM或运动状态。

可选：在项目根目录的终端执行 `启动Z轴上位机.bat 500`，以500RPM目标值打开，仍保持手动待机。执行 `启动Z轴上位机.bat -SetupOnly` 只安装环境；`启动Z轴上位机.bat -CheckOnly` 只检查环境、不安装。两种模式都不打开USB或窗口、不发电机命令。PowerShell中调用BAT时，文件名前加 `.\`。

启动失败时黑色窗口保留错误信息。详细错误文件为项目内 `output/hardware-validation/z-panel-stderr.log`；界面已打开但连接失败时看界面提示。当前只接本次已确认的USB-CAN与节点1，不要同时运行其他电机控制软件。

## 通信故障日志

点击 **打开故障日志** 查看 `output/hardware-validation/z-can-diagnostics.log`。记录北京时间（含+08:00）、单调时钟、错误帧ID/原始字节、解析类别、发生阶段、最近发送的命令和原始JSONL路径。USB初始化失败、应答超时、停止确认失败及重连操作也会记录；初始化失败时不会因Drive对象未创建完成而丢掉故障原因。

2026-10-13实测：500kbit/s节点1连续80次只读查询80次收到对应应答、0超时，同时出现46条`0x20000004 + 8个00`。旧candleLight v1.0的`can_parse_error_status()`默认生成这个基础结构，即使没有填入具体错误位，也会返回该帧；Linux gs_usb对没有warning/passive位的控制器帧按active状态处理。因此只有此精确全零格式记为NOTICE，不再单独触发锁定；仍要求收到正确的新鲜电机应答。其他错误（包括以前的`0x20000024` ACK错误、`0x2000000C`协议错误）、超时及Bus-off照常停止/锁定，不能笼统屏蔽CAN错误。

这是依据实际报文与上游实现做的兼容处理，未把USB报告的固件数字2当作已证明对应某个发布tag，也未刷适配器固件。依据：[candleLight v1.0](https://github.com/candle-usb/candleLight_fw/blob/v1.0/src/can.c)、[Linux gs_usb](https://github.com/torvalds/linux/blob/master/drivers/net/can/usb/gs_usb.c)。实测证据为`output/hardware-validation/can-link-diagnostic-steady.json`。本次通信故障日志不属于仍暂停的高频转速采集任务。

BAT通过自身目录定位程序，项目路径含空格也支持。优先复用 `%LOCALAPPDATA%\Programs\Python\Python311\python.exe`；缺失时自动安装到 `%LOCALAPPDATA%\scara-z-axis-panel\Python311`，依赖在 `%LOCALAPPDATA%\scara-z-axis-panel\python-libs`。不要只复制BAT，需同时复制 `tools/z-axis-panel/`，或使用Release小包。

## 新电脑首次接入

支持Windows 10/11 x64，不支持ARM64。下载 [J1-Windows-x64-Demo.zip](https://github.com/lst92991-eng/four-axis-robot-arm/releases/latest/download/J1-Windows-x64-Demo.zip) 后完整解压，双击根目录BAT即可。随包提供 `SHA256SUMS.txt` 与依赖许可证；本包只准备J1演示所需环境，其他项目模块按各自说明配置。

接入的是原来已经验证的CANable与节点1：VID:PID `1D50:606F`、适配器序列号 `004A00404346570B20363833`、candleLight/gs_usb，500kbit/s，节点1为X42S_V2.0B的Emm闭环固件。序列号绑定防止操作错设备，不自动适配另一只CANable或其他驱动固件。USB应直接交给Windows，避免被虚拟机接管。

在设备管理器查看通讯接口（MI_00），正常驱动为 **WinUSB**。若未绑定，在官方 [Zadig](https://zadig.akeo.ie/) 中启用Options → List All Devices，选择这只 `candleLight USB to CAN adapter` 的通讯接口，核对VID/PID，再安装WinUSB；这一步Windows可能要求管理员确认。已有WinUSB时跳过，不要选择其他USB设备，不用刷CANable固件。[gs_usb上游](https://github.com/jxltom/gs_usb)说明Windows使用libusb/WinUSB接口。

仅USB识别成功不代表CAN总线正常。保持原已验证的接线、终端电阻与500kbit/s设置；适配器USB不为电机提供24V电源。先确认面板正确连接，再由操作人按住按钮。更换电脑不需要重新校准编码器。A上升/B下降映射仅适用于当前接线和MotDir配置；导程、真实Z软限位仍未标定，不能把RPM换算成已知毫米速度。面板CAN停止不能代替独立硬件急停。

Python安装选项依据[官方Windows安装文档](https://docs.python.org/3.11/using/windows.html#installing-without-ui)；固定安装器为[Python 3.11.9](https://www.python.org/downloads/release/python-3119/)，SHA-256取自同版本官方Sigstore bundle。

## 当前功能

运行 `run.ps1`。这是SCARA主线的临时Windows实机诊断入口，面向已校准、已使能的节点1，500 kbit/s、Emm闭环、1.8°电机、16细分。原C++ SocketCAN驱动只实现X固件，本面板单独使用已实测的Emm报文；未改原工程入口。

2026-10-13更新：用户已确认A上升、B下降。手动启动默认50RPM，可在静止状态输入1～3000整数RPM并点“应用转速”，应用本身不使能或启动。范围来自当前手册Emm位置命令（第57页），不是机械臂已测带载上限；不采用6000RPM空载宣传值。界面区分已应用目标、实际速度及本次采样峰值。50RPM曾出现55RPM瞬时反馈并触发原超速保护，尚未完成持续稳定验收。

**当前手动模式已经改为F6持续速度模式，覆盖早期分段位置点动实现。** 按下只发送一次F6（方向、目标RPM、acc=1、立即执行），不周期重发运动目标。运行期间仅周期查询反馈，兼作驱动心跳。松开/急停/失焦/UI租约过期或34秒到时发送FE停止并核对连续零转速。保持闭环，未切换CR_OPEN。acc=1对应约78.43RPM/s，启动仍有一次加速；高目标是否达到必须看实测。

F6没有原先1圈的位置终点。每次连接与启动前必须确认驱动失联停机为500ms，否则拒绝F6。设置命令存储位为0，退出后保持这项临时保护，掉电后的值重新核对。停止发送有效命令后由驱动处理失联急停，不能把它说成独立硬件急停。启动前遇到旧错误帧只记录并清空，等待静默后必须通过新请求校验；新请求遇到ACK/协议错误依然停止。配置的多包应答允许按包号乱序缓存，缺包仍超时。

2026-10-13低速实机核验：F6目标5RPM，测到5～6RPM；不再发送命令约0.797秒后连续三次反馈0RPM；第二次短运行后FE停止也确认成功。参数回读500ms，不存Flash；记录在`output/hardware-validation/z-f6-watchdog-smoke.json`。这是软件静默的失联测试，不是拔线/断电或全速全行程验收。

用户随后要求转速仅作展示：手动模式已删除“半秒80%进度”及“目标RPM+3”自动停机判定，也不按全程理想轨迹停机。实测转速和采样峰值仍记录并显示，不据此宣称达标。驱动故障/失能、2800mA相电流、方向/指令范围、通信、UI租约、34秒上限及松开/急停停止保留；没有修改驱动自身堵转、过热或过流配置。自动测试仍保留轨迹跟随和稳定到位检查。

- 按住上升A/下降B运行，松开、离开按钮、窗口失焦、空格、Esc或停止按钮请求停止。运动中禁止应用新转速；设置由同一个设备线程重新校验静止状态后生效。
- 手动使用F6持续速度模式，必须由停止命令/驱动心跳保护终止；导程和真实Z软限位仍未标定，输入RPM不能当作已知毫米速度。自动往返入口保留FD每段1圈，未冒充恒速测试。
- 工作线程检查200ms的界面租约；USB/反馈错误、失能、堵转或相电流超过本轮2800mA阈值时锁定。该阈值低于读回的3000mA驱动配置，不是电机额定电流或承载极限；不修改驱动本身的堵转保护。
- 停止要求ACK及连续三次零转速反馈。USB断开时不能保证FE停止指令到达；F6依赖已核对的500ms驱动失联停机。按钮标明CAN停止，不能替代独立硬件急停。尚无实测导程、Z软限位和独立急停认证。
- 只临时设置并回读500ms失联停机，不存Flash；不自动使能、清硬件故障、回零、校准或刷固件。保持使能不等于已经验证断电防坠。
- 配置与本次实测不一致时拒绝运行；故障后需处理现场原因并点击重连，不能按住自动恢复。

依赖固定版本安装在本机LocalAppData；报文及操作证据写入 `output/hardware-validation/z-panel-*.jsonl`。只有此面板的互斥锁不能阻止其他CAN上位机，请不要同时运行其他电机控制程序。

离线验证：`python -m unittest discover -s tools/z-axis-panel -p test_panel.py`。它不打开USB，不发实机命令。

用户另行明确授权自动测试后，可用`panel.py --auto-test`启动：窗口置顶，连接检查通过后倒计时5秒，按A/B顺序各1圈，共3轮。每段必须位置到达、零转速与停止确认，停稳1秒再换向；任何取消、超时、反馈异常都终止剩余序列，不自动重试。自动模式由可响应的UI续租，关闭或UI卡死会撤销运动；窗口失焦仍可运行以便用户查看聊天，置顶CAN停止按钮始终可见。普通启动仍为手动待机。结果写入`output/hardware-validation/z-auto-latest.json`及本次报文日志。

自动轨迹暂限50RPM以内，`--cycles=30`指定30轮；高目标输入只开放手动台架入口。主程序保持单文件，无新增依赖。随着用户增加自动序列、34秒点动、交互调速，原450有效行预算扩至约513行，保留停止状态机及测试，不为压行删除保护；本轮23项离线检查通过，未把输入设置检查称作高转速实机验收。
