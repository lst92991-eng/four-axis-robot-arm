# SCARA 中国象棋项目

**2026-10-08已确认：棋类为中国象棋，目标是最小人机对战。** 后续以9列×10行交点、红黑双方和棋子种类识别为准。项目共识见 [AGENTS.md](AGENTS.md)。

技术基础为 **C++17 + OpenCV 5.0.0**。`vision/main.cpp` 保留NV12 → BGR → 保存照片的第一步教学示例；Ubuntu实时预览也已运行OpenCV5.0.0。此前四角定位和15×15判色原型只是视觉实验，尚未完成中国象棋棋子身份识别。

根目录 `main.cpp` 是04线程完成的板端皮卡鱼最小人机对战入口，详见文末。棋力引擎与视觉模型分别讨论；是否采用YOLO须按中国象棋需求判断。

## 历史视觉实验：五子棋 V2（非中国象棋棋盘）

以下15×15文件仅保留作此前的标记与成像实验，不作为当前中国象棋项目的正式打印稿。正式象棋棋盘需重新设计9×10布局及其坐标。

- [A4六页拼接版](output/pdf/gomoku-board-v2-a4-tiles.pdf)：单面、黑白打印，选择100%实际大小，按 `1 2 3 / 4 5 6` 摆放。横向重叠30mm、竖向重叠10mm；对齐重复网格，只裁拼接白边，从背面固定。
- [A1整张版](output/pdf/gomoku-board-v2-a1.pdf)：适合打印店按实际尺寸输出，不能缩印到单张A4。
- 标记黑框从18mm增大到48mm，四周完整白边8mm。外框510×510mm，仍为15×15交点、25mm格距，兼容原来的20mm纸棋子。打印后先测每页100mm标尺。
- 新版几何保存在 [V2坐标参数](assets/board/v2/board.json)。V2标记位置与旧版不同，换上新纸板时需同步切换识别坐标；当前实时预览尚使用旧板坐标。

## 历史视觉实验：五子棋原版与纸棋子

- 旧实验使用 [20 mm 五子棋纸棋子](output/pdf/gomoku-stones-a4.pdf)，共113枚黑子、112枚白子。这些圆片没有中国象棋的棋子身份，不作为象棋识别训练集。
- 旧五子棋实验按直径18～20mm设计；中国象棋棋子的尺寸和末端抓取适配需按实际棋具确定。
- 旧彩色版为 [A2 单页棋盘](output/pdf/gomoku-board-a2.pdf)，原打印要求是实际大小100%、格距25mm、暖黄褐底色。
- 没有 A2 打印条件时可用 [A4 六页拼接版](output/pdf/gomoku-board-a4-tiles.pdf)。按三列两行排列，横向重复区域 20 mm，纵向重复区域 10 mm，重叠对齐相同网格再固定。不能把六页缩印到一页。
- 棋盘有效底板 **410×410 mm**，15×15 个交点，交点跨度 350 mm。使用哑光纸并贴平在硬板上；不要让拼接处翘起，不要裁掉四角标记周围的白色区域。
- 相机固定在棋盘上方，先调整位置和高度，让整张棋盘及四个标记同时进入画面，并留少量边距。尽量正对棋盘，使用均匀照明，避免强反光。
- 标记 0 所在角是棋盘左上角；后续定位将借助四个不同 ID 识别方向。坐标从 0 开始，行向下、列向右。完整规格见 [board.json](assets/board/board.json)。
- 2026-10-08按实拍反馈制作V2大标记版，见上方入口；原版PDF与参数保留，避免混用坐标。

## 第一步怎么运行

程序部署在本板 `/root/scara-vision-v1`。在 **Windows PowerShell** 执行：

```powershell
adb -s 759dce9b268fd05e shell sh /root/scara-vision-v1/capture.sh
adb -s 759dce9b268fd05e pull /root/scara-vision-v1/live/source.jpg ./output/rv1106/source.jpg
```

相机为 MIS5001，采集节点 `/dev/video11`，输出 1280×960 NV12。脚本跳过前30帧后保存一帧，再运行转换程序。看到 `Done: live/source.jpg` 后查看照片的方向、清晰度和颜色即可，这一步不需要棋盘。

- [vision/main.cpp](vision/main.cpp)：读取第一帧 → `cvtColor()` → `imwrite()`。
- [vision/README.md](vision/README.md)：这一小步的数据格式说明。
- [tools/capture.sh](tools/capture.sh)：已准备好的板端采集入口。

## 中国象棋后续接入顺序

复用采集和定位经验，依次完成：中国象棋9×10棋盘及标定 → 识别双方与棋子种类 → 整理可信棋局或已确认走法 → 接入04线程的棋力引擎 → 机械搬运与执行确认。先统一棋盘坐标和接口，每次只增加一个可验证的步骤。

此前五子棋视觉实验实现在 [源码存档](output/rv1106/archive/vision-full-before-step1-2026-10-08.zip)，不参与当前构建。`output/rv1106/validation` 中五子棋合成测试不能作为中国象棋识别验收；阶段一格式转换记录使用 `step1-` 前缀。

## 修改后构建

本机 WSL `Ubuntu-24.04-D` 已有缓存 `/home/lst/.cache/scara-vision`。在 PowerShell 中执行：

```powershell
wsl -d Ubuntu-24.04-D -- bash /mnt/c/Users/lst/Desktop/scara/tools/build-opencv.sh
powershell -ExecutionPolicy Bypass -File .\tools\deploy.ps1
```

当前程序只链接 core、imgproc、imgcodecs。OpenCV 5.0.0 提交为 `40738fb16ceddb5fb3fea747585f7ce6abb0605b`；Rockchip GCC8.3/uClibc 工具链来自 Luckfox 提交 `824b817f889c2cbff1d48fcdb18ab494a68f69d1`，工具链 tree 为 `c3fe8cde3c2b3d1322aa620046c89d629bc2b88a`。工具链配置在 `cmake/rv1106.cmake`。

[早期开放源码调研](research/01-开源方案调研-2026-10-08.md)和五子棋打印测试文件按历史资料保留。`algorithm` 由03线程维护；根目录 `main.cpp` 与 `chess/` 的人机对战接入由04线程维护。

## 机械臂几何算法

`algorithm` 只保留 [正运动学](algorithm/fk.cpp) 和 [逆运动学](algorithm/ik.cpp)，数据类型和臂长在 [fk.hpp](algorithm/fk.hpp)。固定臂长 195/275 mm，长度用毫米、角度用弧度；Z 向上，工具中心与末端旋转轴同轴。这里的 Z 是末端高度，电机零点、丝杆导程、工具高度偏移需在接硬件时标定换算。

```cpp
#include "ik.hpp"

scara::Pose target{300.0, 100.0, 50.0, 0.0}; // x、y、z、朝向
scara::Joints joints{};
if (scara::inverse(target, true, joints)) { // true/false 选择肘角正/负分支
    scara::Pose check = scara::forward(joints);
}
```

逆解失败不修改输出；成功只表示理想几何可达，不代表通过真实限位或碰撞检查。角度按一圈归一化，实际运动仍需连续选解。尚未接入象棋入口或驱动电机。复数、独立旋转和卡尔曼滤波空文件已删除，当前没有相应调用需求。

独立编译：`cmake -S algorithm -B build/algorithm`，然后 `cmake --build build/algorithm`。接入时可用 `add_subdirectory(algorithm)` 并链接 `scara_algorithm`。

## 最小人机对弈

入口是 [main.cpp](main.cpp)，引擎接口位于 [chess/pikafish.hpp](chess/pikafish.hpp) 和 [chess/pikafish.cpp](chess/pikafish.cpp)。标准开局，人执红，皮卡鱼执黑，每步思考 1 秒；使用已真板验证的官方 `Pikafish-2023-03-05`，提交 `4762ca3c77c7f8ccbf204974cf23c5ca3e8c2783`，部署权重须为电脑端预解压后的 NNUE。

本阶段输入四字符走法代表“已经确认的人类走法”，例如 `e3e4`；a～i 为从红方看由左到右的列，0～9 从红方底线向黑方递增。合法性由引擎 `go perft 1` 查询，不另外编写一套棋规。黑方返回候选后，输入 `ok` 模拟落子已成功，才将该着法记入历史。`quit` 或 Ctrl-C 退出。

后续接视觉与机械臂时，替换两处入口即可：视觉确认的走法代替键盘输入；机械臂完成并经视觉确认代替 `ok`。当前没有执行真实机械臂动作。接口是单线程同步调用，等待引擎时支持取消；整机界面、急停和运动控制须由自己的主循环或任务继续响应，不能把该等待直接放进实时运动任务。这里未实现完整竞赛裁判。

Windows PowerShell 中交叉构建：

```powershell
wsl -d Ubuntu-24.04-D -- cmake -S /mnt/c/Users/lst/Desktop/scara -B /home/lst/.cache/scara-chess-minimal '-DCMAKE_TOOLCHAIN_FILE=/mnt/c/Users/lst/Desktop/scara/cmake/rv1106.cmake' '-DSCARA_TOOLCHAIN_ROOT=/home/lst/.cache/scara-vision/luckfox-toolchain/tools/linux/toolchain/arm-rockchip830-linux-uclibcgnueabihf' '-DCMAKE_BUILD_TYPE=Release'
wsl -d Ubuntu-24.04-D -- cmake --build /home/lst/.cache/scara-chess-minimal --parallel 2
```

已部署客户端 `/root/scara-chess-minimal/scara_chess`。在板端终端中运行：

```sh
/root/scara-chess-minimal/scara_chess /root/scara-pikafish-test/2023-03-05
```

网页查看器会常驻另一份约 90 MiB 的模型；运行本入口前需先关闭电脑上的 `tools/pikafish-viewer/server.py`，确保板上只有一个引擎。关闭浏览器标签本身不会关闭该服务。

真板测试包括两轮人机对弈、非法走法、未确认黑方走法就退出、EOF、缺少引擎、等待输入及思考中 Ctrl-C、引擎退出、引擎不响应超时；结果在 [board-tests.json](output/rv1106/chess-minimal/board-tests.json)。本次客户端 C++ 为 299 非空行（含注释）、290 个非空非独立注释行；另有 6 行 CMake，统计排除皮卡鱼源码、权重、网页和验证脚本。详细统计见 [code-metrics.json](output/rv1106/chess-minimal/code-metrics.json)。
