# 跨设备重构阅读材料（2026-11-16）

应用和完整历史请查看[重构分支](https://github.com/lst92991-eng/four-axis-robot-arm/tree/codex/upper-refactor-20261116)，起点为 workspace-handoff-20261116（569ed7b）。在该分支中重构和提交。

此分支用于阅读，不参与应用编译：

- [完整 C++ 代码阅读稿](output/game-direct-20261115/round-01/cpp-code.txt)。
- [原电机驱动源码](output/vision-review-15/reference/MotorDriver)是用户指定的手写代码风格参考；先看 Control/pid.c、Control/wheel_control.c、Board/motor_driver.c。来源及原始哈希记录在应用分支的 docs/evidence/2026-11-13/vision-review-15/reference.json。
- [离线检查日志](output/game-direct-20261115/round-01/build-test.log)和[文件字节校验](source-files.json)。
- 全部补充资料的校验和和附件路径见[交接标签的清单](https://github.com/lst92991-eng/four-axis-robot-arm/blob/workspace-handoff-20261116/docs/evidence/2026-11-16/workspace-assets.json)。

这里还保存[冻结的离线基准](refactor-baseline.json)：16 帧的 RGB、绘图哈希和检测结果，以及 4 份坐标输出的哈希。无需下载大包也可以复验，原图和张量已在应用分支的 tests/fixtures 中。

从应用仓库根目录用新编译的程序运行（Windows 示例）：

~~~powershell
New-Item -ItemType Directory -Force output/refactor-check/geometry | Out-Null
.\build\commit-verify\scara_vision_replay.exe tests/fixtures/vision/review-corpus.json output/refactor-check/replay
.\build\commit-verify\scara_geometry_tool.exe calibrate tests/fixtures/upper-app/measured-pairs.json output/refactor-check/geometry/plane.yml
.\build\commit-verify\scara_geometry_tool.exe inspect tests/fixtures/fixed-app/setup.json output/refactor-check/geometry/grid.json
python <资料分支检出目录>/verify-replay.py output/refactor-check/replay output/refactor-check/geometry
~~~

15 种对弈场景的预期状态和动作记录在应用分支的 docs/evidence/2026-11-15/game-direct-20261115/program-equivalence.json。回交时保持原输入并由接收端独立复验。
