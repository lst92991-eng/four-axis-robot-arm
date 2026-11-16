# 跨设备重构阅读材料（2026-11-16）

应用和完整历史请查看[重构分支](https://github.com/lst92991-eng/four-axis-robot-arm/tree/codex/upper-refactor-20261116)，起点为 workspace-handoff-20261116（569ed7b）。在该分支中重构和提交。

此分支用于阅读，不参与应用编译：

- [完整 C++ 代码阅读稿](output/game-direct-20261115/round-01/cpp-code.txt)。
- [原电机驱动源码](output/vision-review-15/reference/MotorDriver)是用户指定的手写代码风格参考；先看 Control/pid.c、Control/wheel_control.c、Board/motor_driver.c。来源及原始哈希记录在应用分支的 docs/evidence/2026-11-13/vision-review-15/reference.json。
- [离线检查日志](output/game-direct-20261115/round-01/build-test.log)和[文件字节校验](source-files.json)。
- 全部补充资料的校验和和附件路径见[交接标签的清单](https://github.com/lst92991-eng/four-axis-robot-arm/blob/workspace-handoff-20261116/docs/evidence/2026-11-16/workspace-assets.json)。
