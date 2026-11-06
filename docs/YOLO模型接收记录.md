# 中国象棋 YOLO 模型接收记录

日期：2026-11-04。范围：接收、静态核查、资料与部署资产准备。没有运行训练、微调、导出、量化、checkpoint加载或NPU推理；包内复训命令只是来源资料，不作为执行指令。

## 结论

优先使用用户已提供的 `xiangqi14-s.rknn`，暂不重新转换。原始RKNN文件7,395,875字节；SHA-256为 `c843fac9714d97ca3b478193fb98557fef3146ca4c216a98380295a87c693943`。

当前板端实测的 `/oem/usr/lib/librknnmrt.so` 为1.6.0 (`2de554906@2024-01-17T14:53:41`)，SHA-256 `d6259f5f8537acc15b0c3686010109bf4182e654281aad180a44bfd73ce929cc`，与包内历史实板库完全一致；rknpu.ko文件版本0.9.2。运行库字节匹配降低了兼容性不确定性，但不能代替当前板模型加载及当前棋盘精度验收。未替换运行库或加载NPU模块。

## 本次校验

- 外层ZIP：34,227,464字节，计算SHA-256 `a60e291e42452c809752c89aafc7e4c2d59389021487a9f5f0c0dc53665c32b6`；用户未另附外层校验值，此值用于本次留档。
- ZIP CRC、包内18个交付文件的大小/SHA-256、第二份SHA256SUMS全部通过，无清单外文件。
- classes.csv、转换manifest和来源说明的14类顺序一致；RKNN文件头为RKNN。
- 嵌套数据包CRC通过；结构统计为训练174图/2213框、验证22图/204框、测试22图/277框；标签ID和坐标基本范围有效。这不是逐图语义复核，也没有重新验证近重复划分。
- `.pt`是原YOLOv5 checkpoint，不是TorchScript；只校验字节，未反序列化。

## 固定接入约定

| 项目 | 包内约定 |
| --- | --- |
| 架构 | airockchip YOLOv5s / ReLU，rk_opt@v6.2.1，源码提交23a20ef3e5fa648e765e4a70b30edbfd3aaf8c8c |
| 输入 | RGB；保持比例letterbox到640×640；填充值114；保留缩放与补边以还原框 |
| 归一化 | RKNN转换已配置mean=0、std=255；应用按约定送RGB字节，不再次除255 |
| 输出 | 3个已sigmoid概率头，stride=8/16/32；每头3个anchor，每anchor19值，即57逻辑通道；仍需坐标解码 |
| INT8内存 | 查询各输出张量原生NHWC属性、scale、zero-point和stride；历史实测C=57、w_stride=0，不能用较大的分配容量推成C=64 |
| 过滤 | objectness×类别概率，score=0.50；跨类别NMS IoU=0.45；沿用原基准，不在独立test上调阈值 |
| 类别 | `kabnrcpKABNRCP`：黑将士象马车炮卒，红帅仕相马车炮兵；空格不在检测类别中 |

输入manifest的NCHW导出形状，不等同于板端原生I/O内存布局；运行时必须查询实际属性。原`.pt`常规已解码输出与RKNN三个概率头不是同一接口，不能套同一解析函数，也不能重复做sigmoid。

模型不负责棋盘定位、90格映射、棋规或运动。当前原始视图红方在右，是否旋转/裁剪应先对照模型参考输入验证，再保持几何反变换一致；不因预览方向直接更改类别或训练。

## 历史证据与当前边界

`validation.historical.json`报告日期为2026-10-09；其中独立test为22图/277框，在类别一致且IoU≥0.50的一对一匹配口径下TP277、FP0、FN0；检测阶段平均85.31ms，包含letterbox、NPU和NMS，不含取图/文件解码。以上均为历史报告，不是今天相机上的测量，也不能解释为当前项目100%准确或30fps识别。

原转换manifest保留“仅转换、待实测”的历史字段；后续历史实板结果以validation报告为准，本次原始文件不改写。来源说明还记录一处训练标签把红相标成class7红帅；保留此问题，原数据和模型不修改，也不据此启动复训。当前棋具上的分类、漏检、边角、反光、旋转和遮挡仍需独立实拍检验。

## 已准备与尚缺

- 本仓库运行资产：[RKNN](../models/xiangqi14-s/xiangqi14-s.rknn)、[原转换manifest](../models/xiangqi14-s/conversion.original.json)、[类别表](../models/xiangqi14-s/classes.csv)、[历史验证](../models/xiangqi14-s/validation.historical.json)、[来源原文](../models/xiangqi14-s/SOURCES.original.md)及原许可文件。
- 已填写本项目接入清单：[model-manifest.json](../config/model-manifest.json)。`board_inference_verified`和`current_camera_accuracy_verified`保持false；规划配置仍禁止运动。
- PT、完整数据ZIP及模型工具原字节保留在项目外接收缓存；外层包位置、缓存位置和哈希写入接入清单，没有把训练环境装进板子。
- 包内没有已验证的`vision/detector.cpp`/`detector.hpp`、OpenCV5目标库、RKNN头文件或完整SCARA应用，仅提供旧detector源码哈希。下一步优先取得这些现有推理源码与构建环境，复用历史后处理；若无法取得，再实现最小兼容推理入口并做参考样例对照。
- 棋盘横纵格距仍未收到。先做棋盘几何、模型单图推理与90格叠加，得到当前实拍结果后再联动棋局；不直接进入机械臂动作。

本轮没有提交Git，没有重新训练或修改四轴固件。
