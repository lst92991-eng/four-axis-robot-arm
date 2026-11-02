# 象棋 NPU 检测模型约定

本目录只存离线模型工具和来源说明。线上识别是 OpenCV 5 C++ + RV1106 NPU，禁止把 Python/网络 API 作为线上识别替代。

## 当前硬件及固定来源

- 实板 `armv7l`，运行库 `/oem/usr/lib/librknnmrt.so`：`1.6.0 (2de554906@2024-01-17T14:53:41)`；板上 `rknpu.ko` 标注驱动 `0.9.2`。初次调查时模块尚未加载，未据此运行任何推理。
- [RKNN Toolkit2 v1.6.0](https://github.com/airockchip/rknn-toolkit2/tree/b25dadacc24b88eb7dfcaa47c9c525ecca89b319)，提交 `b25dadacc24b88eb7dfcaa47c9c525ecca89b319`。转换环境使用其 Python 3.10 / x86_64 wheel `rknn_toolkit2-1.6.0+81f21f4d-cp310-cp310-linux_x86_64.whl`。
- [RKNN Model Zoo v1.6.0](https://github.com/airockchip/rknn_model_zoo/tree/eaa94d6f57ca553d493bf3bd7399a070452d2774)，提交 `eaa94d6f57ca553d493bf3bd7399a070452d2774`；RV1106 示例查询 `RKNN_QUERY_NATIVE_NHWC_OUTPUT_ATTR`。本项目参考其 `examples/yolov5/cpp/rknpu2/yolov5_rv1106_1103.cc` 和 `postprocess.cc`，保留 Apache-2.0 许可证于本目录。
- [airockchip/yolov5 rk_opt@v6.2.1](https://github.com/airockchip/yolov5/tree/23a20ef3e5fa648e765e4a70b30edbfd3aaf8c8c)，提交 `23a20ef3e5fa648e765e4a70b30edbfd3aaf8c8c`。离线训练/导出源码放缓存，其 GPL-3.0 许可证随源码保留，不把整个训练框架复制进课程客户端。

这份 Rockchip 分支的 Conv 默认激活是 ReLU。最终训练选它的 `models/yolov5s.yaml`，输入 640，关闭 autoanchor。首轮 nano 模型字形分类不足，未作为交付模型。禁止对已经训练好的 SiLU 权重直接更换激活函数后冒充等价模型；导出脚本会拒绝该 SiLU 情况。

## 检测接口及输出

`detector_load` 加载模型并分配 RKNN 内存；`detector_detect` 接受 `CV_8UC3` RGB 图像，内部 letterbox 到 640×640，再执行 NPU 和跨类别 NMS；`detector_close` 释放 NPU 资源。单颗棋子的类别互斥，跨类别 NMS 去掉不同分支对同一棋子的重复框。输出方框已映射回传入 RGB 图的像素坐标。

类别按 `kabnrcpKABNRCP` 固定：黑将、士、象、马、车、炮、卒；红帅、仕、相、马、车、炮、兵。导出脚本实际重排分类卷积的权重与偏置，再保存原始类别表及置换记录，不能只改标签名字。

模型必须是 YOLOv5 的三个概率头，逻辑通道 `3*(5+14)=57`，步长依次 8/16/32。后处理核对逻辑 shape，再使用原生 NHWC 属性里的通道和行步长访问 INT8 张量，保留 scale/zero-point 反量化。标准 anchors 固定为 `[10,13,16,30,33,23]`、`[30,61,62,45,59,119]`、`[116,90,156,198,373,326]`。训练必须关闭 autoanchor；导出时再次核验真实 anchors。

## 数据与权重调查状态

已训练真实 14 类象棋模型，完成 RKNN 转换及 RV1106 实板的内部验证/独立测试。它仍需由主程序另做用户相机、实际棋具、棋盘定位和整局闭环验收。完整数字与每图结果见 `validation.json`。

- 实际训练包 `yolo14-v1.zip`：SHA-256 `b573f76bd8f6aff8718108080fc3372795a2ba8255744eeb4c14725be4e2494e`。照片来自 [nrl-ai/chessai-data](https://github.com/nrl-ai/chessai-data/tree/6fbcacfc9ab9aade3d034ee6e9ecf821d1a9d397/data/data_02)，原始来源为 [Roboflow Chinese Chess v2](https://universe.roboflow.com/hommoner-oclm2/chinese-chess-jtqlg/dataset/2)，数据声明 CC BY 4.0。
- 原资料只有 7 类棋子类型；本项目依据墨色派生红黑 14 类，并作完整视觉复核，6 个边界框由主线程复核。不是原作者提供的 14 类标注，也不是模型预测生成的标签。
- 排除 3 张无法辨认身份的绿色背面无标注图后，共 218 张照片、2,694 个框；训练 174 图/2,213 框，验证 22 图/204 框，独立测试 22 图/277 框。按 202 个原图/近重复连通组隔离，各分集无相同 SHA 或检测阈值内近重复跨组。该内部划分不能代替跨棋具、跨背景或用户实拍验收。
- GPU 训练位于 WSL `Ubuntu-24.04-D` 的 `/home/lst/.cache/scara-xiangqi-train`，计划最多 100 epochs、早停 patience=20、seed=42、batch=4、关闭 autoanchor 和水平翻转，允许 ±15° 旋转。COCO 预训练权重只用于卷积权重初始化，随后针对真实象棋派生标签训练全部网络；不能将初始化权重作为完成的象棋模型。

首轮 YOLOv5n 在 87 epochs 早停，验证集 mAP@50 约 0.20：框位置和颜色已学到，但字形大多混淆成兵/卒，因此明确淘汰。检查了预训练参数迁移、类别顺序、坐标及缩放，不将问题归咎于随机初始化。

第二轮采用 YOLOv5s/ReLU，AdamW，学习率 0.001，batch=4，150 epochs，patience=30，seed=42；分类损失权重提高到 2.0，关闭 mosaic，缩放增强降到 0.2，避免缩小棋字。官方 COCO 预训练卷积权重只作初始化，14 类检测头和整网均在本项目真实照片派生标签上训练。选定 best 后才首次使用独立测试集；之后没有依据 test 调整训练参数。

训练环境实际为 Torch 2.7.1+cu128、TorchVision 0.22.1+cu128、OpenCV-Python 5.0.0.93（运行版本 5.0.0）、NumPy 2.2.6，RTX 5060 Laptop GPU 的 sm_120。CUDA 矩阵运算、AMP 卷积反向和 GPU NMS 均已实测。现代依赖兼容改动仅作用于缓存 vendor 源码，完整补丁见 `vendor-compat.patch`。

## 最终模型与实板结果

- 模型：`xiangqi14-s.rknn`，7,395,875 字节；SHA-256 `c843fac9714d97ca3b478193fb98557fef3146ca4c216a98380295a87c693943`。
- 使用 174 张真实训练图校准 INT8；校准图由 OpenCV 5 做 640×640 letterbox。验证和测试图片未参与量化校准。
- GPU 官方评估：独立 test 22 图/277 框，mAP@50=0.995、mAP@50–95≈0.845。它是当前同类棋具/背景来源内的实验划分指标，不是跨棋具泛化或用户相机精度。
- 固定实际部署阈值：score=0.50、NMS IoU=0.45、跨类别去重。跨类别 NMS 基于 val 中一个同一棋子重复分类框确定；定稿后才运行首次 NPU test。
- 实板 OpenCV 运行库和头文件均为 5.0.0。验证集 204 框、独立测试集 277 框，在类别一致且 IoU≥0.50 的一对一匹配下均无漏框或额外框；不是把 mAP 写成“准确率”。相同预处理/阈值/NMS 的 FP32 参考结果也均完整匹配。
- 独立 test 上 INT8 与 FP32 匹配框的平均角点绝对差约 0.405 像素，平均置信度差约 0.00258；不据此宣称所有图片均无量化误差。
- 实板完整 `detector_detect`（OpenCV letterbox + NPU + NMS）平均 85.31 ms，最大 86.11 ms；不包含相机采集、文件解码、画图或传输。
- SDK 内存查询：weights=7,117,184 B、internal=6,144,000 B、DMA allocated=13,467,648 B；用户 I/O 张量分配容量合计 1,766,400 B。进程 VmHWM 约 8.5 MiB 是另一统计口径，不能作为总 NPU 占用，也不能把这些字段盲目相加。各阶段 MemAvailable/CmaFree 快照保留在报告中。
- 测试后所有模型上下文和探针进程已释放，清理临时输入后 MemAvailable 恢复约 146 MiB。没有替换实板运行库、改开机脚本或操纵机械臂。

重要布局核查：此模型 native NHWC 的 `dims[3]=57`、`w_stride=0`，有效数据按 57 通道紧密存储；`size_with_stride` 分别为 409600/102400/25600，而有效 payload 为 364800/91200/22800。较大的分配容量不能用于反推逐像素 C64 步长。已通过实板数据及完整类别/位置匹配验证当前读取方式。

另行调研但未作为当前训练数据的候选：

- [project-xdges/Chinese-chess](https://universe.roboflow.com/project-xdges/chinese-chess-knfpp/dataset/9)：CC BY 4.0；原项目 248 图，v9 增强后 594 图，公开页面标注训练 519、验证 50、测试 25。14 类为 Advisor/Cannon/Elephant/General/Knight/Pawn/Rook 各黑红。必须拿到真实导出包并核查 `data.yaml`、分集和图片，不能把网页报告的指标当本项目实测。
- [FINALTEST/CHINESE CHESS FPT](https://universe.roboflow.com/finaltest-lqish/chinese-chess-fpt)：页面标注 CC BY 4.0、685 图、14 类，尚未选作本项目数据来源。
- [VinXiangQi](https://github.com/Vincentzyx/VinXiangQi)：GPL-3.0，主要识别电脑游戏截图，15 类还包括棋盘。不能未经真实棋盘验证就替代本项目相机检测。
- [bansh98/AI_chess_recongnition](https://github.com/bansh98/AI_chess_recongnition/tree/107017cc8c88da86430ea0dd4da1c16645829861)：仓库根 MIT，已静态核对其 `best.pt` 只含 `r-bing`、`b-zu` 两类，因此排除。未执行该 checkpoint 的 pickle 内容。

## 离线使用

### 最小交付清单

运行包只需 `xiangqi14-s.rknn`、同名转换 manifest、当前验证报告和来源/许可说明。离线复现附件再加固定的 `xiangqi14-s-best-portable.pt`、`yolo14-v1.zip`、本目录两个 Python 工具、`train-hyp.yaml` 和 `vendor-compat.patch`。不把训练框架、venv、pip 缓存、COCO 初始化权重或中间 ONNX 放进业务源码包。

固定 portable checkpoint 是本次已验证导出的确切输入，已去掉优化器和 NumPy 2 统计对象，方便 Torch 1.13.1 读取；网络和权重保留。使用交付权重无需重新训练。重新训练用于教学/后续改进，不承诺不同 CUDA 环境得到逐字节相同的权重；已交付文件以 `DELIVERY-SHA256.json` 的原始字节哈希校验。

派生数据 ZIP 内保留原 `README.roboflow.txt`/`README.dataset.txt` 的来源与 CC BY 4.0 声明、`DERIVED-LABELS.md`、原图与标注 provenance、分组划分及每类统计。本目录保留训练框架的 GPL-3.0 和 Rockchip 示例的 Apache-2.0；来源与许可按所属材料分别记录，不混称一种许可证。

### 复训命令（在独立训练环境中）

训练依赖的实测组合：Python 3.12、Torch 2.7.1+cu128、TorchVision 0.22.1+cu128、NumPy 2.2.6、OpenCV-Python 5.0.0.93、Pillow 11.1.0、PyYAML 6.0.2、Pandas 2.2.3、Matplotlib 3.10.1、Seaborn 0.13.2、SciPy 1.15.2、TensorBoard 2.19.0、protobuf 5.29.5、setuptools 69.5.1。旧 YOLOv5 的 NumPy/Pillow/Torch 兼容改动由随附补丁记录。

1. 在项目外取得上述固定提交的 `airockchip/yolov5`，在该目录执行 `patch -p1 < /path/to/tools/model/vendor-compat.patch`。
2. 解压数据 ZIP，在其 `data.yaml` 的副本顶部添加 `path: /absolute/path/to/yolo14-v1`；原始包不改。
3. 初始化权重使用 `https://github.com/ultralytics/yolov5/releases/download/v6.2/yolov5s.pt`。哈希记录在 `validation.json` 的 training 字段。不是拿它直接识别象棋。
4. 从固定 YOLOv5 源码目录执行下列一条训练命令；`$DATA`、`$INIT`、`$TOOLS`、`$RUNS` 分别为数据 YAML、初始化权重、本目录和缓存输出目录的绝对路径。

```sh
python train.py --weights "$INIT" --cfg models/yolov5s.yaml \
  --data "$DATA" --hyp "$TOOLS/train-hyp.yaml" \
  --imgsz 640 --batch-size 4 --epochs 150 --patience 30 --seed 42 \
  --optimizer AdamW --noautoanchor --workers 2 --device 0 \
  --project "$RUNS" --name xiangqi14-s
```

训练与测试进程清除外部 `PYTHONPATH`/`LD_LIBRARY_PATH` 干扰；本次另设 `YOLOv5_AUTOINSTALL=false`、`WANDB_MODE=disabled`、`MPLBACKEND=Agg`、`OMP_NUM_THREADS=4`。只有训练/验证集参与选模型。固定 best 后，官方独立测试命令为 `python val.py --weights best.pt --data "$DATA" --task test --imgsz 640 --batch-size 4 --device 0 --conf-thres 0.001 --iou-thres 0.6`。这是 mAP 评估配置，实际部署阈值另见上文，不能混为一种指标。

### 复现固定模型的导出

不必复训，直接使用交付的 portable checkpoint。在 OpenCV 5 环境执行：

```sh
python prepare_calibration.py /path/to/yolo14-v1 /path/to/calibration
```

得到 174 张只来自 train 的 PNG、`calibration.txt` 和逐图哈希 manifest。移动目录后应重新生成清单中的绝对路径。在下述独立 RKNN 1.6.0 环境，用固定 portable checkpoint 执行本节末的导出命令。保留交付 RKNN 作为已经实板验证的固定版本；新的转换产物要重新记录哈希与验证结果。

依赖、数据和权重放项目以外的缓存。Ubuntu 转换缓存为 `/home/st/.cache/scara-xiangqi-model`；固定 SDK 头位于其中 `3rdparty/rknpu2/include/rknn_api.h`，训练源码位于 `yolov5`。构建 C++ 时由调用工程指向该 SDK 头和实板复制的 `board-runtime/librknnmrt.so`，无需改动系统库。实板库 SHA-256 为 `d6259f5f8537acc15b0c3686010109bf4182e654281aad180a44bfd73ce929cc`，ELF32 ARM hard-float，依赖 `libc.so.0`；官方 v1.6.0 tag 同名库不是相同字节，不静默替换实板运行库。

隔离的 Python 3.10 转换环境位于缓存下 `venv-rknn`，使用 Torch 1.13.1+cpu、RKNN Toolkit2 1.6.0、ONNX 1.14.1、ONNX Runtime 1.16.0、NumPy 1.23.5。已验证导入及 `rknn.config(target_platform='rv1106')`，未据此声称模型转换成功。老 Toolkit 的 Python OpenCV 4.8.1 依赖仅用于离线转换，不替换主程序或 Ubuntu 本机的 OpenCV 5 C++ 库。该环境用 CPU 版 TensorFlow 2.8.0 提供 SDK 需要的 `tensorflow` 模块，SDK wheel 使用 `--no-deps` 安装以避免再装完整 TensorFlow 包。

Windows RTX 5060 的 GPU 训练环境与此转换环境分开；不能把老 Torch 1.13 CPU 转换环境当作支持该显卡的训练环境。

取得真实数据并训练后，运行：

```text
python export_rknn.py trained.pt calibration.txt output.rknn /path/to/pinned/yolov5
```

`calibration.txt` 每行是预先 letterbox 到 640×640 的真实训练图片绝对路径，背景填充 RGB(114,114,114)。不能使用 COCO 图片或纯合成图片充当象棋校准与验收数据；测试集不能参与训练、校准和选阈值。PyTorch 1.13 checkpoint 使用 pickle，只加载自己训练或已可信审查的文件。

转换成功仅证明生成了 RKNN 文件。CPU/ONNX 输出对齐、INT8 精度、实板 NPU 推理、真实相机整盘识别准确度和内存峰值需要分别验收。NPU 检测结束后应释放模型，再让棋力引擎运行，以实测结果决定是否允许两者同时驻留。
