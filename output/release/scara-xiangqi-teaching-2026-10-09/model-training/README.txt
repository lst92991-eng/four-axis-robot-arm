固定模型复现附件

直接运行不需要重新训练。板端模型位于：
../output/rv1106/scara-robot/models/xiangqi-yolov5s-relu.rknn
它与来源清单中的xiangqi14-s.rknn是同一字节内容，只改了部署文件名。

xiangqi14-s-best-portable.pt：本次导出的固定输入权重。
yolo14-v1.zip：真实照片与派生14类标注，内附CC BY 4.0来源/划分/派生说明。
xiangqi14-s.json：实际RKNN转换manifest。
SOURCES.md：训练及导出步骤；validation.json：真实NPU与FP32对照。
original-source-hashes.json：原始附件哈希；整个交付包另有根SHA256SUMS。

重新训练不保证与交付权重逐字节相同。新模型必须重新验证；不能把当前测试集数字当作新棋具实拍精度。
