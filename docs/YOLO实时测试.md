# YOLO 实时测试（2026-11-04）

用户已授权直接实时测试已有YOLO。使用原RKNN文件，不训练、不微调、不转换模型，不连接任何机械臂指令接口。

## 看画面

当前Ubuntu虚拟机Firefox打开：`http://192.168.31.233:8088/`。用户已确认看到实时识别框（2026-11-04）。

也可在虚拟机终端单独播放识别画面：

```sh
ffplay -fflags nobuffer -framedrop http://192.168.31.233:8088/stream.mjpg
```

原相机地址`rtsp://192.168.31.233/live/0`仍是无识别框的30fps视频。识别页面显示中文类别、置信度、推理耗时与帧号。网页会提示超过3秒未更新；旧画面不能当作新结果。

## 本次链路和边界

```text
SC3336 → 板端H.264 RTSP → Windows取最新帧、裁剪与RGB letterbox
→ RV1106实际NPU运行原INT8模型 → 原生57通道INT8输出
→ Windows反量化、解码、跨类别NMS、绘框
→ 上传已绘框JPEG → RV1106 HTTP/MJPEG → Ubuntu虚拟机显示
```

这是临时效果测试入口，不是已完成的全板端C++/OpenCV5生产程序。主机使用Python3.13、OpenCV4.13.0、NumPy、Pillow；板端使用自带Python3.11 ctypes调用当前RKNN1.6.0，不需要在板子安装Torch或训练环境。页面服务是局域网临时服务，不对公网部署；推理/预览上传只接收本次Windows地址192.168.31.164，板子地址变化需更新启动参数。

保留原模型score0.50、跨类别NMS0.45。实际查询输入NHWC `[1,640,640,3]`、UINT8字节输入；输出分别 `[1,80,80,57]`、`[1,40,40,57]`、`[1,20,20,57]`。各输出查询scale/zero-point，按57有效通道访问，不从分配容量推导64通道；sigmoid已经在图中，不重复应用。每次请求、应答及绘框共用frame_id，绘框使用送入该次推理的图像。

当前手工ROI为原1280×720画面上的`x=300,y=100,w=620,h=600`，包含当前棋盘，旋转0°。ROI用于减少桌面背景和保留棋子像素，不是自动棋盘定位或物理标定。模型框坐标记录在裁剪图内；恢复原帧坐标需加ROI偏移，本次未生成机械臂坐标。移动相机/棋盘后应重新检查ROI，不把漏出ROI的棋子当空格。

## 实际结果

- 当前板成功加载原模型，运行库1.6.0、驱动0.9.2，与接收资料相符。
- 先用提供的独立test22复核接入链路：22图/277标注框，在类别一致、IoU≥0.50的一对一匹配下TP277、FP0、FN0。没有改阈值或用该test训练模型；这是旧数据分布的重放，不是当前棋盘的精度评估。
- 当前全景直接letterbox的样图在0.50阈值下检出0框；裁出完整棋盘后才有框。检查过0/90/270°等输入朝向，仍有明显漏检与错分类。实时结果按模型原样展示，不按棋局或棋子数量修正。
- 实际NPU耗时约84ms；临时网络链路预览约2.5～3fps，不能把NPU推理耗时等同于端到端耗时或相机30fps。页面的host age仅从电脑收到解码帧起计，不是曝光到显示的完整延迟。
- 当前实拍可见将/炮/车等错分类，检出数随帧变化；未逐子建立人工真值，不能报告当前盘面准确率。**实时推理链路已通，当前棋具识别验收未通过，不可据此自动走棋。**

证据：[汇总](../temp/rv1106-logs/20261104-yolo-live-summary.json)、[test22重放](../temp/rv1106-logs/yolo-reference/current-board-test22.json)、[实时预览抓帧](../temp/rv1106-logs/yolo-live-active/observed-preview.jpg)。`temp/rv1106-logs/yolo-live-active/detections.jsonl`持续记录实际检测，不将空检测补成真值。

## 复现和停止

相机仍由原ADB前台会话运行。NPU服务：

```sh
# 板端，当前模型和脚本已在该目录；模块已加载则跳过insmod。
test -d /sys/module/rknpu || insmod /oem/usr/ko/rknpu.ko
python3 /userdata/scara-yolo-test/board_npu.py \
  --model /userdata/scara-yolo-test/xiangqi14-s.rknn \
  --bind 192.168.31.233 --client 192.168.31.164 --port 8088
```

主机入口为`temp/yolo_live/host_preview.py --roi 300 100 620 600 --publish --out 结果目录`，使用本机`C:/Users/shuto/AppData/Local/arm4-tools/can-venv/Scripts/python.exe`。测试环境pip版本由当次记录给出；当前依赖opencv-python-headless4.13.0.92、numpy2.5.4、pillow13.0.0。再次运行使用新的结果目录，避免覆盖前次检测记录。

停止主机取流进程后，网页会显示过期状态；停止板端NPU服务用其原会话Ctrl+C，或确认`ps`中的精确PID后发送TERM，服务会释放I/O内存和模型上下文。不要广泛结束所有python/adbd/相机进程。退出ffplay只关闭播放器，不停止推理和相机。

本轮使用的是临时测试接口，不把包内缺失的历史detector.cpp宣称为已复用。ABI与RV1106零拷贝调用顺序参考[Rockchip固定提交示例](https://github.com/airockchip/rknn_model_zoo/blob/eaa94d6f57ca553d493bf3bd7399a070452d2774/examples/yolov5/cpp/rknpu2/yolov5_rv1106_1103.cc)，头文件来自相同提交；模型来源及原许可仍保留在models/xiangqi14-s目录。
