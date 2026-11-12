#pragma once
#include <opencv2/core.hpp>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace scara {
// 固定模型：每个候选框有 4 个位置数、1 个目标分数和 14 个类别分数。
constexpr int YoloSize = 640, YoloClasses = 14, YoloAnchors = 3, YoloHeads = 3;
constexpr int YoloBoxValues = 5 + YoloClasses;
constexpr int YoloChannels = YoloAnchors * YoloBoxValues;
constexpr int YoloGrid[YoloHeads] = {80, 40, 20};

// 识别结果交给棋盘模块使用，框的位置和尺寸均为原始图像像素。
struct Detection
{
  int classId = -1;
  double confidence = 0;
  cv::Rect2d bbox;
};

//---------图像与模型参数区---------
struct VisionConfig
{
  cv::Size frameSize{1280, 720};
  cv::Rect inferenceRoi{300, 70, 650, 600};
  float scoreThreshold = .50f;
  float nmsThreshold = .45f;
};

struct ImageMapping
{
  cv::Rect roi;
  double scale = 1;
  int left = 0, top = 0;
};

// NPU 原生输出为 NHWC INT8。跨度和容量均以字节计。
struct YoloHead
{
  const int8_t *data = nullptr;
  size_t bytes = 0;
  size_t rowBytes = 0;
  int zeroPoint = 0;
  float scale = 0;
};

//---------图像处理接口区---------
cv::Mat Vision_prepare(const cv::Mat& bgr, const VisionConfig&, ImageMapping&);
std::vector<Detection> Vision_decode(const YoloHead heads[YoloHeads],
                                    const ImageMapping&, const VisionConfig&);
cv::Mat Vision_draw(const cv::Mat& bgr, const std::vector<Detection>&,
                    const cv::Mat& chineseLabels);

//---------NPU接口区---------
// 模块内只保存一个模型，由主流程依次调用这些接口。
void Yolo_open(std::string modelPath);
// 返回三路输出数组，数据在下一次推理或关闭模型前有效。
const YoloHead *Yolo_run(const cv::Mat& rgb640);
void Yolo_close();
}
