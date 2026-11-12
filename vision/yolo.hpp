#pragma once
#include "geometry.hpp"
#include <cstdint>

namespace scara {
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
std::vector<Detection> Vision_decode(const YoloHead heads[3],
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
