#pragma once
#include <opencv2/core.hpp>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace scara
{
// 固定模型：每个候选框有 4 个位置数、1 个目标分数和 14 个类别分数。
constexpr int YoloSize = 640;
constexpr int YoloClasses = 14;
constexpr int YoloAnchors = 3;
constexpr int YoloHeads = 3;
constexpr int YoloBoxValues = 5 + YoloClasses;
constexpr int YoloChannels = YoloAnchors * YoloBoxValues;
constexpr int YoloGrid[YoloHeads] = {80, 40, 20};
constexpr int AtlasWidth = 64;
constexpr int LabelHeight = 24;

// 识别结果交给棋盘模块使用，框的位置和尺寸均为原始图像像素。
struct Detection
{
    int classId = -1;      // 0～6 为黑方，7～13 为红方，顺序与模型一致。
    double confidence = 0; // 目标分数 × 类别分数。
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
    cv::Rect roi;     // 原图中的裁剪区域，单位：原图像素。
    double scale = 1; // 输入图像素 / 原图像素，X、Y 共用此缩放比。
    int left = 0;     // 640 输入图左侧的留白像素。
    int top = 0;      // 640 输入图顶部的留白像素。
};

// NPU 原生输出为 NHWC INT8，布局与量化参数在 Yolo_open 中检查一次。
struct YoloHead
{
    const int8_t *data = nullptr;
    size_t rowBytes = 0; // 相邻两行的实际间距，包含硬件对齐字节。
    int zeroPoint = 0;
    float scale = 0;
};

//---------图像处理接口区---------
// 输入原图 BGR；返回 640×640 RGB，同时填写后处理要用的缩放和留白。
cv::Mat Vision_prepare(const cv::Mat &bgr, const VisionConfig &config, ImageMapping &mapping);
// heads 来自成功的 Yolo_run；离线文件读取入口负责检查自己的数据边界和格式。
std::vector<Detection> Vision_decode(const YoloHead heads[YoloHeads], const ImageMapping &mapping,
                                     const VisionConfig &config);
// 复制原图后绘制。检测来自 Vision_decode，字图在载入时核对尺寸并转换为灰度。
cv::Mat Vision_draw(const cv::Mat &bgr, const std::vector<Detection> &detections, const cv::Mat &chineseLabels);

//---------NPU接口区---------
// 模块内只保存一个模型。主流程统一调用 Yolo_close，包括打开或运行失败时。
void Yolo_open(std::string modelPath);
// 返回三路输出数组，数据在下一次推理或关闭模型前有效。
const YoloHead *Yolo_run(const cv::Mat &rgb640);
void Yolo_close();
} // namespace scara
