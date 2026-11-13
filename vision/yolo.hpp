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

// NPU 原生输出为 NHWC INT8，布局与量化参数在 Yolo_open 中检查一次。
struct YoloHead
{
    const int8_t *data = nullptr;
    size_t rowBytes = 0; // 相邻两行的实际间距，包含硬件对齐字节。
    int zeroPoint = 0;
    float scale = 0;
};

//---------图像处理接口区---------
// 启动时设置一次已选定的有效图像几何；正常循环只处理同一相机和裁剪区域。
void Vision_setup(cv::Size frameSize = {1280, 720}, cv::Rect roi = {300, 70, 650, 600});
cv::Mat Vision_prepare(const cv::Mat &bgr);
// heads 来自成功的 Yolo_run；离线文件读取入口负责检查自己的数据边界和格式。
std::vector<Detection> Vision_decode(const YoloHead heads[YoloHeads]);
// 复制原图后绘制。检测来自 Vision_decode，字图在载入时核对尺寸并转换为灰度。
cv::Mat Vision_draw(const cv::Mat &bgr, const std::vector<Detection> &detections, const cv::Mat &chineseLabels);

//---------NPU接口区---------
// 模块内只保存一个模型。主流程统一调用 Yolo_close，包括打开或运行失败时。
bool Yolo_open(std::string modelPath);
// 先成功打开模型，输入必须来自 Vision_prepare；失败返回 nullptr。
// 输出数据在下一次推理或关闭模型前有效。
const YoloHead *Yolo_run(const cv::Mat &rgb640);
void Yolo_close();
} // namespace scara
