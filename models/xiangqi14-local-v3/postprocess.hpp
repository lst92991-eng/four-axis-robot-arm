// v3 模型内部接口，板端推理、台架及张量回放共用。
#pragma once
#include "vision/yolo.hpp"
#include <cstddef>
#include <cstdint>

namespace scara
{
//每个网格点使用3个锚框，模型共有3路不同尺度的输出
constexpr int YoloAnchors = 3;
// 模型分别输出 80、40、20 三种网格尺度。
constexpr int YoloHeads = 3;
//每个候选框保存4个位置数、1个目标分数和14个类别分数
constexpr int YoloBoxValues = 5 + YoloClasses;
//一个网格点连续保存3个候选框，共57个通道值
constexpr int YoloChannels = YoloAnchors * YoloBoxValues;
//三路输出的网格边长，对应输入图中的8、16、32像素步长
constexpr int YoloGrid[YoloHeads] = {80, 40, 20};

//一路NPU输出的读取信息，NHWC布局按行、列、通道排列
struct YoloHead
{
    //INT8输出数据首地址，内存由NPU模块管理
    const int8_t *data = nullptr;
    //相邻两行的实际字节间距，包含硬件对齐区域
    size_t rowBytes = 0;
    //量化零点，表示真实数值0对应的量化整数
    int zeroPoint = 0;
    //反量化系数，真实数值等于(量化整数-zeroPoint)乘以scale
    float scale = 0;
};

// 用本帧的裁剪起点，把三路模型输出还原成原图上的棋子框。
std::vector<Detection> Vision_decode(const YoloHead heads[YoloHeads], cv::Point origin);
} // namespace scara
