// RV1106 模型的输入输出缓冲，由模型配套代码按 SDK 的原生格式绑定。
#pragma once
#include "postprocess.hpp"
#include <rknn_api.h>

namespace scara
{
struct RknnBuffers
{
    rknn_tensor_mem *memory[1 + YoloHeads]{}; // 输入图在第 0 块，其余三块为输出。
    cv::Mat input; // 映射到第 0 块内存的 RGB 图，预先设好行距。
    YoloHead heads[YoloHeads]{};
};

bool bindModelBuffers(rknn_context context, RknnBuffers &buffers);
}
