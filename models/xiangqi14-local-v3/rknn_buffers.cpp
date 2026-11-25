#include "rknn_buffers.hpp"
#include <cstring>

namespace scara
{
//---------绑定 RKNN 内存区---------
// 成功返回相邻两行的字节数；0 表示这块内存无法使用，已申请部分统一由 Yolo_close 回收。
static size_t bindMemory(rknn_context context, rknn_tensor_mem *&buffer, rknn_tensor_attr &attr, unsigned side, unsigned channels)
{
    const uint32_t shape[] = {1, side, side, channels};
    if (attr.fmt != RKNN_TENSOR_NHWC || attr.n_dims != 4 || std::memcmp(attr.dims, shape, sizeof(shape)) != 0)
        return 0;

    // SDK 的 w_stride 为 0 表示各行紧挨。用 64 位计算最后一行的末端，避免板端乘法溢出。
    uint64_t rowBytes = uint64_t(attr.w_stride ? attr.w_stride : side) * channels;
    unsigned usedBytes = side * channels;
    if (rowBytes < usedBytes || (side - 1) * rowBytes + usedBytes > attr.size_with_stride) return 0;

    buffer = rknn_create_mem(context, attr.size_with_stride);
    if (!buffer || !buffer->virt_addr || buffer->size < attr.size_with_stride) return 0;
    std::memset(buffer->virt_addr, 0, buffer->size);
    attr.pass_through = 0;
    if (rknn_set_io_mem(context, buffer, &attr) != 0) return 0;
    return size_t(rowBytes);
}

bool bindModelBuffers(rknn_context context, RknnBuffers &buffers)
{
    // 输入是一张 640×640 RGB 图，由 SDK 将 UINT8 像素转换成模型输入。
    rknn_tensor_attr input{};
    if (rknn_query(context, RKNN_QUERY_NATIVE_INPUT_ATTR, &input, sizeof(input)) != 0) return false;
    input.type = RKNN_TENSOR_UINT8;
    size_t inputRowBytes = bindMemory(context, buffers.memory[0], input, YoloSize, 3);
    if (inputRowBytes == 0) return false;
    buffers.input = cv::Mat(YoloSize, YoloSize, CV_8UC3, buffers.memory[0]->virt_addr, inputRowBytes);

    // 每路输出单独查询。后处理使用这一路的实际行距、零点和量化比例。
    for (unsigned i = 0; i < YoloHeads; ++i)
    {
        rknn_tensor_attr output{};
        output.index = i;
        if (rknn_query(context, RKNN_QUERY_NATIVE_NHWC_OUTPUT_ATTR, &output, sizeof(output)) != 0) return false;
        if (output.type != RKNN_TENSOR_INT8 || !(output.scale > 0)) return false;
        auto &buffer = buffers.memory[i + 1];
        size_t rowBytes = bindMemory(context, buffer, output, YoloGrid[i], YoloChannels);
        if (rowBytes == 0) return false;
        buffers.heads[i] = {static_cast<const int8_t *>(buffer->virt_addr), rowBytes, output.zp, output.scale};
    }
    return true;
}
} // namespace scara
