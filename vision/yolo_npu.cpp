#include "yolo.hpp"
#include <rknn_api.h>
#include <cstring>

namespace scara
{
// 缓冲0是输入，缓冲1～3是输出；SDK输出编号从0开始。
static struct
{
    rknn_context context = 0;
    rknn_tensor_attr attr[4]{};
    rknn_tensor_mem *memory[4]{};
    YoloHead heads[YoloHeads];
} Model;

bool Yolo_open(std::string path)
{
    Yolo_close();
    if (rknn_init(&Model.context, path.data(), 0, 0, nullptr) != 0) return false;
    for (int i = 0; i < 4; ++i)
    {
        rknn_tensor_attr &attr = Model.attr[i];
        attr.index = i ? i - 1 : 0;
        rknn_query_cmd query = i ? RKNN_QUERY_NATIVE_NHWC_OUTPUT_ATTR : RKNN_QUERY_NATIVE_INPUT_ATTR;
        if (rknn_query(Model.context, query, &attr, sizeof(attr)) != 0) return false;
        int width = i ? YoloGrid[i - 1] : YoloSize;
        int channels = i ? YoloChannels : 3;
        size_t rowWidth = attr.w_stride ? attr.w_stride : width;
        if (attr.n_dims != 4 || attr.dims[0] != 1 || attr.dims[1] != unsigned(width) ||
            attr.dims[2] != unsigned(width) || attr.dims[3] != unsigned(channels) || attr.fmt != RKNN_TENSOR_NHWC ||
            rowWidth < unsigned(width) || ((width - 1) * rowWidth + width) * channels > attr.size_with_stride)
            return false;
        if (i && (attr.type != RKNN_TENSOR_INT8 || !(attr.scale > 0))) return false;
        attr.type = i ? RKNN_TENSOR_INT8 : RKNN_TENSOR_UINT8;
        attr.pass_through = 0;
        Model.memory[i] = rknn_create_mem(Model.context, attr.size_with_stride);
        rknn_tensor_mem *memory = Model.memory[i];
        if (!memory || !memory->virt_addr || memory->size < attr.size_with_stride) return false;
        std::memset(memory->virt_addr, 0, memory->size);
        if (rknn_set_io_mem(Model.context, memory, &attr) != 0) return false;
        if (i)
            Model.heads[i - 1] = {static_cast<const int8_t *>(memory->virt_addr), rowWidth * YoloChannels, attr.zp,
                                  attr.scale};
    }
    return true;
}

const YoloHead *Yolo_run(const cv::Mat &rgb)
{
    uint8_t *input = static_cast<uint8_t *>(Model.memory[0]->virt_addr);
    size_t rowBytes = (Model.attr[0].w_stride ? Model.attr[0].w_stride : YoloSize) * 3;
    for (int row = 0; row < YoloSize; ++row)
        std::memcpy(input + row * rowBytes, rgb.ptr<uint8_t>(row), YoloSize * 3);
    return rknn_run(Model.context, nullptr) == 0 ? Model.heads : nullptr;
}

void Yolo_close()
{
    for (rknn_tensor_mem *memory : Model.memory)
        if (memory) rknn_destroy_mem(Model.context, memory);
    if (Model.context) rknn_destroy(Model.context);
    Model = {};
}
} // namespace scara
