#include "yolo.hpp"
#include <rknn_api.h>
#include <cstring>
#include <stdexcept>

namespace scara
{
struct TensorSpec
{
    rknn_query_cmd query;
    unsigned index;
    unsigned width;
    unsigned channels;
    rknn_tensor_type type;
};

// 输入和三路输出按固定次序准备；每路矩阵的高、宽相同。
static const TensorSpec Tensors[] = {{RKNN_QUERY_NATIVE_INPUT_ATTR, 0, YoloSize, 3, RKNN_TENSOR_UINT8},
                                     {RKNN_QUERY_NATIVE_NHWC_OUTPUT_ATTR, 0, 80, YoloChannels, RKNN_TENSOR_INT8},
                                     {RKNN_QUERY_NATIVE_NHWC_OUTPUT_ATTR, 1, 40, YoloChannels, RKNN_TENSOR_INT8},
                                     {RKNN_QUERY_NATIVE_NHWC_OUTPUT_ATTR, 2, 20, YoloChannels, RKNN_TENSOR_INT8}};

// 缓冲0是输入，缓冲1～3是输出；SDK输出编号从0开始。
static struct
{
    rknn_context context = 0;
    rknn_tensor_attr attr[4]{};
    rknn_tensor_mem *memory[4]{};
    YoloHead heads[YoloHeads];
} Model;

static bool prepareTensor(int slot)
{
    const TensorSpec &spec = Tensors[slot];
    rknn_tensor_attr &attr = Model.attr[slot];
    attr.index = spec.index;
    int status = rknn_query(Model.context, spec.query, &attr, sizeof(attr));
    if (status != 0) return false;

    size_t rowWidth = attr.w_stride ? attr.w_stride : spec.width;
    bool layout = attr.n_dims == 4 && attr.dims[0] == 1 && attr.dims[1] == spec.width && attr.dims[2] == spec.width &&
                  attr.dims[3] == spec.channels && attr.fmt == RKNN_TENSOR_NHWC;
    bool capacity =
        rowWidth >= spec.width && ((spec.width - 1) * rowWidth + spec.width) * spec.channels <= attr.size_with_stride;
    bool quantized = slot == 0 || (attr.type == RKNN_TENSOR_INT8 && attr.scale > 0);
    if (layout == false || capacity == false || quantized == false) return false;

    attr.type = spec.type;
    attr.pass_through = 0;
    Model.memory[slot] = rknn_create_mem(Model.context, attr.size_with_stride);
    rknn_tensor_mem *memory = Model.memory[slot];
    bool allocated = memory && memory->virt_addr && memory->size >= attr.size_with_stride;
    if (allocated)
    {
        std::memset(memory->virt_addr, 0, memory->size);
        status = rknn_set_io_mem(Model.context, memory, &attr);
        if (slot > 0)
        {
            YoloHead &head = Model.heads[slot - 1];
            head.data = static_cast<const int8_t *>(memory->virt_addr);
            head.rowBytes = rowWidth * YoloChannels;
            head.zeroPoint = attr.zp;
            head.scale = attr.scale;
        }
    }
    return allocated && status == 0;
}

bool Yolo_open(std::string path)
{
    Yolo_close();
    int status = rknn_init(&Model.context, path.data(), 0, 0, nullptr);
    bool ready = status == 0;
    int slot = 0;
    while (ready && slot < 4)
    {
        ready = prepareTensor(slot);
        ++slot;
    }
    return ready;
}

void Yolo_run(const cv::Mat &rgb, YoloHead heads[YoloHeads])
{
    uint8_t *input = static_cast<uint8_t *>(Model.memory[0]->virt_addr);
    size_t rowBytes = (Model.attr[0].w_stride ? Model.attr[0].w_stride : YoloSize) * 3;
    for (int row = 0; row < YoloSize; ++row)
        std::memcpy(input + row * rowBytes, rgb.ptr<uint8_t>(row), YoloSize * 3);
    int status = rknn_run(Model.context, nullptr);
    if (status != 0) throw std::runtime_error("NPU inference failed");
    for (int i = 0; i < YoloHeads; ++i)
        heads[i] = Model.heads[i]; // 复制地址和格式描述，不复制整块输出数据。
}

void Yolo_close()
{
    for (rknn_tensor_mem *memory : Model.memory)
        if (memory) rknn_destroy_mem(Model.context, memory);
    if (Model.context) rknn_destroy(Model.context);
    Model = {};
}
} // namespace scara
