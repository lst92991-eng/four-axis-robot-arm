// 固定模型初始化：一个 RGB 输入，三个 INT8 输出。
#include "yolo.hpp"
#include <rknn_api.h>
#include <cmath>
#include <cstring>
#include <stdexcept>

namespace scara
{
//---------模型运行数据---------
static struct
{
    rknn_context context = 0;
    rknn_tensor_attr input{};
    rknn_tensor_attr output[YoloHeads]{};
    rknn_tensor_mem *inputMemory = nullptr;
    rknn_tensor_mem *outputMemory[YoloHeads]{};
    YoloHead heads[YoloHeads];
} Model;

static void check(int status, const char *operation);
static bool validLayout(const rknn_tensor_attr &attr, int width, int channels);
static void bindBuffer(rknn_tensor_mem *memory, rknn_tensor_attr &attr);

//---------加载模型与准备缓冲---------
void Yolo_open(std::string path)
{
    Yolo_close();
    check(rknn_init(&Model.context, path.data(), 0, 0, nullptr), "rknn_init");
    rknn_input_output_num count{};
    check(rknn_query(Model.context, RKNN_QUERY_IN_OUT_NUM, &count, sizeof(count)), "rknn_query(count)");
    if (count.n_input != 1 || count.n_output != YoloHeads)
        throw std::runtime_error("Expected one input and three outputs");

    // 输入为 640×640 RGB 图像，归一化由模型完成。
    check(rknn_query(Model.context, RKNN_QUERY_NATIVE_INPUT_ATTR, &Model.input, sizeof(Model.input)),
          "rknn_query(input)");
    if (!validLayout(Model.input, YoloSize, 3)) throw std::runtime_error("Input layout mismatch");
    Model.input.type = RKNN_TENSOR_UINT8;
    Model.input.pass_through = 0;
    Model.inputMemory = rknn_create_mem(Model.context, Model.input.size_with_stride);
    bindBuffer(Model.inputMemory, Model.input);

    // 三路输出大小为 80×80、40×40、20×20，每格包含 57 个数。
    for (int i = 0; i < YoloHeads; ++i)
    {
        rknn_tensor_attr &attr = Model.output[i];
        attr.index = i;
        check(rknn_query(Model.context, RKNN_QUERY_NATIVE_NHWC_OUTPUT_ATTR, &attr, sizeof(attr)), "rknn_query(output)");
        if (!validLayout(attr, YoloGrid[i], YoloChannels) || attr.type != RKNN_TENSOR_INT8 ||
            attr.qnt_type != RKNN_TENSOR_QNT_AFFINE_ASYMMETRIC || !std::isfinite(attr.scale) || attr.scale <= 0)
            throw std::runtime_error("Output layout mismatch");
        Model.outputMemory[i] = rknn_create_mem(Model.context, attr.size_with_stride);
        bindBuffer(Model.outputMemory[i], attr);
        rknn_tensor_mem *memory = Model.outputMemory[i];
        YoloHead &head = Model.heads[i];
        head.data = static_cast<const int8_t *>(memory->virt_addr);
        head.rowBytes = (attr.w_stride ? attr.w_stride : attr.dims[2]) * YoloChannels;
        head.zeroPoint = attr.zp;
        head.scale = attr.scale;
    }
}

//---------送入图像并执行推理---------
const YoloHead *Yolo_run(const cv::Mat &rgb)
{
    if (!Model.context || rgb.type() != CV_8UC3 || rgb.size() != cv::Size(YoloSize, YoloSize))
        throw std::runtime_error("Expected an open model and 640x640 RGB input");
    uint8_t *input = static_cast<uint8_t *>(Model.inputMemory->virt_addr);
    size_t rowBytes = (Model.input.w_stride ? Model.input.w_stride : YoloSize) * 3;
    for (int row = 0; row < YoloSize; ++row)
        std::memcpy(input + row * rowBytes, rgb.ptr<uint8_t>(row), YoloSize * 3);
    check(rknn_run(Model.context, nullptr), "rknn_run");
    return Model.heads;
}

//---------释放模型---------
void Yolo_close()
{
    for (rknn_tensor_mem *memory : Model.outputMemory)
        if (memory) rknn_destroy_mem(Model.context, memory);
    if (Model.inputMemory) rknn_destroy_mem(Model.context, Model.inputMemory);
    if (Model.context) rknn_destroy(Model.context);
    Model = {};
}

//---------缓冲区检查与绑定---------
static bool validLayout(const rknn_tensor_attr &attr, int width, int channels)
{
    if (attr.n_dims != 4 || attr.fmt != RKNN_TENSOR_NHWC) return false;
    if (attr.dims[0] != 1 || attr.dims[1] != unsigned(width) || attr.dims[2] != unsigned(width) ||
        attr.dims[3] != unsigned(channels))
        return false;
    size_t rowWidth = attr.w_stride ? attr.w_stride : width;
    // 前面的行包含对齐间距，最后一行只需容纳有效数据。
    size_t requiredBytes = ((width - 1) * rowWidth + width) * channels;
    return rowWidth >= unsigned(width) && requiredBytes <= attr.size_with_stride;
}

static void bindBuffer(rknn_tensor_mem *memory, rknn_tensor_attr &attr)
{
    if (!memory || !memory->virt_addr || memory->size < attr.size_with_stride)
        throw std::runtime_error("Cannot allocate tensor memory");
    std::memset(memory->virt_addr, 0, memory->size);
    check(rknn_set_io_mem(Model.context, memory, &attr), "rknn_set_io_mem");
}

static void check(int status, const char *operation)
{
    if (status != 0) throw std::runtime_error(std::string(operation) + " failed: " + std::to_string(status));
}
} // namespace scara
