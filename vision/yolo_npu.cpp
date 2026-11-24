//管理RKNN模型和输入输出缓冲，执行逐帧推理
#include "yolo.hpp"
#include <rknn_api.h>
#include <cstring>
#include <stdexcept>

namespace scara
{
//---------模型与缓冲数据区---------
// 模型只保留逐帧推理会用到的句柄、缓冲和读取格式。
static struct
{
    //SDK返回的模型句柄，0表示当前没有模型上下文
    rknn_context context = 0;
    size_t inputRowBytes = 0; // RGB 输入相邻两行的实际字节间距。
    //缓冲0是输入，缓冲1～3是三路输出
    rknn_tensor_mem *memory[1 + YoloHeads]{};
    //交给后处理的三路输出地址与读取格式
    YoloHead heads[YoloHeads];
} Model;

static bool prepareTensor(int slot);

//---------模型初始化区---------
bool Yolo_open(std::string path)
{
    Yolo_close();
    // 从模型文件创建上下文，再按输入、输出的顺序准备缓冲。
    bool ready = rknn_init(&Model.context, path.data(), 0, 0, nullptr) == 0;
    for (int slot = 0; ready && slot < 1 + YoloHeads; ++slot)
        ready = prepareTensor(slot);
    return ready;
}

//---------单帧推理区---------
void Yolo_run(const cv::Mat &rgb, YoloHead heads[YoloHeads])
{
    //取得NPU输入缓冲的CPU映射地址
    uint8_t *input = static_cast<uint8_t *>(Model.memory[0]->virt_addr);
    // 每行复制 640×3 个有效字节，目标行按实际字节间距跨过对齐区。
    for (int row = 0; row < YoloSize; ++row)
        std::memcpy(input + row * Model.inputRowBytes, rgb.ptr<uint8_t>(row), YoloSize * 3);
    //执行一次同步推理，结果写入已经绑定的三路输出缓冲
    int status = rknn_run(Model.context, nullptr);
    if (status != 0) throw std::runtime_error("NPU inference failed");
    //交付输出地址和格式描述，后处理直接读取对应缓冲
    for (int i = 0; i < YoloHeads; ++i)
        heads[i] = Model.heads[i];
}

//---------模型关闭区---------
void Yolo_close()
{
    //先释放已经申请的输入输出内存
    for (rknn_tensor_mem *memory : Model.memory)
        if (memory) rknn_destroy_mem(Model.context, memory);
    //内存释放后再销毁模型上下文
    if (Model.context) rknn_destroy(Model.context);
    //清空句柄、指针和输出描述，供后续重新打开模型
    Model = {};
}
//---------张量准备区---------
//查询并绑定一块缓冲，slot为Model中的缓冲编号
static bool prepareTensor(int slot)
{
    // 第 0 块是 RGB 输入，第 1～3 块按模型的网格顺序对应三路输出。
    bool input = slot == 0;
    unsigned index = input ? 0 : slot - 1;
    unsigned width = input ? YoloSize : YoloGrid[index];
    unsigned channels = input ? 3 : YoloChannels;
    rknn_query_cmd query = input ? RKNN_QUERY_NATIVE_INPUT_ATTR : RKNN_QUERY_NATIVE_NHWC_OUTPUT_ATTR;

    // SDK 属性仅在查询和绑定此缓冲时使用。
    rknn_tensor_attr attr{};
    attr.index = index;
    if (rknn_query(Model.context, query, &attr, sizeof(attr)) != 0) return false;

    //w_stride为含对齐的每行宽度，返回0时采用有效图像宽度
    size_t rowWidth = attr.w_stride ? attr.w_stride : width;
    //核对单批次NHWC布局以及固定模型的高、宽和通道数
    bool layout = attr.n_dims == 4 && attr.dims[0] == 1 && attr.dims[1] == width && attr.dims[2] == width &&
                  attr.dims[3] == channels && attr.fmt == RKNN_TENSOR_NHWC;
    //按最后一行有效数据的终点核对缓冲容量，单位为字节
    bool capacity =
        rowWidth >= width && ((width - 1) * rowWidth + width) * channels <= attr.size_with_stride;
    //后处理要求输出为INT8，并使用正的反量化系数
    bool quantized = input || (attr.type == RKNN_TENSOR_INT8 && attr.scale > 0);
    if (!layout || !capacity || !quantized) return false;

    //指定缓冲数据类型，输入由SDK按type和fmt完成模型所需的转换
    attr.type = input ? RKNN_TENSOR_UINT8 : RKNN_TENSOR_INT8;
    attr.pass_through = 0;
    //按包含对齐空间的总字节数申请NPU内存
    Model.memory[slot] = rknn_create_mem(Model.context, attr.size_with_stride);
    rknn_tensor_mem *memory = Model.memory[slot];
    //确认内存映射地址和实际容量可用
    if (!memory || !memory->virt_addr || memory->size < attr.size_with_stride) return false;

    // 先清零有效数据和对齐空间，再把缓冲交给 SDK。
    std::memset(memory->virt_addr, 0, memory->size);
    if (rknn_set_io_mem(Model.context, memory, &attr) != 0) return false;
    if (input) Model.inputRowBytes = (attr.w_stride ? attr.w_stride : width) * channels;
    else Model.heads[index] = {static_cast<const int8_t *>(memory->virt_addr),
                               rowWidth * channels, attr.zp, attr.scale};
    return true;
}

} // namespace scara
