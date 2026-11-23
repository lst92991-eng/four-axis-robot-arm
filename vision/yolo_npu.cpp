//管理RKNN模型和输入输出缓冲，执行逐帧推理
#include "yolo.hpp"
#include <rknn_api.h>
#include <cstring>
#include <stdexcept>

namespace scara
{
//---------模型与缓冲数据区---------
//约定每路张量的查询方式、尺寸和绑定数据类型
struct TensorSpec
{
    //查询输入属性或原生NHWC输出属性的SDK命令
    rknn_query_cmd query;
    //SDK中的张量编号，输入为0，三路输出为0～2
    unsigned index;
    //正方形张量的边长，输入为640，输出为80、40、20
    unsigned width;
    //每个位置的通道数，RGB输入为3，模型输出为57
    unsigned channels;
    //绑定内存时使用的类型，输入UINT8，输出INT8
    rknn_tensor_type type;
};

//按输入、输出0、输出1、输出2的顺序准备四块缓冲
static const TensorSpec Tensors[] = {
    {RKNN_QUERY_NATIVE_INPUT_ATTR, 0, YoloSize, 3, RKNN_TENSOR_UINT8},
    {RKNN_QUERY_NATIVE_NHWC_OUTPUT_ATTR, 0, YoloGrid[0], YoloChannels, RKNN_TENSOR_INT8},
    {RKNN_QUERY_NATIVE_NHWC_OUTPUT_ATTR, 1, YoloGrid[1], YoloChannels, RKNN_TENSOR_INT8},
    {RKNN_QUERY_NATIVE_NHWC_OUTPUT_ATTR, 2, YoloGrid[2], YoloChannels, RKNN_TENSOR_INT8}};

//一个模型持有上下文、张量属性及对应的NPU内存
static struct
{
    //SDK返回的模型句柄，0表示当前没有模型上下文
    rknn_context context = 0;
    //四路张量的形状、布局、行跨度和量化参数
    rknn_tensor_attr attr[1 + YoloHeads]{};
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
    //RGB每个像素占3字节，行间距包含NPU要求的对齐空间
    size_t rowBytes = (Model.attr[0].w_stride ? Model.attr[0].w_stride : YoloSize) * 3;
    // 每行从 OpenCV 复制 640×3 个有效字节，目标行按 rowBytes 跨过对齐区。
    for (int row = 0; row < YoloSize; ++row)
        std::memcpy(input + row * rowBytes, rgb.ptr<uint8_t>(row), YoloSize * 3);
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
    //取得该缓冲的固定要求和用于接收SDK结果的属性结构
    const TensorSpec &spec = Tensors[slot];
    rknn_tensor_attr &attr = Model.attr[slot];
    //按SDK张量编号查询原生布局
    attr.index = spec.index;
    if (rknn_query(Model.context, spec.query, &attr, sizeof(attr)) != 0) return false;

    //w_stride为含对齐的每行宽度，返回0时采用有效图像宽度
    size_t rowWidth = attr.w_stride ? attr.w_stride : spec.width;
    //核对单批次NHWC布局以及固定模型的高、宽和通道数
    bool layout = attr.n_dims == 4 && attr.dims[0] == 1 && attr.dims[1] == spec.width && attr.dims[2] == spec.width &&
                  attr.dims[3] == spec.channels && attr.fmt == RKNN_TENSOR_NHWC;
    //按最后一行有效数据的终点核对缓冲容量，单位为字节
    bool capacity =
        rowWidth >= spec.width && ((spec.width - 1) * rowWidth + spec.width) * spec.channels <= attr.size_with_stride;
    //后处理要求输出为INT8，并使用正的反量化系数
    bool quantized = slot == 0 || (attr.type == RKNN_TENSOR_INT8 && attr.scale > 0);
    if (!layout || !capacity || !quantized) return false;

    //指定缓冲数据类型，输入由SDK按type和fmt完成模型所需的转换
    attr.type = spec.type;
    attr.pass_through = 0;
    //按包含对齐空间的总字节数申请NPU内存
    Model.memory[slot] = rknn_create_mem(Model.context, attr.size_with_stride);
    rknn_tensor_mem *memory = Model.memory[slot];
    //确认内存映射地址和实际容量可用
    if (!memory || !memory->virt_addr || memory->size < attr.size_with_stride) return false;

    // 先清零有效数据和对齐空间，再把缓冲交给 SDK。
    std::memset(memory->virt_addr, 0, memory->size);
    if (rknn_set_io_mem(Model.context, memory, &attr) != 0) return false;
    // 第 0 块是输入。其余缓冲的读取格式交给后处理。
    if (slot > 0)
    {
        YoloHead &head = Model.heads[slot - 1];
        head.data = static_cast<const int8_t *>(memory->virt_addr);
        head.rowBytes = rowWidth * YoloChannels;
        head.zeroPoint = attr.zp;
        head.scale = attr.scale;
    }
    return true;
}

} // namespace scara
