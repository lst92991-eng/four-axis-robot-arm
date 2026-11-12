// 固定模型初始化：一个 RGB 输入，三个 INT8 输出。
#include "yolo.hpp"
#include <rknn_api.h>
#include <cmath>
#include <cstring>
#include <stdexcept>

namespace scara {
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

static void require(bool valid, const char *message);
static bool validLayout(const rknn_tensor_attr& attr, int width, int channels);
static void bindBuffer(rknn_tensor_mem *memory, rknn_tensor_attr& attr);

//---------加载模型与准备缓冲---------
void Yolo_open(std::string path)
{
  Yolo_close();
  try
  {
    require(rknn_init(&Model.context, path.data(), 0, 0, nullptr)==0, "Open model failed");
    rknn_input_output_num count{};
    require(rknn_query(Model.context, RKNN_QUERY_IN_OUT_NUM, &count, sizeof(count))==0,
            "Read tensor count failed");
    require(count.n_input==1 && count.n_output==YoloHeads, "Expected one input and three outputs");

    // 输入为 640×640 RGB 图像，归一化由模型完成。
    require(rknn_query(Model.context, RKNN_QUERY_NATIVE_INPUT_ATTR, &Model.input, sizeof(Model.input))==0,
            "Read input failed");
    require(validLayout(Model.input,YoloSize,3), "Input layout mismatch");
    Model.input.type = RKNN_TENSOR_UINT8;
    Model.input.pass_through = 0;
    Model.inputMemory = rknn_create_mem(Model.context, Model.input.size_with_stride);
    bindBuffer(Model.inputMemory, Model.input);

    // 三路输出大小为 80×80、40×40、20×20，每格包含 57 个数。
    for (int i=0; i<YoloHeads; ++i)
    {
      auto& attr = Model.output[i];
      attr.index = i;
      require(rknn_query(Model.context, RKNN_QUERY_NATIVE_NHWC_OUTPUT_ATTR, &attr, sizeof(attr))==0,
              "Read output failed");
      require(validLayout(attr,YoloGrid[i],YoloChannels) && attr.type==RKNN_TENSOR_INT8 &&
              attr.qnt_type==RKNN_TENSOR_QNT_AFFINE_ASYMMETRIC && std::isfinite(attr.scale) && attr.scale>0,
              "Output layout mismatch");
      Model.outputMemory[i] = rknn_create_mem(Model.context, attr.size_with_stride);
      bindBuffer(Model.outputMemory[i], attr);
      auto *memory = Model.outputMemory[i];
      size_t rowBytes = (attr.w_stride ? attr.w_stride : attr.dims[2])*YoloChannels;
      Model.heads[i] = {static_cast<const int8_t *>(memory->virt_addr), memory->size, rowBytes, attr.zp, attr.scale};
    }
  }
  catch (...)
  {
    Yolo_close();
    throw;
  }
}

//---------送入图像并执行推理---------
const YoloHead *Yolo_run(const cv::Mat& rgb)
{
  require(Model.context && rgb.type()==CV_8UC3 && rgb.size()==cv::Size(YoloSize,YoloSize), "Expected 640x640 RGB input");
  auto *input = static_cast<uint8_t *>(Model.inputMemory->virt_addr);
  size_t rowBytes = (Model.input.w_stride ? Model.input.w_stride : YoloSize)*3;
  for (int row=0; row<YoloSize; ++row)
    std::memcpy(input+row*rowBytes, rgb.ptr(row), YoloSize*3);
  require(rknn_run(Model.context, nullptr)==0, "Inference failed");
  return Model.heads;
}

//---------释放模型---------
void Yolo_close()
{
  for (auto *memory : Model.outputMemory)
    if (memory) rknn_destroy_mem(Model.context, memory);
  if (Model.inputMemory) rknn_destroy_mem(Model.context, Model.inputMemory);
  if (Model.context) rknn_destroy(Model.context);
  Model = {};
}

//---------缓冲区检查与绑定---------
static bool validLayout(const rknn_tensor_attr& attr, int width, int channels)
{
  if (attr.n_dims!=4 || attr.fmt!=RKNN_TENSOR_NHWC) return false;
  if (attr.dims[0]!=1 || attr.dims[1]!=unsigned(width) ||
      attr.dims[2]!=unsigned(width) || attr.dims[3]!=unsigned(channels)) return false;
  size_t rowWidth = attr.w_stride ? attr.w_stride : width;
  return rowWidth>=unsigned(width) && ((width-1)*rowWidth+width)*channels<=attr.size_with_stride;
}

static void bindBuffer(rknn_tensor_mem *memory, rknn_tensor_attr& attr)
{
  require(memory && memory->virt_addr && memory->size>=attr.size_with_stride, "Cannot allocate tensor memory");
  std::memset(memory->virt_addr, 0, memory->size);
  require(rknn_set_io_mem(Model.context, memory, &attr)==0, "Bind tensor failed");
}

static void require(bool valid, const char *message)
{
  if (!valid) throw std::runtime_error(message);
}
}
