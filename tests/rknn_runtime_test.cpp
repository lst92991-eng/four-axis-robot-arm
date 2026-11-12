// 模拟 RKNN C ABI，验证本项目适配层的步幅、失败清理和退出。
#include "vision/yolo.hpp"
#include <rknn_api.h>
#include <array>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <stdexcept>

namespace {
int allocations = 0, alive = 0, failAt = 0;
int failQuery = -1, failBind = 0, bindings = 0;
bool contextOpen = false, badOutput = false, failRun = false;
rknn_tensor_mem *input = nullptr;
std::array<rknn_tensor_mem *, 3> outputs{};

void check(bool value, const char *message)
{
  if (!value) throw std::runtime_error(message);
}
}

extern "C" {
int rknn_init(rknn_context *context, void *, uint32_t, uint32_t, rknn_init_extend *)
{
#ifdef RKNN_MOCK_ENTRY
  std::cerr << "TEST ONLY: simulated RKNN ABI, empty model outputs\n";
#endif
  *context = 1;
  contextOpen = true;
  allocations = 0;
  bindings = 0;
  return 0;
}

int rknn_query(rknn_context, rknn_query_cmd command, void *data, uint32_t)
{
  if (command==failQuery) return -21;
  if (command==RKNN_QUERY_IN_OUT_NUM)
  {
    *static_cast<rknn_input_output_num *>(data) = {1,3};
    return 0;
  }
  auto& attr = *static_cast<rknn_tensor_attr *>(data);
  bool isInput = command==RKNN_QUERY_NATIVE_INPUT_ATTR;
  int size = isInput ? 640 : 80 >> attr.index;
  attr.n_dims = 4;
  attr.dims[0] = 1;
  attr.dims[1] = attr.dims[2] = size;
  attr.dims[3] = isInput ? 3 : 57;
  attr.fmt = RKNN_TENSOR_NHWC;
  attr.type = badOutput && !isInput ? RKNN_TENSOR_FLOAT32 : RKNN_TENSOR_INT8;
  attr.qnt_type = RKNN_TENSOR_QNT_AFFINE_ASYMMETRIC;
  attr.w_stride = size+2;
  attr.n_elems = size*size*attr.dims[3];
  attr.size_with_stride = size*(size+2)*attr.dims[3]+32;
  attr.scale = 1.f/255;
  attr.zp = -128;
  return 0;
}

rknn_tensor_mem *rknn_create_mem(rknn_context, uint32_t size)
{
  if (++allocations==failAt) return nullptr;
  auto *memory = new rknn_tensor_mem{};
  memory->virt_addr = std::calloc(size,1);
  memory->size = size;
  ++alive;
  return memory;
}

int rknn_set_io_mem(rknn_context, rknn_tensor_mem *memory, rknn_tensor_attr *attr)
{
  if (++bindings==failBind) return -22;
  if (attr->type==RKNN_TENSOR_UINT8) input = memory;
  else outputs[attr->index] = memory;
  return 0;
}

int rknn_run(rknn_context, rknn_run_extend *)
{
  if (failRun) return -7;
#ifndef RKNN_MOCK_ENTRY
  const auto *bytes = static_cast<const uint8_t *>(input->virt_addr);
  for (int row=0; row<640; ++row)
  {
    for (int x=0; x<640; ++x)
    {
      size_t offset = (row*642+x)*3;
      check(bytes[offset]==11 && bytes[offset+1]==22 && bytes[offset+2]==33,
            "Input rows were copied with the wrong stride");
    }
  }
#endif
  for (auto *output : outputs) std::memset(output->virt_addr, 128, output->size);
#ifndef RKNN_MOCK_ENTRY
  auto *p = static_cast<int8_t *>(outputs[0]->virt_addr)+(20*82+20)*57;
  p[0] = p[1] = p[2] = p[3] = 0;
  p[4] = p[5+12] = 127;
#endif
  return 0;
}

int rknn_destroy_mem(rknn_context, rknn_tensor_mem *memory)
{
  std::free(memory->virt_addr);
  delete memory;
  --alive;
  return 0;
}

int rknn_destroy(rknn_context)
{
  contextOpen = false;
  return 0;
}
}

#ifndef RKNN_MOCK_ENTRY
int main()
{
  try
  {
    for (failAt=1; failAt<=4; ++failAt)
    {
      bool rejected = false;
      try { scara::Yolo_open("mock.rknn"); scara::Yolo_close(); }
      catch (const std::exception&) { rejected = true; }
      check(rejected && !contextOpen && !alive, "Partial allocation leaked resources");
    }
    failAt = 0;
    for (int command : {RKNN_QUERY_IN_OUT_NUM, RKNN_QUERY_NATIVE_INPUT_ATTR, RKNN_QUERY_NATIVE_NHWC_OUTPUT_ATTR})
    {
      failQuery = command;
      bool reported = false;
      try { scara::Yolo_open("mock.rknn"); }
      catch (const std::exception& e) { reported = std::string(e.what()).find("-21")!=std::string::npos; }
      check(reported && !contextOpen && !alive, "Query failure lost status or leaked resources");
    }
    failQuery = -1;
    for (failBind=1; failBind<=4; ++failBind)
    {
      bool reported = false;
      try { scara::Yolo_open("mock.rknn"); }
      catch (const std::exception& e) { reported = std::string(e.what()).find("-22")!=std::string::npos; }
      check(reported && !contextOpen && !alive, "Binding failure lost status or leaked resources");
    }
    failBind = 0;
    badOutput = true;
    bool rejected = false;
    try { scara::Yolo_open("mock.rknn"); scara::Yolo_close(); }
    catch (const std::exception&) { rejected = true; }
    check(rejected && !contextOpen && !alive, "Output layout failure leaked resources");
    badOutput = false;
    scara::Yolo_open("mock.rknn");
    scara::Yolo_open("mock.rknn");
    check(contextOpen && alive==4, "Reopening the fixed model leaked its previous buffers");
    cv::Mat rgb(640,640,CV_8UC3,cv::Scalar(11,22,33));
    auto heads = scara::Yolo_run(rgb);
    auto pieces = scara::Vision_decode(heads, {{0,0,640,640},1,0,0}, {});
    check(pieces.size()==1 && pieces[0].classId==12, "Native padded output was decoded incorrectly");
    failRun = true;
    rejected = false;
    try { scara::Yolo_run(rgb); }
    catch (const std::exception& e) { rejected = std::string(e.what()).find("rknn_run failed: -7")!=std::string::npos; }
    scara::Yolo_close();
    check(rejected && !contextOpen && !alive, "Inference failure cleanup failed");
    std::cout << "PASS RKNN adapter input/output strides and failure cleanup; simulated ABI\n";
    return 0;
  }
  catch (const std::exception& e)
  {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
#endif
