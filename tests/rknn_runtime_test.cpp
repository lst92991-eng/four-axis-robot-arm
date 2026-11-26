// 模拟 RKNN C ABI，验证本项目适配层的步幅、失败清理和退出。
#include "vision/yolo.hpp"
#include <rknn_api.h>
#include <array>
#include <csignal>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <stdexcept>

#ifdef RKNN_MOCK_ENTRY
#include "vision/capture.hpp"
#include <opencv2/videoio.hpp>

namespace scara
{
static cv::VideoCapture camera;

void Camera_open()
{
  const char *fixture = std::getenv("SCARA_MOCK_CAMERA_URL");
  if (fixture) camera.open(fixture);
}

cv::Mat Camera_read()
{
  cv::Mat frame;
  camera.read(frame);
  return frame;
}

void Camera_close()
{
  camera.release();
}
}
#endif

namespace {
int allocations = 0, alive = 0, failAt = 0;
int failQuery = -1, failBind = 0, bindings = 0;
bool contextOpen = false, badOutput = false, failRun = false;
int rowPadding = 2;
bool shortOutput = false;
int runs = 0;
rknn_tensor_mem *input = nullptr;
std::array<rknn_tensor_mem *, 3> outputs{};

void check(bool value, const char *message)
{
  if (!value) throw std::runtime_error(message);
}

bool openFailed()
{
  return scara::Yolo_open("mock.rknn")==false;
}
}

extern "C" {
int rknn_init(rknn_context *context, void *model, uint32_t, uint32_t, rknn_init_extend *)
{
  if (std::strcmp(static_cast<const char *>(model), "missing.rknn") == 0) return -1;
#ifdef RKNN_MOCK_ENTRY
  std::cerr << "TEST ONLY: simulated RKNN ABI, one synthetic detection\n";
  const char *failure = std::getenv("SCARA_MOCK_FAILURE");
  if (failure)
  {
    if (std::strcmp(failure,"query")==0) failQuery = RKNN_QUERY_NATIVE_INPUT_ATTR;
    if (std::strcmp(failure,"allocate")==0) failAt = 2;
    if (std::strcmp(failure,"bind")==0) failBind = 3;
    if (std::strcmp(failure,"run")==0) failRun = true;
  }
#endif
  *context = 1;
  contextOpen = true;
  allocations = 0;
  bindings = 0;
  runs = 0;
#ifdef RKNN_MOCK_ENTRY
  if (std::getenv("SCARA_MOCK_CANCEL_INIT")) std::raise(SIGINT);
#endif
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
  attr.w_stride = rowPadding ? size + rowPadding : 0;
  attr.n_elems = size*size*attr.dims[3];
  attr.size_with_stride = size*(size+rowPadding)*attr.dims[3]+32;
  if (!isInput && shortOutput) attr.size_with_stride = 1;
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
  ++runs;
  if (failRun) return -7;
#ifndef RKNN_MOCK_ENTRY
  const auto *bytes = static_cast<const uint8_t *>(input->virt_addr);
  for (int row=0; row<640; ++row)
  {
    for (int x=0; x<640; ++x)
    {
      size_t offset = (row*(640+rowPadding)+x)*3;
      check(bytes[offset]==11 && bytes[offset+1]==22 && bytes[offset+2]==33,
            "Input rows were copied with the wrong stride");
    }
    for (int padding=640*3; padding<(640+rowPadding)*3; ++padding)
      check(bytes[row*(640+rowPadding)*3+padding]==0, "Image copy overwrote the NPU input row padding");
  }
#endif
  for (auto *output : outputs) std::memset(output->virt_addr, 128, output->size);
  auto *p = static_cast<int8_t *>(outputs[0]->virt_addr)+(20*(80+rowPadding)+20)*57;
  p[0] = p[1] = p[2] = p[3] = 0;
  p[4] = p[5+12] = 127;
#ifdef RKNN_MOCK_ENTRY
  // 测试驱动自行结束连续程序，产品入口不再保留运行N帧的分支。
  if (runs==3) std::raise(SIGINT);
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
#ifdef RKNN_MOCK_ENTRY
  std::cerr << "TEST cleanup: allocations=" << alive << " runs=" << runs << '\n';
#endif
  return 0;
}
}

#ifndef RKNN_MOCK_ENTRY
int main()
{
  try
  {
    check(!scara::Yolo_open("missing.rknn") && !contextOpen && allocations == 0,
          "Model load failure continued to buffer setup");
    for (failAt=1; failAt<=4; ++failAt)
    {
      bool failed = openFailed();
      check(failed && !contextOpen && !alive, "Partial allocation leaked resources");
    }
    failAt = 0;
    for (int command : {RKNN_QUERY_NATIVE_INPUT_ATTR, RKNN_QUERY_NATIVE_NHWC_OUTPUT_ATTR})
    {
      failQuery = command;
      bool failed = openFailed();
      check(failed && !contextOpen && !alive, "Query failure was accepted or leaked resources");
    }
    failQuery = -1;
    for (failBind=1; failBind<=4; ++failBind)
    {
      bool failed = openFailed();
      check(failed && !contextOpen && !alive, "Binding failure was accepted or leaked resources");
    }
    failBind = 0;
    badOutput = true;
    bool failed = openFailed();
    check(failed && !contextOpen && !alive, "Unsupported output was accepted or leaked resources");
    badOutput = false;
    shortOutput = true;
    check(openFailed() && !contextOpen && !alive, "Accepted a buffer shorter than one output row");
    shortOutput = false;
    check(scara::Yolo_open("mock.rknn"), "Cannot open test model");
    check(scara::Yolo_open("mock.rknn"), "Cannot reopen test model");
    check(contextOpen && alive==4, "Reopening the fixed model leaked its previous buffers");
    cv::Mat storage(648,656,CV_8UC3,cv::Scalar(99,88,77));
    cv::Mat rgb = storage(cv::Rect(3,4,640,640));
    rgb.setTo(cv::Scalar(11,22,33));
    check(!rgb.isContinuous(), "Stride test unexpectedly used tightly packed source rows");
    std::vector<scara::Detection> pieces;
    check(scara::Yolo_run(rgb, {0,0}, pieces), "Inference with padded rows failed");
    check(pieces.size()==1 && pieces[0].classId==12, "Native padded output was decoded incorrectly");
    rowPadding = 0;
    check(scara::Yolo_open("mock.rknn"), "Could not reopen with packed rows");
    std::vector<scara::Detection> packed;
    check(scara::Yolo_run(rgb, {0,0}, packed), "Inference with packed rows failed");
    check(packed.size()==1 && packed[0].classId==pieces[0].classId && packed[0].bbox==pieces[0].bbox &&
          packed[0].confidence==pieces[0].confidence, "Packed and padded layouts produced different detections");
    failRun = true;
    failed = !scara::Yolo_run(rgb, {0,0}, packed);
    scara::Yolo_close();
    check(failed && !contextOpen && !alive, "Inference failure cleanup failed");
    std::cout << "PASS RKNN input/output strides, failure propagation and cleanup; simulated ABI\n";
    return 0;
  }
  catch (const std::exception& e)
  {
    scara::Yolo_close();
    std::cerr << e.what() << '\n';
    return 1;
  }
}
#endif
