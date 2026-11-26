// RV1106 上的模型加载、推理和释放。
#include "yolo.hpp"
#include "models/xiangqi14-local-v3/rknn_buffers.hpp"

namespace scara
{
//---------模型与缓冲区---------
static rknn_context context = 0;
static RknnBuffers buffers;

//---------打开模型区---------
bool Yolo_open(std::string path)
{
    Yolo_close();
    int status = rknn_init(&context, path.data(), 0, 0, nullptr);
    if (status != 0) return false;

    bool ready = bindModelBuffers(context, buffers);
    if (!ready) Yolo_close();
    return ready;
}

//---------单帧推理区---------
bool Yolo_run(const cv::Mat &rgb, cv::Point origin, std::vector<Detection> &pieces)
{
    // 输入图已绑定到 NPU 内存。固定目标视图，按实际行距写入像素。
    const cv::Mat &input = buffers.input;
    rgb.copyTo(input);
    if (rknn_run(context, nullptr) != 0) return false;
    pieces = Vision_decode(buffers.heads, origin);
    return true;
}

//---------关闭模型区---------
void Yolo_close()
{
    for (auto *buffer : buffers.memory)
        if (buffer) rknn_destroy_mem(context, buffer);
    if (context) rknn_destroy(context);
    buffers = {};
    context = 0;
}

} // namespace scara
