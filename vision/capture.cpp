// 主线程按需读取下一帧 BGR 原图，随后交给 YOLO。
#include "capture.hpp"
#include <opencv2/videoio.hpp>

namespace scara
{
//---------相机区---------
static cv::VideoCapture video;

static void Camera_setSize()
{
    // 设置相机采集尺寸为 1280 × 720
    video.set(cv::CAP_PROP_FRAME_WIDTH, ImageWidth);
    video.set(cv::CAP_PROP_FRAME_HEIGHT, ImageHeight);
}

void Camera_open()
{
    // 打开相机设备 按照 V4L2 取流
    video.open("/dev/video11", cv::CAP_V4L2);
    // 设置相机采集尺寸，等比例缩放
    Camera_setSize();
}

//---------同步取帧区---------
cv::Mat Camera_read()
{
    // 创建Mat 对象用于存储读取的帧
    cv::Mat frame;
    // 从相机读取一帧图像
    video.read(frame);
    // 返回读取的帧给调用者
    return frame;
}

//---------关闭相机区---------
void Camera_close()
{
    // 释放相机资源
    video.release();
}
} // namespace scara
