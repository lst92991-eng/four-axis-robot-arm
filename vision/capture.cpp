// 主线程按需读取下一帧 BGR 原图，随后交给 YOLO。
#include "capture.hpp"
#include <opencv2/videoio.hpp>

namespace scara
{
//---------相机区---------
static cv::VideoCapture video;

// OpenCV 根据地址自动选择视频后端。
void Camera_open(const std::string &cameraUrl)
{
    video.open(cameraUrl);
}

//---------同步取帧区---------
cv::Mat Camera_read()
{
    cv::Mat frame;
    video.read(frame);
    return frame;
}

//---------关闭相机区---------
void Camera_close()
{
    video.release();
}
} // namespace scara
