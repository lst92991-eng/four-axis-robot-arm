#pragma once
#include <opencv2/core.hpp>
#include <string>

namespace scara
{
// 打开 RTSP 流，预览和对局都由自己的主线程取图。
void Camera_open(const std::string &cameraUrl);
// 返回下一帧 BGR 原图。读不到时返回空图。
cv::Mat Camera_read();
void Camera_close();
} // namespace scara
