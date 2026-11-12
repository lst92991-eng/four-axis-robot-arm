#pragma once
#include <opencv2/core.hpp>
#include <string>

namespace scara {
// 一个模块管理一台相机。主流程依次调用打开、读取、关闭，后台只负责接收。
void Camera_open(const std::string& source);
// 等待调用后接收到的一帧。调用者持有图像后，后台不会改写其中的像素。
cv::Mat Camera_read();
void Camera_close();
}
