#pragma once
#include <opencv2/core.hpp>
#include <string>

namespace scara
{
// 一个模块管理一台相机。主流程依次调用打开、读取、关闭，后台只负责接收。
bool Camera_open(const std::string &source);
// 等待调用后接收到的一帧；读流结束返回空图，调用者统一关闭。
cv::Mat Camera_read();
void Camera_close();
} // namespace scara
