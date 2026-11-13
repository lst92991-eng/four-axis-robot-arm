#pragma once
#include <opencv2/core.hpp>
#include <string>

namespace scara
{
// 一个模块管理一台相机。主流程依次调用打开、读取、关闭，后台只负责接收。
void Camera_open(const std::string &source);
// 写出调用后收到的新帧；失败交给主入口的异常出口，调用者不用逐句判断。
void Camera_read(cv::Mat &frame);
void Camera_close();
} // namespace scara
