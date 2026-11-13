#pragma once
#include <opencv2/core.hpp>
#include <string>

namespace scara
{
// 一个模块管理一台相机。主流程依次调用打开、读取、关闭，后台只负责接收。
bool Camera_open(const std::string &source);
// 将调用后接收到的新帧写入 frame；结束时写入空图并返回 false。
bool Camera_read(cv::Mat &frame);
void Camera_close();
} // namespace scara
