#pragma once
#include <opencv2/core.hpp>
#include <string>

namespace scara
{
// 打开 RTSP 地址，并启动后台接收。一个模块管理一台相机。
void Camera_open(const std::string &cameraUrl);
// 等待并写出下一帧 BGR 原图；没有图就继续等，停止后按原出口结束。
void Camera_read(cv::Mat &frame);
// 请求停止接收，允许当前读取收尾；用于主程序的停止回调。
void Camera_stop();
// 停止接收，等后台线程结束，再关闭视频流。
void Camera_close();
} // namespace scara
