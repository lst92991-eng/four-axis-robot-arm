#pragma once
#include <opencv2/core.hpp>
#include <string>

namespace scara
{
// 打开 RTSP 地址，并启动后台接收。一个模块管理一台相机。
void Camera_open(const std::string &cameraUrl);
// 写出调用后收到的一帧 BGR 原图；短暂失败重试，5秒无图由 main 的统一出口处理。
void Camera_read(cv::Mat &frame);
// 停止接收，等后台线程结束，再关闭视频流。
void Camera_close();
} // namespace scara
