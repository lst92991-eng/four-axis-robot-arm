// 相机接口按打开、交帧、请求停止和关闭排列，供视觉预览与对局共用。
#pragma once
#include <opencv2/core.hpp>
#include <string>

namespace scara
{
//---------取流与资源释放接口区---------
// 打开 RTSP 地址，结果直接决定接收开关；打开失败时 Camera_read 按无帧出口结束。
void Camera_open(const std::string &cameraUrl);
// frame 接收调用后到达的新 BGR 原图。timeoutMs 为 0 时等待新帧或停止，正数指定等待毫秒数。
void Camera_read(cv::Mat &frame, int timeoutMs = 0);
// 请求停止接收，允许当前读取收尾；用于主程序的停止回调。
void Camera_stop();
// 停止接收，等后台线程结束，再关闭视频流。
void Camera_close();
} // namespace scara
