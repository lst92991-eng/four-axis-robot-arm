#pragma once
#include <opencv2/core.hpp>

namespace scara
{
// SC3336 的棋盘标定和识别使用 1280×720 原图。
constexpr int ImageWidth = 1280, ImageHeight = 720;

// 打开 SC3336 的 ISP 主路并设置采集尺寸。
void Camera_open();
// 返回下一帧 BGR 原图。读不到时返回空图。
cv::Mat Camera_read();
// 关闭相机并释放资源。
void Camera_close();
} // namespace scara
