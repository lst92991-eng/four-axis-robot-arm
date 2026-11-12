#pragma once
#include <opencv2/core.hpp>
#include <string>

namespace scara {
// 一个模块管理一台相机，图像数据由模块内部保存。
void Camera_open(const std::string& source);
// 等待调用后接收到的一帧，返回供识别使用的独立图像。
cv::Mat Camera_read();
void Camera_close();
}
