#pragma once
#include "motion/plan.hpp"
#include <string>

namespace scara
{
// 相机、引擎和驱动各取本次运行所需的参数，首次观察时打开视觉。
void RV1106_configure(const std::string &modelPath, cv::Point cropOrigin,
                      const std::string &engineDir, int skillLevel, int thinkMs,
                      const MotionLimits &limits);
}
