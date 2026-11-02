#pragma once
#include "motion/kinematics.hpp"
#include "task/pick_place.hpp"
#include "vision/board_vision.hpp"
#include <string>

namespace app {
struct Config {
    std::string canInterface, gpioChip, engineDirectory, model, cameraScript, journal;
    unsigned gpioLine = 0;
    bool gpioActiveHigh = true, robotRed = false, hardwareVerified = false;
    // 故障停止后，必须确认当前场地允许在限时结束断磁；默认禁止实机入口。
    bool faultReleaseSafe = false, latencyCalibrated = false, requireTcp = false;
    double captureLatencyBoundSeconds = 0;
    int thinkMs = 1000, cameraWidth = 1280, cameraHeight = 960;
    double pieceContactZ = 0, pieceYaw = 0;
    motion::Config motion;
    task::Config task;
    VisionConfig vision;
    std::array<std::vector<motion::Pose>, 2> captureSlots; // 0红子，1黑子。
};
Config loadConfig(const std::string& path);
void validateConfig(const Config& config);
} // namespace app
