#pragma once
#include "algorithm/motion_plan.hpp"

// 教学基准尺寸集中放在这里。35mm为常见成品规格，40mm是本项目选定格距。
// 这是虚拟布局，不代表机械零点、真实限位、棋盘高度已经标定。
namespace project {
inline constexpr double PieceDiameter = 35.0;
inline constexpr double Pitch = 40.0;
inline constexpr double BoardWidth = 480.0;
inline constexpr double BoardHeight = 520.0;
inline constexpr double GridStart = 80.0;
inline constexpr double MarkerSide = 48.0;
inline constexpr double MarkerInset = 32.0;
inline constexpr int ImageWidth = 1280;
inline constexpr int ImageHeight = 960;
inline constexpr int StableFrames = 3;
inline constexpr char Classes[] = "kabnrcpKABNRCP";

// 肩轴在(0,0)。棋盘长边沿机器人X方向，减少向前所需的伸展距离。
inline scara::Pose squarePose(int row, int col)
{
    return {(4.5-row)*Pitch, 90.0+col*Pitch, 12.0, 0.0};
}

// 每方15个被吃棋子的槽位，位于棋盘两侧；将帅不放入吃子槽。
inline scara::Pose capturePose(bool red, int slot)
{
    const double x = 230.0+Pitch*(slot%3);
    return {red ? -x : x, 90.0+Pitch*(slot/3), 12.0, 0.0};
}

inline scara::MotionConfig simulationMotionConfig()
{
    scara::MotionConfig c;
    c.minimum = {-4*scara::Pi, -scara::Pi, 0, -4*scara::Pi};
    c.maximum = {4*scara::Pi, scara::Pi, 160, 4*scara::Pi};
    c.maxSpeed = {1.0, 1.0, 80.0, 1.5};
    c.safeZ = 70;
    c.cartesianSpeed = 120;
    c.yawSpeed = 1;
    c.cartesianStep = 1;
    c.angularStep = 0.05;
    c.toolDelay = 0.25;
    c.calibrated = false;
    return c;
}
} // namespace project
