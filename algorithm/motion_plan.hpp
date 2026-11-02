#pragma once
#include "fk.hpp"
#include <string>
#include <vector>

namespace scara {

enum class ToolAction { Keep, VacuumOn, VacuumOff };

struct PlanStep {
    Pose pose{};
    Joints joints{};
    // Keep表示在seconds秒内到达目标；吸/放表示在当前位置切换气路后等待seconds。
    double seconds = 0;
    ToolAction tool = ToolAction::Keep;
};

struct MotionConfig {
    Joints minimum{};   // 肩/肘/腕为rad，Z为mm。
    Joints maximum{};
    Joints maxSpeed{};  // 肩/肘/腕为rad/s，Z为mm/s，均须显式提供正值。
    double safeZ = 0;   // 使用者给出的通行高度，不是程序自动测得的避障高度。
    double cartesianSpeed = 0;  // mm/s。
    double yawSpeed = 0;        // rad/s。
    double cartesianStep = 0;   // 相邻笛卡尔采样点最大间距，mm。
    double angularStep = 0;     // 相邻转动关节最大变化，rad；也用于yaw分段。
    double toolDelay = 0;       // 吸/放后的等待秒数；真实气路尚须实测。
    bool calibrated = false;   // 未确认的限位/速度不能默认视为硬件标定值。
};

struct MotionPlan {
    std::vector<PlanStep> steps;
    bool calibrationConfirmed = false;
};

// 依次连接给定Pose，选最靠近上一个关节位置且满足限位的IK解。
// 轨迹只有离散目标和限速时间，不包含加速度/碰撞/驱动器规划，不发送硬件动作。
// calibrated=false仍可用显式假设参数生成预览，结果也明确标记为未标定。
// 失败清空plan，error给出原因；未知的零速度/零步长配置会被拒绝。
bool planTrajectory(const Joints& start, const std::vector<Pose>& targets,
                    const MotionConfig& config, MotionPlan& plan, std::string& error);

// 抬升→取子上方→下降→吸→抬升→放子上方→下降→放→抬升。
// 起点应为空吸盘；这个约定由应用层确认，算法无法从模型判断是否已吸住棋子。
bool planPickPlace(const Joints& start, const Pose& pick, const Pose& place,
                   const MotionConfig& config, MotionPlan& plan, std::string& error);

} // namespace scara
