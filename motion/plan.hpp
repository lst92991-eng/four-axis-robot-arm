#pragma once
#include <opencv2/core.hpp>
#include <string>
#include <vector>

namespace scara
{
// 直接使用实机示教保存的模型；J2反馈角、J3电机角的换算分别保留。
struct ArmGeometry
{
    double upperMm = 0;
    double forearmMm = 0;
    double j2ReferenceDeg = 0;
    double j3ReferenceMotorDeg = 0;
    double j2Scale = 1;
    double j3Ratio = 1;
    double j2Sign = 1;
    double j3Sign = 1;
    double shoulderReferenceRad = 0;
    double elbowReferenceRad = 0;
};
struct JointTarget
{
    double j2Deg = 0;
    double j3MotorDeg = 0;
};
struct WorkHeights
{
    double pickupDeg = 0;
    double travelDeg = 0;
    double releaseDeg = 0;
    double speedDegS = 180;
    int releasePulseMs = 1000;
};
enum class PickAction
{
    Hold,
    MoveXY,
    MoveZ,
    ReleasePulse
};
struct PickStep
{
    PickAction action = PickAction::Hold;
    JointTarget joints;
    double zDeg = 0;
    double speedDegS = 0;
    int durationMs = 0;
};

bool Arm_load(const std::string &path, ArmGeometry &arm);
void Arm_forward(const ArmGeometry &arm, const JointTarget &joints, cv::Point2d &baseMm);
bool Arm_inverse(const ArmGeometry &arm, cv::Point2d baseMm, JointTarget &joints);
bool WorkHeights_load(const std::string &path, WorkHeights &heights);
// Arm_load、WorkHeights_load成功后使用；输入为已标定XY和两层示教高度。
// 执行前已经在搬运高度，J4保持原姿态。
// 本模块只生成步骤，不发送CAN/GPIO。释放脉冲完成后，才进入最后的抬升步骤。
bool Pick_plan(const ArmGeometry &arm, const WorkHeights &heights, cv::Point2d sourceMm, cv::Point2d destinationMm,
               bool alreadyHeld, std::vector<PickStep> &steps);
} // namespace scara
