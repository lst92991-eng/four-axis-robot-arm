#pragma once
#include <opencv2/core.hpp>
#include <array>
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
    //示教得到的J2反馈角到关节角比例
    double j2Scale = 1;
    //J3电机角到肘轴角的传动比
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
    //J1吸取层的已示教角度
    double pickupDeg = 0;
    //允许横移的已示教角度，缺省沿用搬运层
    double clearanceDeg = 0;
    //搬运层与释放层的已示教角度
    double travelDeg = 0;
    double releaseDeg = 0;
    //释放脉冲持续毫秒数
    int releasePulseMs = 1000;
};
//三个元素依次为J1电机角、J2反馈角、J3电机角；J4保持原姿态
using AxisValues = std::array<double, 3>;
constexpr unsigned ZAxis = 1, AllAxes = 7;
struct MotionLimits
{
    //每轴的速度和加速度上限，单位为度/秒、度/秒²
    AxisValues velocity{}, acceleration{};
};
struct JointTrajectory
{
    //同一段三轴的起点、终点和公共时长，秒
    AxisValues start{}, target{};
    double seconds = 0;
};
struct JointReference
{
    //同一时刻同时交给三个驱动的参考
    AxisValues position{}, velocity{};
};
enum class PickAction
{
    Hold,
    Move,
    ReleasePulse
};
struct PickStep
{
    PickAction action = PickAction::Hold;
    AxisValues target{};
    //位0/1/2选择J1/J2/J3，其余轴使用本段起点位置
    unsigned axes = AllAxes;
    int durationMs = 0;
};

bool Arm_load(const std::string &path, ArmGeometry &arm);
void Arm_forward(const ArmGeometry &arm, const JointTarget &joints, cv::Point2d &baseMm);
bool Arm_inverse(const ArmGeometry &arm, cv::Point2d baseMm, JointTarget &joints);
bool WorkHeights_load(const std::string &path, WorkHeights &heights);
//共同进度s(t)生成三轴轨迹，按最慢轴的速度/加速度约束确定时长
bool Trajectory_plan(const AxisValues &start, const AxisValues &target, const MotionLimits &limits,
                     JointTrajectory &trajectory);
void Trajectory_sample(const JointTrajectory &trajectory, double elapsedSeconds, JointReference &reference);
//进入时至少处于clearance层；脱离棋子后，J1后半段抬升与J2/J3搬运共同执行
//释放脉冲结束后再抬起，J4保持原姿态
bool Pick_plan(const ArmGeometry &arm, const WorkHeights &heights, cv::Point2d sourceMm, cv::Point2d destinationMm,
               bool alreadyHeld, std::vector<PickStep> &steps);
} // namespace scara
