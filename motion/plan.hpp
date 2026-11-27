// 运动规划的数据约定和接口，按机械参数、关节目标、动作表与轨迹组织。
#pragma once
#include <opencv2/core.hpp>
#include <array>
#include <vector>

namespace scara
{
//---------机构与示教参数区---------
// 同一示教姿态下的驱动读数、机械角，以及驱动每转一度对应的关节转角。
struct JointCalibration
{
    double driveRefDeg = 0;
    double jointRefRad = 0;
    double jointPerDrive = 1; // 负值表示两者转向相反。

    double jointRad(double driveDeg) const;
    double driveDeg(double jointRad) const;
};
struct ArmGeometry
{
    // 从肩轴到肘轴、从肘轴到吸头中心的两段有效臂长，单位毫米。
    double upperMm = 0;
    double forearmMm = 0;
    // J2 肩角相对基座，J3 肘角相对大臂。
    JointCalibration shoulder, elbow;
};
// 水平目标用驱动可接收的两个角表示，J3 保留电机端读数。
struct JointTarget
{
    double j2Deg = 0;
    double j3MotorDeg = 0;
};
// Z 的工作层保存 J1 丝杆电机的示教角，单位度。
struct WorkHeights
{
    double pickupDeg = 0;     // 在棋子上方吸合的高度。
    double clearanceDeg = 0;  // 保留示教净空层，用于核对释放高度。
    double travelDeg = 0;     // 搬运棋子的高度。
    double releaseDeg = 0;    // 在目标格断开磁铁的高度。
    // 正向吸合后看齐并贴住棋子的时间，单位毫秒。
    int gripDwellMs = 420;
};
// 本机主流程取得贴子时的 J1 驱动角后，生成棋面和搬运两档。liftCm 为相对抬升厘米数。
WorkHeights WorkHeights_fromContact(double contactDeg, double liftCm = 4.3);
// 三个元素依次为 J1 丝杆驱动角、J2 反馈角和 J3 电机角，单位度。
using AxisValues = std::array<double, 3>;

//---------取放动作数据区---------
// Grip 使磁铁吸持，Release 和 ReleaseOffBoard 分别标记盘上与盘外的关断。
enum class PickAction
{
    Grip,
    Move,
    Release,
    ReleaseOffBoard,
    LiftZ // 从装置的当前姿态先垂直抬升，保持 J2/J3 的位置。
};
// steps 的每一项保存一种动作，执行器按追加顺序读取。
struct PickStep
{
    PickAction action = PickAction::Grip;
    AxisValues target{};
    // 吸合时的原地停留时间，单位毫秒。
    int durationMs = 0;
};

//---------取放动作接口区---------
// 向本手的动作表追加一个关节目标。
void Plan_move(JointTarget joints, double zDeg, std::vector<PickStep> &steps);
// 用已算好的关节目标排取放和回观察位。普通走子的弃子目标传 nullptr。
void Plan_turn(const WorkHeights &z, const JointTarget &source, const JointTarget &target,
               const JointTarget *discard, const JointTarget &park, std::vector<PickStep> &steps);

//---------吸头坐标与关节角区---------
// 用驱动角算吸头的基座 XY 毫米，供示教采点核对。
cv::Point2d Arm_forward(const ArmGeometry &arm, const JointTarget &joints);
// 用基座 XY 毫米算一组关节角，不可达时返回 false。
bool Arm_inverse(const ArmGeometry &arm, cv::Point2d baseMm, JointTarget &joints);

//---------三轴共同轨迹区---------
struct MotionLimits
{
    AxisValues velocity{}, acceleration{}; // 各轴最大速度（度/秒）和加速度（度/秒²）。
    bool valid() const;
};
struct JointTrajectory
{
    AxisValues start{}, target{};
    double seconds = 0; // 三轴共用的运动时间。
};
struct JointReference
{
    AxisValues position{}, velocity{}; // 同一时刻发给三轴的位置（度）和速度（度/秒）。
};
// 按最慢轴的速度和加速度约束确定三轴共用时长。
bool Trajectory_plan(const AxisValues &start, const AxisValues &target, const MotionLimits &limits,
                     JointTrajectory &trajectory);
// 返回这一时刻的三轴位置和速度参考。
JointReference Trajectory_sample(const JointTrajectory &trajectory, double elapsedSeconds);
} // namespace scara
