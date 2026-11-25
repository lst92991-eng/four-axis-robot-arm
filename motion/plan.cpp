// 把基座 XY 换成关节目标，顺序排出取放动作，并为一段三轴运动计算共同进度。
#include "plan.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace scara
{
// 驱动读数用度，三角函数用弧度，在正逆解的边界换算。
constexpr double RadiansPerDegree = CV_PI / 180.0;
constexpr double DegreesPerRadian = 180.0 / CV_PI;

//---------取放步骤区---------
WorkHeights WorkHeights_fromContact(double contactDeg, double liftCm)
{
    // 这台 J1 的历史记录：驱动角增加 2006.1005°，吸头估计抬高 4.3 cm。
    if (!std::isfinite(contactDeg) || !(liftCm > 0)) throw std::runtime_error("需要有效贴子角和正抬升距离");
    WorkHeights z;
    z.pickupDeg = z.releaseDeg = contactDeg;
    z.travelDeg = z.clearanceDeg = contactDeg + (liftCm / 4.3) * 2006.1005;
    if (!std::isfinite(z.travelDeg)) throw std::runtime_error("抬升目标超出范围");
    return z;
}

// 把一个目标姿态追加到 steps。三个位置依次是 J1、J2 和 J3 驱动角。
void Plan_move(JointTarget xy, double zDeg, std::vector<PickStep> &steps)
{
    steps.push_back({PickAction::Move, {zDeg, xy.j2Deg, xy.j3MotorDeg}});
}

// 取子先到棋子上方，再竖直下降吸合，最后抬到允许横移的高度。
static void Plan_pick(const WorkHeights &z, const JointTarget &source, std::vector<PickStep> &steps)
{
    Plan_move(source, z.travelDeg, steps);
    Plan_move(source, z.pickupDeg, steps);
    // 吸头停在取子高度通电，gripDwellMs 留出棋子贴合的时间。
    PickStep grip;
    grip.action = PickAction::Grip;
    grip.durationMs = z.gripDwellMs;
    steps.push_back(grip);
    Plan_move(source, z.clearanceDeg, steps);
}

// 落子先到目标格上方，下降到释放高度，关断磁铁后抬起。
static void Plan_place(const WorkHeights &z, const JointTarget &target, std::vector<PickStep> &steps)
{
    Plan_move(target, z.travelDeg, steps);
    Plan_move(target, z.releaseDeg, steps);
    steps.push_back({PickAction::Release});
    Plan_move(target, z.clearanceDeg, steps);
}

void Plan_turn(const ArmGeometry &arm, const WorkHeights &z, cv::Point2d sourceMm, cv::Point2d targetMm,
               const cv::Point2d *discardMm, const JointTarget &park, std::vector<PickStep> &steps)
{
    steps.clear();
    if (z.pickupDeg == z.travelDeg) throw std::runtime_error("吸取和搬运高度尚未示教");
    JointTarget source, target, bin;
    if (!Arm_inverse(arm, sourceMm, source) || !Arm_inverse(arm, targetMm, target) ||
        (discardMm && !Arm_inverse(arm, *discardMm, bin)))
        throw std::runtime_error("取放目标不可达");

    // 目标全部有效后再排动作。吃子先腾空目标格，落子时复用它的关节角。
    if (discardMm)
    {
        Plan_pick(z, target, steps);
        Plan_move(bin, z.travelDeg, steps); // 盘外在搬运高度关断磁铁。
        steps.push_back({PickAction::ReleaseOffBoard});
    }
    Plan_pick(z, source, steps);
    Plan_place(z, target, steps);
    Plan_move(park, z.travelDeg, steps);
}

//---------坐标正逆解区---------
// 输入 J2、J3 的驱动读数，输出吸头中心相对基座的 XY 毫米。
cv::Point2d Arm_forward(const ArmGeometry &arm, const JointTarget &joints)
{
    //大臂相对基座、小臂相对大臂的角度，弧度。
    double shoulder, elbow;

    // 从示教读数算转角差，应用方向和传动比例，再加回示教时的实际关节角。
    shoulder =
        arm.shoulderReferenceRad + (joints.j2Deg - arm.j2ReferenceDeg) * arm.j2JointPerDrive * RadiansPerDegree;
    elbow = arm.elbowReferenceRad +
            (joints.j3MotorDeg - arm.j3ReferenceMotorDeg) * arm.j3JointPerDrive * RadiansPerDegree;
    // 小臂相对基座的方向是 shoulder + elbow。两段臂在 X、Y 上的投影相加。
    return {arm.upperMm * std::cos(shoulder) + arm.forearmMm * std::cos(shoulder + elbow),
            arm.upperMm * std::sin(shoulder) + arm.forearmMm * std::sin(shoulder + elbow)};
}

// 输入吸头目标 XY，求与示教肘形同侧的一组 J2、J3 目标读数。
bool Arm_inverse(const ArmGeometry &arm, cv::Point2d baseMm, JointTarget &joints)
{
    joints = {};
    // 基座原点到目标的距离平方和两段臂长，组成求肘角的三角形。
    double radiusSquared = baseMm.dot(baseMm);
    double upper = arm.upperMm, lower = arm.forearmMm;
    if (!(upper > 0 && lower > 0) || !std::isfinite(arm.j2JointPerDrive) ||
        !std::isfinite(arm.j3JointPerDrive) || arm.j2JointPerDrive == 0 || arm.j3JointPerDrive == 0) return false;
    // 用余弦定理求肘角。余弦超出 [-1,1] 时，两段臂够不到这个点。
    double cosine = (radiusSquared - upper*upper - lower*lower) / (2*upper*lower);
    if (!(cosine >= -1 && cosine <= 1)) return false;
    // 先按示教肘角的符号选肘形，再从目标方位角中减去小臂带来的夹角。
    double elbow = std::copysign(std::acos(cosine), arm.elbowReferenceRad);
    double shoulder = std::atan2(baseMm.y, baseMm.x) - std::atan2(lower*std::sin(elbow), upper+lower*std::cos(elbow));
    // 把相对示教姿态的角差折回半圈范围，再按传动比和方向换算为驱动读数。
    double shoulderDelta = std::remainder(shoulder-arm.shoulderReferenceRad, 2*CV_PI) * DegreesPerRadian;
    double elbowDelta = std::remainder(elbow-arm.elbowReferenceRad, 2*CV_PI) * DegreesPerRadian;
    JointTarget target;
    target.j2Deg = arm.j2ReferenceDeg + shoulderDelta / arm.j2JointPerDrive;
    target.j3MotorDeg = arm.j3ReferenceMotorDeg + elbowDelta / arm.j3JointPerDrive;
    if (!std::isfinite(target.j2Deg) || !std::isfinite(target.j3MotorDeg)) return false;
    joints = target;
    return true;
}

//---------联动轨迹区---------
// 输入三轴的起止角和各轴限幅，输出整段共用的时长，单位秒。
bool Trajectory_plan(const AxisValues &start, const AxisValues &target, const MotionLimits &limits,
                     JointTrajectory &trajectory)
{
    trajectory = {};
    //五次进度曲线的最大一阶导为1.875，最大二阶导为10/sqrt(3)
    double seconds = 0;
    for (int axis = 0; axis < 3; ++axis)
    {
        if (!std::isfinite(start[axis]) || !std::isfinite(target[axis])) return false;
        if (!(limits.velocity[axis] > 0) || !(limits.acceleration[axis] > 0) ||
            !std::isfinite(limits.velocity[axis]) || !std::isfinite(limits.acceleration[axis])) return false;
        // 用每个轴要走的角度，分别算出满足限速和限加速度所需的最短时长。
        double distance = std::abs(target[axis] - start[axis]);
        double velocityTime = 1.875 * distance / limits.velocity[axis];
        double accelerationTime = std::sqrt((10 / std::sqrt(3.0)) * distance / limits.acceleration[axis]);
        // 取三个轴所有时长要求中的最大值，使三轴能在同一时刻到达。
        seconds = std::max({seconds, velocityTime, accelerationTime});
    }
    if (!std::isfinite(seconds) || seconds > 3600) return false;
    trajectory.start = start;
    trajectory.target = target;
    trajectory.seconds = seconds;
    return true;
}

JointReference Trajectory_sample(const JointTrajectory &trajectory, double elapsedSeconds)
{
    // 轨迹时间走完，参考位置固定在终点，参考速度置零。实际停稳由执行器查反馈。
    if (!(trajectory.seconds > 0) || elapsedSeconds >= trajectory.seconds)
        return {trajectory.target, {}};

    // u 是已经用掉的时间比例。五次曲线给出走过的路程比例和每秒变化量。
    double u = std::max(elapsedSeconds, 0.0) / trajectory.seconds;
    double progress = u * u * u * (10 + u * (-15 + 6 * u));
    double progressPerSecond = 30 * u * u * (1 - u) * (1 - u) / trajectory.seconds;
    // 三个轴使用同一个进度，各自乘本轴带方向的行程，得到参考角和参考角速度。
    JointReference reference;
    for (int axis = 0; axis < 3; ++axis)
    {
        double distance = trajectory.target[axis] - trajectory.start[axis];
        reference.position[axis] = trajectory.start[axis] + distance * progress;
        reference.velocity[axis] = distance * progressPerSecond;
    }
    return reference;
}
} // namespace scara
