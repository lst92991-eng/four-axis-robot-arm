// 将吸头的位置换算成驱动角，再按三轴共用的进度计算参考位置和速度。
#include "plan.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace scara
{
// 驱动读数用度，三角函数用弧度，在正逆解的边界换算。
constexpr double RadiansPerDegree = CV_PI / 180.0;
constexpr double DegreesPerRadian = 180.0 / CV_PI;

//---------J1 高度换算区---------
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

//---------驱动读数与关节角换算区---------
double JointCalibration::jointRad(double driveDeg) const
{
    return jointRefRad + (driveDeg - driveRefDeg) * jointPerDrive * RadiansPerDegree;
}

double JointCalibration::driveDeg(double jointRad) const
{
    if (jointPerDrive == 0 || !std::isfinite(jointPerDrive)) return NAN;
    // 以示教姿态为基准，角差跨过 ±π 时折回同一圈。
    double deltaRad = std::remainder(jointRad - jointRefRad, 2 * CV_PI);
    return driveRefDeg + deltaRad * DegreesPerRadian / jointPerDrive;
}

//---------平面两连杆正逆解区---------
cv::Point2d Arm_forward(const ArmGeometry &arm, const JointTarget &joints)
{
    // 大臂相对基座转过肩角，小臂相对基座再加上肘角。
    double upperAngle = arm.shoulder.jointRad(joints.j2Deg);
    double forearmAngle = upperAngle + arm.elbow.jointRad(joints.j3MotorDeg);
    double x = arm.upperMm * std::cos(upperAngle) + arm.forearmMm * std::cos(forearmAngle);
    double y = arm.upperMm * std::sin(upperAngle) + arm.forearmMm * std::sin(forearmAngle);
    return {x, y};
}

bool Arm_inverse(const ArmGeometry &arm, cv::Point2d xy, JointTarget &joints)
{
    joints = {};
    double upper = arm.upperMm;
    double lower = arm.forearmMm;
    if (!(upper > 0 && lower > 0)) return false;

    // 余弦定理求肘角，沿用示教时肘部弯曲的方向。
    double radiusSquared = xy.dot(xy);
    double cosElbow = (radiusSquared - upper * upper - lower * lower) / (2 * upper * lower);
    if (!(std::abs(cosElbow) <= 1)) return false;
    double elbow = std::copysign(std::acos(cosElbow), arm.elbow.jointRefRad);

    // 从目标方位角减去目标连线相对大臂的偏角，得到肩角。
    double alongUpper = upper + lower * std::cos(elbow);
    double acrossUpper = lower * std::sin(elbow);
    double offset = std::atan2(acrossUpper, alongUpper);
    double bearing = std::atan2(xy.y, xy.x);
    double shoulder = bearing - offset;

    double j2 = arm.shoulder.driveDeg(shoulder);
    double j3 = arm.elbow.driveDeg(elbow);
    if (!std::isfinite(j2) || !std::isfinite(j3)) return false;
    joints = {j2, j3};
    return true;
}

//---------联动轨迹区---------
// 速度与加速度上限都参与除法。安装文件和规划入口共用这条要求。
bool MotionLimits::valid() const
{
    for (double speed : velocity)
        if (!std::isfinite(speed) || speed <= 0) return false;
    for (double accel : acceleration)
        if (!std::isfinite(accel) || accel <= 0) return false;
    return true;
}

// 五次进度曲线在 u = 0～1 上的一阶导数峰值和二阶导数绝对值峰值。
constexpr double PeakProgressSpeed = 1.875;
const double PeakProgressAcceleration = 10 / std::sqrt(3.0);

bool Trajectory_plan(const AxisValues &start, const AxisValues &target, const MotionLimits &limits,
                     JointTrajectory &trajectory)
{
    trajectory = {};
    if (!limits.valid()) return false;

    double seconds = 0;
    for (int axis = 0; axis < 3; ++axis)
    {
        double distance = std::abs(target[axis] - start[axis]);
        if (!std::isfinite(distance)) return false;

        // 本轴走完这段行程，限速和限加速度各给出一个最短用时，单位为秒。
        double speedTime = PeakProgressSpeed * distance / limits.velocity[axis];
        double accelTime = std::sqrt(PeakProgressAcceleration * distance / limits.acceleration[axis]);
        double axisTime = std::max(speedTime, accelTime);

        // 最慢的轴决定三轴的共同用时。
        seconds = std::max(seconds, axisTime);
    }
    // 单段允许的最长时间为一小时。
    if (seconds > 3600) return false;
    trajectory = {start, target, seconds};
    return true;
}

JointReference Trajectory_sample(const JointTrajectory &trajectory, double elapsedSeconds)
{
    // 轨迹时间走完，参考位置固定在终点，参考速度置零。实际停稳由执行器查反馈。
    if (!(trajectory.seconds > 0) || elapsedSeconds >= trajectory.seconds)
        return {trajectory.target, {}};

    // u 从 0 走到 1。路程比例 s = 10u³ − 15u⁴ + 6u⁵，两端的速度和加速度均为零。
    double u = std::max(elapsedSeconds, 0.0) / trajectory.seconds;
    double u2 = u * u;
    double u3 = u2 * u;
    double progress = u3 * (10 - 15 * u + 6 * u2);
    // s 对时间求导，得到每秒走过的路程比例。
    double progressPerSecond = 30 * u2 * (1 - u) * (1 - u) / trajectory.seconds;
    // 三个轴使用同一个进度，各自乘本轴带方向的行程，得到参考角和参考角速度。
    JointReference reference;
    for (int axis = 0; axis < 3; ++axis)
    {
        double travelDeg = trajectory.target[axis] - trajectory.start[axis];
        reference.position[axis] = trajectory.start[axis] + travelDeg * progress;
        reference.velocity[axis] = travelDeg * progressPerSecond;
    }
    return reference;
}
} // namespace scara
