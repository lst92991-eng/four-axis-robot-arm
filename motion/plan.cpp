#include "plan.hpp"
#include "vision/io.hpp"
#include <algorithm>
#include <cmath>

namespace scara
{
constexpr double RadiansPerDegree = CV_PI / 180.0;
constexpr double DegreesPerRadian = 180.0 / CV_PI;

//---------参数读取区---------
bool Arm_load(const std::string &path, ArmGeometry &arm)
{
    cv::FileStorage file;
    cv::FileNode lengths;
    bool valid;

    arm = {};
    readStorage(path, file);
    lengths = file["arm_lengths_mm"];
    if (!lengths.isSeq() || lengths.size() != 2) return false;
    for (const char *key : {"j2_reference_deg", "j3_reference_motor_deg", "j2_scale", "j3_ratio", "j2_sign", "j3_sign",
                            "shoulder_ref_rad", "elbow_ref_rad"})
        if (!file[key].isInt() && !file[key].isReal()) return false;
    arm.upperMm = double(lengths[0]);
    arm.forearmMm = double(lengths[1]);
    arm.j2ReferenceDeg = double(file["j2_reference_deg"]);
    arm.j3ReferenceMotorDeg = double(file["j3_reference_motor_deg"]);
    arm.j2Scale = double(file["j2_scale"]);
    arm.j3Ratio = double(file["j3_ratio"]);
    arm.j2Sign = double(file["j2_sign"]);
    arm.j3Sign = double(file["j3_sign"]);
    arm.shoulderReferenceRad = double(file["shoulder_ref_rad"]);
    arm.elbowReferenceRad = double(file["elbow_ref_rad"]);
    valid = arm.upperMm > 0 && arm.forearmMm > 0 && arm.j2Scale > 0 && arm.j3Ratio > 0 && std::abs(arm.j2Sign) == 1 &&
            std::abs(arm.j3Sign) == 1;
    if (valid == false) arm = {};
    return valid;
}

bool WorkHeights_load(const std::string &path, WorkHeights &heights)
{
    cv::FileStorage file;
    cv::FileNode z;
    //操作员估计的抬高量，不是丝杆导程。
    double estimatedRiseMm = 20;
    //释放层相对吸取层的估计高度差。
    double releaseOffsetMm = 0;
    //两个示教高度之间的J1角度差。
    double deltaDeg;
    bool valid;

    heights = {};
    readStorage(path, file);
    z = file["z"];
    for (const char *key : {"pickup_deg", "travel_deg"})
        if (!z[key].isInt() && !z[key].isReal()) return false;
    if (!file["release_pulse_ms"].empty() && !file["release_pulse_ms"].isInt()) return false;
    heights.pickupDeg = double(z["pickup_deg"]);
    heights.travelDeg = double(z["travel_deg"]);
    //未另外示教可横移高度时，先沿用已经验证的搬运层
    heights.clearanceDeg = z["clearance_deg"].empty() ? heights.travelDeg : double(z["clearance_deg"]);
    if (!file["release_pulse_ms"].empty()) heights.releasePulseMs = int(file["release_pulse_ms"]);
    if (!z["operator_estimated_rise_mm"].empty()) estimatedRiseMm = double(z["operator_estimated_rise_mm"]);
    if (!z["release_offset_mm"].empty()) releaseOffsetMm = double(z["release_offset_mm"]);
    deltaDeg = heights.travelDeg - heights.pickupDeg;
    heights.releaseDeg = heights.pickupDeg + deltaDeg / estimatedRiseMm * releaseOffsetMm;
    valid = std::isfinite(heights.pickupDeg) && std::isfinite(heights.travelDeg) &&
            std::isfinite(heights.releaseDeg) && std::isfinite(heights.clearanceDeg) &&
            deltaDeg != 0 && estimatedRiseMm > 0 && heights.releasePulseMs > 0 &&
            (heights.clearanceDeg - heights.pickupDeg) * deltaDeg > 0 &&
            (heights.travelDeg - heights.clearanceDeg) * deltaDeg >= 0 &&
            (heights.releaseDeg - heights.pickupDeg) * deltaDeg >= 0 &&
            (heights.clearanceDeg - heights.releaseDeg) * deltaDeg >= 0;
    if (!valid) heights = {};
    if (valid == false) heights = {};
    return valid;
}

//---------坐标正逆解区---------
void Arm_forward(const ArmGeometry &arm, const JointTarget &joints, cv::Point2d &baseMm)
{
    //大臂相对基座、小臂相对大臂的角度，弧度。
    double shoulder, elbow;

    shoulder =
        arm.shoulderReferenceRad + (joints.j2Deg - arm.j2ReferenceDeg) * arm.j2Scale * arm.j2Sign * RadiansPerDegree;
    elbow = arm.elbowReferenceRad +
            (joints.j3MotorDeg - arm.j3ReferenceMotorDeg) * arm.j3Sign / arm.j3Ratio * RadiansPerDegree;
    baseMm.x = arm.upperMm * std::cos(shoulder) + arm.forearmMm * std::cos(shoulder + elbow);
    baseMm.y = arm.upperMm * std::sin(shoulder) + arm.forearmMm * std::sin(shoulder + elbow);
}

bool Arm_inverse(const ArmGeometry &arm, cv::Point2d baseMm, JointTarget &joints)
{
    //目标到基座的距离平方，mm²。
    double radiusSquared;
    double cosine, shoulder, elbow;
    //相对已示教参考姿态的角度差，弧度。
    double shoulderDelta, elbowDelta;
    bool reachable;

    joints = {};
    radiusSquared = baseMm.x * baseMm.x + baseMm.y * baseMm.y;
    cosine =
        (radiusSquared - arm.upperMm * arm.upperMm - arm.forearmMm * arm.forearmMm) / (2 * arm.upperMm * arm.forearmMm);
    reachable = cosine >= -1 && cosine <= 1;
    if (reachable)
    {
        elbow = std::copysign(std::acos(cosine), arm.elbowReferenceRad);
        shoulder = std::atan2(baseMm.y, baseMm.x) -
                   std::atan2(arm.forearmMm * std::sin(elbow), arm.upperMm + arm.forearmMm * std::cos(elbow));
        shoulderDelta = std::remainder(shoulder - arm.shoulderReferenceRad, 2 * CV_PI);
        elbowDelta = std::remainder(elbow - arm.elbowReferenceRad, 2 * CV_PI);
        joints.j2Deg = arm.j2ReferenceDeg + shoulderDelta * DegreesPerRadian / (arm.j2Scale * arm.j2Sign);
        joints.j3MotorDeg = arm.j3ReferenceMotorDeg + elbowDelta * DegreesPerRadian * arm.j3Ratio / arm.j3Sign;
    }
    reachable = reachable && std::isfinite(joints.j2Deg) && std::isfinite(joints.j3MotorDeg);
    if (reachable == false) joints = {};
    return reachable;
}

//---------取放步骤区---------
bool Pick_plan(const ArmGeometry &arm, const WorkHeights &heights, cv::Point2d sourceMm, cv::Point2d destinationMm,
               bool alreadyHeld, std::vector<PickStep> &steps)
{
    //起点和终点的J2反馈角、J3电机角，度。
    JointTarget source, destination;
    //吸持状态。
    PickStep hold;
    //每个移动段统一给出J1/J2/J3的目标。
    PickStep move;
    //放棋脉冲，毫秒。
    PickStep pulse;

    steps.clear();
    if (heights.pickupDeg == heights.travelDeg) return false;
    if (!alreadyHeld && !Arm_inverse(arm, sourceMm, source)) return false;
    if (!Arm_inverse(arm, destinationMm, destination)) return false;

    hold.action = PickAction::Hold;
    steps.push_back(hold);

    //每步复制完整的三轴目标，三个分量共用同一条时间曲线
    move.action = PickAction::Move;
    if (!alreadyHeld)
    {
        move.target = {heights.travelDeg, source.j2Deg, source.j3MotorDeg};
        steps.push_back(move);
        move.target[0] = heights.pickupDeg;
        steps.push_back(move);
        //先垂直脱离棋子，达到已示教的可横移高度
        move.target[0] = heights.clearanceDeg;
        steps.push_back(move);
    }

    //这段同时抬升至搬运层并移动J2/J3
    move.target = {heights.travelDeg, destination.j2Deg, destination.j3MotorDeg};
    steps.push_back(move);
    move.target[0] = heights.releaseDeg;
    steps.push_back(move);
    // 先在释放层放棋，脉冲结束后抬升离开。
    pulse.action = PickAction::ReleasePulse;
    pulse.durationMs = heights.releasePulseMs;
    steps.push_back(pulse);
    //离开释放位置后，下一段可继续抬升并同时移到观察位
    move.target[0] = heights.clearanceDeg;
    steps.push_back(move);
    return true;
}

//---------联动轨迹区---------
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
        double distance = std::abs(target[axis] - start[axis]);
        double velocityTime = 1.875 * distance / limits.velocity[axis];
        double accelerationTime = std::sqrt((10 / std::sqrt(3.0)) * distance / limits.acceleration[axis]);
        seconds = std::max(seconds, std::max(velocityTime, accelerationTime));
    }
    if (!std::isfinite(seconds) || seconds > 3600) return false;
    trajectory.start = start;
    trajectory.target = target;
    trajectory.seconds = seconds;
    return true;
}

void Trajectory_sample(const JointTrajectory &trajectory, double elapsedSeconds, JointReference &reference)
{
    //u从0走到1；位置进度s=10u³-15u⁴+6u⁵，起止速度与加速度均为0
    double u = trajectory.seconds > 0 ? std::clamp(elapsedSeconds / trajectory.seconds, 0.0, 1.0) : 1;
    double s = u * u * u * (10 + u * (-15 + 6 * u));
    double ds = trajectory.seconds > 0 ? 30 * u * u * (1 - u) * (1 - u) / trajectory.seconds : 0;
    for (int axis = 0; axis < 3; ++axis)
    {
        double distance = trajectory.target[axis] - trajectory.start[axis];
        reference.position[axis] = u == 1 ? trajectory.target[axis] : trajectory.start[axis] + distance * s;
        reference.velocity[axis] = distance * ds;
    }
}
} // namespace scara
