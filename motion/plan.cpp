#include "plan.hpp"
#include "vision/io.hpp"
#include <cmath>

namespace scara
{
constexpr double RadiansPerDegree = CV_PI / 180.0;
constexpr double DegreesPerRadian = 180.0 / CV_PI;

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

void Arm_forward(const ArmGeometry &arm, const JointTarget &joints, cv::Point2d &baseMm)
{
    double shoulder, elbow; // 大臂相对基座、小臂相对大臂的角度，弧度。

    shoulder =
        arm.shoulderReferenceRad + (joints.j2Deg - arm.j2ReferenceDeg) * arm.j2Scale * arm.j2Sign * RadiansPerDegree;
    elbow = arm.elbowReferenceRad +
            (joints.j3MotorDeg - arm.j3ReferenceMotorDeg) * arm.j3Sign / arm.j3Ratio * RadiansPerDegree;
    baseMm.x = arm.upperMm * std::cos(shoulder) + arm.forearmMm * std::cos(shoulder + elbow);
    baseMm.y = arm.upperMm * std::sin(shoulder) + arm.forearmMm * std::sin(shoulder + elbow);
}

bool Arm_inverse(const ArmGeometry &arm, cv::Point2d baseMm, JointTarget &joints)
{
    double radiusSquared; // 目标到基座的距离平方，mm²。
    double cosine, shoulder, elbow;
    double shoulderDelta, elbowDelta; // 相对已示教参考姿态的角度差，弧度。
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

bool WorkHeights_load(const std::string &path, WorkHeights &heights)
{
    cv::FileStorage file;
    cv::FileNode z;
    double estimatedRiseMm = 20; // 操作员估计的抬高量，不是丝杆导程。
    double releaseOffsetMm = 0;  // 释放层相对吸取层的估计高度差。
    double deltaDeg;             // 两个示教高度之间的J1角度差。
    bool valid;

    heights = {};
    readStorage(path, file);
    z = file["z"];
    for (const char *key : {"pickup_deg", "travel_deg"})
        if (!z[key].isInt() && !z[key].isReal()) return false;
    if (!file["release_pulse_ms"].empty() && !file["release_pulse_ms"].isInt()) return false;
    heights.pickupDeg = double(z["pickup_deg"]);
    heights.travelDeg = double(z["travel_deg"]);
    if (!z["speed_deg_s"].empty()) heights.speedDegS = double(z["speed_deg_s"]);
    if (!file["release_pulse_ms"].empty()) heights.releasePulseMs = int(file["release_pulse_ms"]);
    if (!z["operator_estimated_rise_mm"].empty()) estimatedRiseMm = double(z["operator_estimated_rise_mm"]);
    if (!z["release_offset_mm"].empty()) releaseOffsetMm = double(z["release_offset_mm"]);
    deltaDeg = heights.travelDeg - heights.pickupDeg;
    heights.releaseDeg = heights.pickupDeg + deltaDeg / estimatedRiseMm * releaseOffsetMm;
    valid = std::isfinite(heights.pickupDeg) && std::isfinite(heights.travelDeg) && std::isfinite(heights.releaseDeg) &&
            deltaDeg != 0 && estimatedRiseMm > 0 && (heights.releaseDeg - heights.travelDeg) * deltaDeg <= 0 &&
            heights.releasePulseMs > 0 && std::isfinite(heights.speedDegS) && heights.speedDegS > 0;
    if (valid == false) heights = {};
    return valid;
}

bool Pick_plan(const ArmGeometry &arm, const WorkHeights &heights, cv::Point2d sourceMm, cv::Point2d destinationMm,
               bool alreadyHeld, std::vector<PickStep> &steps)
{
    JointTarget source, destination; // 起点和终点的J2反馈角、J3电机角，度。
    bool sourceValid = true, destinationValid;
    PickStep hold;   // 吸持状态。
    PickStep moveXY; // 平面移动的关节目标，度。
    PickStep moveZ;  // 示教高度对应的J1角度，度。
    PickStep pulse;  // 放棋脉冲，毫秒。

    steps.clear();
    if (heights.pickupDeg == heights.travelDeg) return false;
    if (!alreadyHeld) sourceValid = Arm_inverse(arm, sourceMm, source);
    destinationValid = Arm_inverse(arm, destinationMm, destination);
    if (sourceValid == false || destinationValid == false) return false;

    hold.action = PickAction::Hold;
    steps.push_back(hold);

    // 入列时复制本步目标，后续可填写新的关节角或高度。
    moveXY.action = PickAction::MoveXY;
    moveZ.action = PickAction::MoveZ;
    moveZ.speedDegS = heights.speedDegS;
    if (!alreadyHeld)
    {
        moveXY.joints = source;
        steps.push_back(moveXY);
        moveZ.zDeg = heights.pickupDeg;
        steps.push_back(moveZ);
        moveZ.zDeg = heights.travelDeg;
        steps.push_back(moveZ);
    }

    moveXY.joints = destination;
    steps.push_back(moveXY);
    moveZ.zDeg = heights.releaseDeg;
    steps.push_back(moveZ);
    // 先在释放层放棋，脉冲结束后抬升离开。
    pulse.action = PickAction::ReleasePulse;
    pulse.durationMs = heights.releasePulseMs;
    steps.push_back(pulse);
    moveZ.zDeg = heights.travelDeg;
    steps.push_back(moveZ);
    return true;
}
} // namespace scara
