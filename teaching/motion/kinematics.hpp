#pragma once
#include <array>
#include <vector>

namespace motion {

constexpr double Pi = 3.14159265358979323846;
// 顺序固定为Z、肩、肘、腕；Z为mm，其余为rad。
using Joints = std::array<double, 4>;
struct Pose { double x = 0, y = 0, z = 0, yaw = 0; };
struct Axis {
    unsigned node = 0;
    int direction = 1;
    double motorDegPerUnit = 0, zeroMotorDeg = 0;
    double minimum = 0, maximum = 0, tolerance = 0;
    unsigned maxRpm = 0, pulsesPerRev = 0, currentLimitMa = 0;
};
struct Cylinder {
    double x = 0, y = 0, radius = 0;
    double zMinimum = 0, zMaximum = 0;
};
struct Config {
    std::array<Axis, 4> axes{};
    double l1 = 0, l2 = 0;
    double xMinimum = 0, xMaximum = 0, yMinimum = 0, yMaximum = 0;
    double linkRadius = 0, toolRadius = 0, minimumSinElbow = 0;
    double sampleAngle = .02;
    double moveTimeoutSeconds = 15, settleSeconds = .15, feedbackSeconds = .03;
    double estimateErrorLimit = .05, accelerationNoise = 15, encoderSigma = .004;
    double tcpSigmaMm = .5, maximumObservationAge = .1;
    unsigned acceleration = 1;
    bool calibrated = false;
    // 关节限位须包含实测挡块/自碰限制；圆柱是另加的固定障碍。
    std::vector<Cylinder> forbidden;
};

void validate(const Config& config);
Pose forward(const Joints& joints, const Config& config);
Joints inverse(const Pose& pose, const Joints& nearby, const Config& config);
double motorAngle(double joint, const Axis& axis);
double jointPosition(double motorDeg, const Axis& axis);
struct Timing { std::array<unsigned, 4> rpm{}; double seconds = 0; };
Timing timing(const Joints& from, const Joints& to, const Config& config);
// 检查整段各轴单调运动可能覆盖的关节区间，含采样间几何膨胀余量。
// 它是配置几何包络检查，不是SolidWorks实体碰撞或实物净空认证。
void checkPath(const Joints& from, const Joints& to, const Config& config);

} // namespace motion
