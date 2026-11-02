#include "ik.hpp"
#include <algorithm>
#include <cmath>

namespace scara {

bool inverse(const Pose& target, bool positiveElbow, Joints& joints)
{
    if (!std::isfinite(target.x) || !std::isfinite(target.y) ||
        !std::isfinite(target.z) || !std::isfinite(target.yaw))
        return false;

    const double r = std::hypot(target.x, target.y);
    // 仅容纳浮点舍入误差；不能把真正不可达的目标强行截到边界。
    constexpr double tolerance = 1e-9; // mm
    if (r < std::abs(L1 - L2) - tolerance || r > L1 + L2 + tolerance)
        return false;

    const double c = (r * r - L1 * L1 - L2 * L2) / (2.0 * L1 * L2);
    double elbow = std::acos(std::clamp(c, -1.0, 1.0));
    if (!positiveElbow)
        elbow = -elbow;

    // 目标方向减去连杆三角形的夹角，得到肩角。
    const double shoulder = std::remainder(
        std::atan2(target.y, target.x) -
        std::atan2(L2 * std::sin(elbow), L1 + L2 * std::cos(elbow)),
        2.0 * Pi);
    const double yaw = std::remainder(target.yaw, 2.0 * Pi);
    const double wrist = std::remainder(yaw - shoulder - elbow, 2.0 * Pi);
    joints = {shoulder, elbow, target.z, wrist};
    return true;
}

} // namespace scara
