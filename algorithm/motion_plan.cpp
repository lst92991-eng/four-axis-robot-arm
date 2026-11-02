#include "motion_plan.hpp"
#include "ik.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <utility>

namespace scara {
namespace {

std::array<double, 4> jointValues(const Joints& joints)
{
    return {joints.shoulder, joints.elbow, joints.z, joints.wrist};
}

bool validConfiguration(const Joints& start, const MotionConfig& config, std::string& error)
{
    const auto low = jointValues(config.minimum);
    const auto high = jointValues(config.maximum);
    const auto speed = jointValues(config.maxSpeed);
    const auto position = jointValues(start);
    for (int axis = 0; axis < 4; ++axis) {
        if (!std::isfinite(low[axis]) || !std::isfinite(high[axis]) || low[axis] > high[axis] ||
            !std::isfinite(speed[axis]) || speed[axis] <= 0 || !std::isfinite(position[axis]) ||
            position[axis] < low[axis] || position[axis] > high[axis]) {
            error = "关节限位、最大速度或起始位置无效；需要显式配置。";
            return false;
        }
    }
    for (double value : {config.cartesianSpeed, config.yawSpeed,
                         config.cartesianStep, config.angularStep}) {
        if (!std::isfinite(value) || value <= 0) {
            error = "笛卡尔速度、转角速度和采样步长必须为正数。";
            return false;
        }
    }
    if (config.angularStep > Pi / 2 || !std::isfinite(config.safeZ) ||
        config.safeZ < config.minimum.z || config.safeZ > config.maximum.z ||
        !std::isfinite(config.toolDelay) || config.toolDelay < 0) {
        error = "通行高度、气路等待时间或关节采样步长无效。";
        return false;
    }
    return true;
}

bool appendMove(Joints& current, const Pose& target, const MotionConfig& config,
                std::vector<PlanStep>& steps, std::string& error)
{
    Joints endpoint;
    if (!inverse(target, true, endpoint)) {
        error = "目标坐标非有限或超出195/275mm双连杆的可达范围。";
        return false;
    }
    const Pose origin = forward(current);
    const double dx = target.x - origin.x, dy = target.y - origin.y;
    const double dz = target.z - origin.z;
    const double yawChange = std::remainder(target.yaw - origin.yaw, 2 * Pi);

    // 两个端点可达不代表直线可达；整条XY线段不能穿过内侧80mm空区。
    const double lengthSquared = dx * dx + dy * dy;
    const double closest = lengthSquared > 0
        ? std::clamp(-(origin.x * dx + origin.y * dy) / lengthSquared, 0.0, 1.0) : 0;
    if (std::hypot(origin.x + closest * dx, origin.y + closest * dy) < std::abs(L1 - L2) - 1e-8) {
        error = "直线路径穿过机械臂内侧不可达区域。";
        return false;
    }
    const double distance = std::hypot(std::hypot(dx, dy), dz);
    const double count = std::max({1.0, std::ceil(distance / config.cartesianStep),
                                  std::ceil(std::abs(yawChange) / config.angularStep)});
    if (!std::isfinite(count) || count + steps.size() > 10000) {
        error = "轨迹采样点过多，请检查步长和目标坐标。";
        return false;
    }
    const auto low = jointValues(config.minimum);
    const auto high = jointValues(config.maximum);
    const auto speed = jointValues(config.maxSpeed);
    Pose previousPose = origin;
    for (int sample = 1; sample <= static_cast<int>(count); ++sample) {
        const double fraction = sample / count;
        const Pose pose = {origin.x + dx * fraction, origin.y + dy * fraction,
                           origin.z + dz * fraction, origin.yaw + yawChange * fraction};
        const auto previous = jointValues(current);
        std::array<double, 4> selected{};
        double bestCost = std::numeric_limits<double>::infinity();

        for (bool positiveElbow : {true, false}) {
            Joints candidate;
            if (!inverse(pose, positiveElbow, candidate)) continue;
            auto values = jointValues(candidate);
            bool allowed = true;
            double cost = 0;
            for (int axis = 0; axis < 4; ++axis) {
                if (axis != 2) {
                    // 角度先加减整圈贴近上一个位置，避免跨±pi时出现假大跳变。
                    const double firstTurn = std::ceil((low[axis] - values[axis]) / (2 * Pi));
                    const double lastTurn = std::floor((high[axis] - values[axis]) / (2 * Pi));
                    if (firstTurn > lastTurn) { allowed = false; break; }
                    const double turn = std::clamp(std::round((previous[axis] - values[axis]) / (2 * Pi)),
                                                   firstTurn, lastTurn);
                    values[axis] += turn * 2 * Pi;
                }
                const double change = values[axis] - previous[axis];
                if (!std::isfinite(values[axis]) || values[axis] < low[axis] - 1e-9 ||
                    values[axis] > high[axis] + 1e-9 ||
                    (axis != 2 && std::abs(change) > config.angularStep + 1e-9)) {
                    allowed = false;
                    break;
                }
                if (axis != 2) cost += change * change;
            }
            if (allowed && cost < bestCost) { bestCost = cost; selected = values; }
        }
        if (!std::isfinite(bestCost)) {
            error = "没有满足关节限位且连续的逆解；请核对限位或缩小笛卡尔步长。";
            return false;
        }
        // 每个离散段按最慢的一条约束确定时间，不能独立给各轴相同速度。
        double seconds = std::hypot(std::hypot(pose.x - previousPose.x, pose.y - previousPose.y),
                                     pose.z - previousPose.z) / config.cartesianSpeed;
        seconds = std::max(seconds, std::abs(pose.yaw - previousPose.yaw) / config.yawSpeed);
        for (int axis = 0; axis < 4; ++axis)
            seconds = std::max(seconds, std::abs(selected[axis] - previous[axis]) / speed[axis]);
        if (!std::isfinite(seconds)) { error = "轨迹时间计算溢出。"; return false; }
        current = {selected[0], selected[1], selected[2], selected[3]};
        if (seconds > 1e-12) steps.push_back({pose, current, seconds, ToolAction::Keep});
        previousPose = pose;
    }
    return true;
}

} // namespace

bool planTrajectory(const Joints& start, const std::vector<Pose>& targets,
                    const MotionConfig& config, MotionPlan& plan, std::string& error)
{
    plan = {};
    error.clear();
    if (!validConfiguration(start, config, error)) return false;
    Joints current = start;
    std::vector<PlanStep> steps;
    for (const auto& target : targets) {
        if (!appendMove(current, target, config, steps, error)) return false;
    }
    plan.steps = std::move(steps);
    plan.calibrationConfirmed = config.calibrated;
    return true;
}

bool planPickPlace(const Joints& start, const Pose& pick, const Pose& place,
                   const MotionConfig& config, MotionPlan& plan, std::string& error)
{
    plan = {};
    error.clear();
    if (!validConfiguration(start, config, error)) return false;
    if (!std::isfinite(pick.z) || !std::isfinite(place.z) ||
        config.safeZ < pick.z || config.safeZ < place.z) {
        error = "通行高度不能低于取子或放子高度。";
        return false;
    }
    Joints current = start;
    const Pose origin = forward(start);
    const double travelZ = std::max(config.safeZ, origin.z);
    const Pose startAbove = {origin.x, origin.y, travelZ, origin.yaw};
    const Pose pickAbove = {pick.x, pick.y, travelZ, pick.yaw};
    const Pose placeAbove = {place.x, place.y, travelZ, place.yaw};
    std::vector<PlanStep> steps;
    if (!appendMove(current, startAbove, config, steps, error) ||
        !appendMove(current, pickAbove, config, steps, error) ||
        !appendMove(current, pick, config, steps, error)) return false;
    steps.push_back({forward(current), current, config.toolDelay, ToolAction::VacuumOn});
    if (!appendMove(current, pickAbove, config, steps, error) ||
        !appendMove(current, placeAbove, config, steps, error) ||
        !appendMove(current, place, config, steps, error)) return false;
    steps.push_back({forward(current), current, config.toolDelay, ToolAction::VacuumOff});
    if (!appendMove(current, placeAbove, config, steps, error)) return false;
    plan.steps = std::move(steps);
    plan.calibrationConfirmed = config.calibrated;
    return true;
}

} // namespace scara
