// Release构建也必须执行断言及其中的算法调用。
#ifdef NDEBUG
#undef NDEBUG
#endif
#include "fk.hpp"
#include "ik.hpp"
#include "homography_ekf.hpp"
#include "motion_plan.hpp"
#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <limits>
#include <random>
#include <string>
#include <vector>

namespace {

bool near(double a, double b, double tolerance = 1e-8)
{
    return std::abs(a - b) <= tolerance;
}

void checkPose(const scara::Pose& actual, const scara::Pose& expected)
{
    assert(near(actual.x, expected.x));
    assert(near(actual.y, expected.y));
    assert(near(actual.z, expected.z));
    assert(near(std::remainder(actual.yaw - expected.yaw, 2 * scara::Pi), 0));
}

// 用独立的Cholesky分解检查协方差对称、有限和半正定，不复写EKF更新公式。
void checkCovariance(const std::array<double, 64>& covariance)
{
    double lower[8][8] = {};
    for (int row = 0; row < 8; ++row) {
        for (int column = 0; column <= row; ++column) {
            assert(std::isfinite(covariance[row * 8 + column]));
            assert(near(covariance[row * 8 + column], covariance[column * 8 + row], 1e-12));
            double value = covariance[row * 8 + column];
            for (int k = 0; k < column; ++k) value -= lower[row][k] * lower[column][k];
            if (row == column) {
                assert(value >= -1e-12);
                lower[row][column] = std::sqrt(std::max(value, 1e-20));
            } else {
                lower[row][column] = value / lower[column][column];
            }
        }
    }
}

scara::MotionConfig previewConfig()
{
    // 仅供软件测试的假设限位和速度，不是用户机械臂的标定参数。
    scara::MotionConfig config;
    config.minimum = {-2 * scara::Pi, -scara::Pi, 0, -2 * scara::Pi};
    config.maximum = {2 * scara::Pi, scara::Pi, 160, 2 * scara::Pi};
    config.maxSpeed = {0.8, 0.7, 60, 1.0};
    config.safeZ = 100;
    config.cartesianSpeed = 60;
    config.yawSpeed = 0.6;
    config.cartesianStep = 1;
    config.angularStep = 0.1;
    config.toolDelay = 0.3;
    return config;
}

void checkPlan(const scara::Joints& start, const scara::MotionConfig& config,
               const scara::MotionPlan& plan)
{
    auto previous = start;
    for (const auto& step : plan.steps) {
        checkPose(scara::forward(step.joints), step.pose);
        const auto previousPose = scara::forward(previous);
        const double travel = std::hypot(std::hypot(step.pose.x - previousPose.x,
                                                    step.pose.y - previousPose.y),
                                         step.pose.z - previousPose.z);
        assert(travel <= config.cartesianSpeed * step.seconds + 1e-8);
        assert(std::abs(std::remainder(step.pose.yaw - previousPose.yaw, 2 * scara::Pi))
               <= config.yawSpeed * step.seconds + 1e-8);
        const double changes[] = {step.joints.shoulder - previous.shoulder,
            step.joints.elbow - previous.elbow, step.joints.z - previous.z,
            step.joints.wrist - previous.wrist};
        const double speeds[] = {config.maxSpeed.shoulder, config.maxSpeed.elbow,
                                 config.maxSpeed.z, config.maxSpeed.wrist};
        assert(step.seconds >= 0 && std::isfinite(step.seconds));
        for (int axis = 0; axis < 4; ++axis) {
            assert(std::abs(changes[axis]) <= speeds[axis] * step.seconds + 1e-8);
            if (axis != 2) assert(std::abs(changes[axis]) <= config.angularStep + 1e-8);
        }
        previous = step.joints;
    }
}

} // namespace

void testAlgorithms()
{
    // 两个肘分支、不同方位、Z与末端朝向均需可往返；失败不能改写输出。
    for (double shoulder : {-2.0, -0.4, 0.0, 1.3, 2.9}) {
        for (double elbow : {-2.9, -1.2, 0.0, 0.7, 2.9}) {
            const scara::Pose expected = scara::forward({shoulder, elbow, 32, 0.4});
            for (bool branch : {true, false}) {
                scara::Joints joints;
                assert(scara::inverse(expected, branch, joints));
                checkPose(scara::forward(joints), expected);
            }
        }
    }
    scara::Joints unchanged = {1, 2, 3, 4};
    assert(!scara::inverse({79, 0, 0, 0}, true, unchanged));
    assert(!scara::inverse({471, 0, 0, 0}, true, unchanged));
    assert(!scara::inverse({std::numeric_limits<double>::quiet_NaN(), 0, 0, 0}, true, unchanged));
    assert(unchanged.shoulder == 1 && unchanged.elbow == 2 && unchanged.z == 3 && unchanged.wrist == 4);

    // 解析Jacobian必须与非线性投影的中心差分相符。
    const std::array<double, 8> truth = {0.75, 0.07, 0.10, -0.04, 0.80, 0.12, 0.10, -0.08};
    for (const auto& point : std::vector<std::array<double, 2>>{{0, 0}, {0.4, 0.7}, {1, 1}}) {
        std::array<double, 2> projected;
        std::array<double, 16> jacobian;
        assert(scara::projectHomography(truth, point[0], point[1], projected, &jacobian));
        for (int parameter = 0; parameter < 8; ++parameter) {
            auto plus = truth, minus = truth;
            plus[parameter] += 1e-6;
            minus[parameter] -= 1e-6;
            std::array<double, 2> above, below;
            assert(scara::projectHomography(plus, point[0], point[1], above));
            assert(scara::projectHomography(minus, point[0], point[1], below));
            for (int coordinate = 0; coordinate < 2; ++coordinate)
                assert(near(jacobian[coordinate * 8 + parameter], (above[coordinate] - below[coordinate]) / 2e-6, 1e-7));
        }
    }

    scara::HomographyEkf filter;
    std::array<double, 9> matrix{};
    assert(!scara::homographyMatrix(filter, matrix));
    for (int i = 0; i < 8; ++i) matrix[i] = truth[i];
    matrix[8] = 1;
    matrix[2] += 0.01;  // 故意偏离真值初始化，不能靠已知真值冒充降噪。
    assert(scara::initializeHomography(filter, matrix, 0.001));
    assert(!scara::predictHomography(filter, -1, 0.001));
    std::mt19937 random(20261008);
    std::normal_distribution<double> noise(0, 0.003);
    double measuredError = 0, filteredError = 0;
    int accepted = 0;
    for (int frame = 0; frame < 100; ++frame) {
        assert(scara::predictHomography(filter, 1.0 / 30, 1e-8));
        std::vector<scara::HomographyObservation> points;
        for (double centerY : {0.06, 0.94}) {
            for (double centerX : {0.06, 0.94}) {
                for (double dy : {-0.02, 0.02}) {
                    for (double dx : {-0.02, 0.02}) {
                        std::array<double, 2> exact;
                        assert(scara::projectHomography(truth, centerX + dx, centerY + dy, exact));
                        points.push_back({centerX + dx, centerY + dy, exact[0] + noise(random), exact[1] + noise(random)});
                    }
                }
            }
        }
        accepted += scara::updateHomography(filter, points, 9e-6);
        checkCovariance(filter.covariance);
        if (frame >= 20) {
            for (const auto& point : points) {
                std::array<double, 2> exact, estimate;
                assert(scara::projectHomography(truth, point.x, point.y, exact));
                assert(scara::projectHomography(filter.state, point.x, point.y, estimate));
                measuredError += std::pow(point.u - exact[0], 2) + std::pow(point.v - exact[1], 2);
                filteredError += std::pow(estimate[0] - exact[0], 2) + std::pow(estimate[1] - exact[1], 2);
            }
        }
    }
    assert(accepted > 1400);
    assert(filteredError < measuredError * 0.35);
    const auto beforeOutlier = filter;
    assert(!scara::updateHomography(filter, {0.5, 0.5, 8, -8}, 9e-6));
    assert(!scara::updateHomography(filter, {0, 0, 0, std::numeric_limits<double>::infinity()}, 9e-6));
    assert(filter.state == beforeOutlier.state && filter.covariance == beforeOutlier.covariance);
    assert(scara::updateHomography(filter, std::vector<scara::HomographyObservation>{}, 9e-6) == 0);
    assert(scara::predictHomography(filter, 0.5, 2e-5));
    assert(filter.state == beforeOutlier.state);
    for (int i = 0; i < 8; ++i)
        assert(near(filter.covariance[i * 8 + i] - beforeOutlier.covariance[i * 8 + i], 1e-5, 1e-12));
    checkCovariance(filter.covariance);
    matrix[2] = truth[2];
    assert(scara::initializeHomography(filter, matrix, 0.02));
    assert(filter.state == truth && near(filter.covariance[0], 0.02));
    const auto reset = filter;
    matrix[8] = 0;
    assert(!scara::initializeHomography(filter, matrix, 0.1));
    assert(filter.state == reset.state && filter.covariance == reset.covariance);

    const auto config = previewConfig();
    scara::Joints start;
    assert(scara::inverse({0, 250, 100, 0}, true, start));
    scara::MotionPlan plan;
    std::string error;
    assert(!scara::planTrajectory(start, {{0, 250, 10, 0}}, {}, plan, error));
    assert(plan.steps.empty() && !plan.calibrationConfirmed && !error.empty());
    assert(scara::planPickPlace(start, {180, 90, 10, 0}, {-180, 410, 10, 0}, config, plan, error));
    assert(!plan.calibrationConfirmed);
    checkPlan(start, config, plan);
    int on = 0, off = 0;
    bool carrying = false;
    auto previousPose = scara::forward(start);
    for (const auto& step : plan.steps) {
        if (std::hypot(step.pose.x - previousPose.x, step.pose.y - previousPose.y) > 1e-8) {
            assert(previousPose.z >= config.safeZ - 1e-8 && step.pose.z >= config.safeZ - 1e-8);
        }
        if (step.tool == scara::ToolAction::VacuumOn) {
            ++on;
            carrying = true;
            checkPose(step.pose, {180, 90, 10, 0});
        } else if (step.tool == scara::ToolAction::VacuumOff) {
            ++off;
            assert(carrying);
            carrying = false;
            checkPose(step.pose, {-180, 410, 10, 0});
        }
        previousPose = step.pose;
    }
    assert(on == 1 && off == 1 && !carrying);
    checkPose(plan.steps.back().pose, {-180, 410, 100, 0});

    // 当前讨论的40mm象棋预览布局只出现在测试中，不写死进算法。
    for (int row = 0; row < 10; ++row) {
        for (int col = 0; col < 9; ++col) {
            assert(scara::planTrajectory(start, {{(4.5 - row) * 40, 90.0 + col * 40, 10, 0}}, config, plan, error));
            checkPlan(start, config, plan);
        }
    }
    assert(!scara::planTrajectory(start, {{500, 0, 20, 0}}, config, plan, error));
    assert(plan.steps.empty());
    assert(!scara::planTrajectory(start, {{0, 250, 200, 0}}, config, plan, error));
    assert(plan.steps.empty());
    scara::Joints oppositeStart;
    assert(scara::inverse({200, 0, 20, 0}, true, oppositeStart));
    assert(!scara::planTrajectory(oppositeStart, {{-200, 0, 20, 0}}, config, plan, error));

    // 经过±pi表示边界时仍须选择连续的关节圈数，不能突然反转整圈。
    const scara::Joints wrapStart = {scara::Pi - 0.03, -1.0, 20, 0.2};
    const scara::Joints wrapEnd = {scara::Pi + 0.03, -1.0, 20, 0.2};
    assert(scara::planTrajectory(wrapStart, {scara::forward(wrapEnd)}, config, plan, error));
    checkPlan(wrapStart, config, plan);
    assert(near(plan.steps.back().joints.shoulder, wrapEnd.shoulder));
    auto limited = config;
    limited.maximum.z = 15;
    assert(!scara::planTrajectory(start, {{0, 250, 10, 0}}, limited, plan, error));
    limited = config;
    limited.maxSpeed.elbow = 0;
    assert(!scara::planTrajectory(start, {}, limited, plan, error));
    assert(plan.steps.empty());
    limited = config;
    limited.minimum.shoulder = start.shoulder - 0.01;
    limited.maximum.shoulder = start.shoulder + 0.01;
    auto beyondShoulder = start;
    beyondShoulder.shoulder += 0.3;
    assert(!scara::planTrajectory(start, {scara::forward(beyondShoulder)}, limited, plan, error));
    assert(plan.steps.empty());
    auto confirmed = config;
    confirmed.calibrated = true;
    assert(scara::planTrajectory(start, {{0, 250, 20, 0}}, confirmed, plan, error));
    assert(plan.calibrationConfirmed);
}
