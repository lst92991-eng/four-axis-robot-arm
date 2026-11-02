#include "motion/kinematics.hpp"
#include "motion/filter.hpp"
#include "task/pick_place.hpp"
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>

namespace {
void check(bool value, const char* message)
{
    if (!value) throw std::runtime_error(std::string("motion test: ") + message);
}
template<class Function> void rejected(Function function)
{
    bool failed = false;
    try { function(); } catch (const std::exception&) { failed = true; }
    check(failed, "invalid scenario was accepted");
}
motion::Config fixture()
{
    // 以下数值只属于离线数学测试；不是任何实机的标定参数。
    motion::Config c;
    c.l1 = 195; c.l2 = 275;
    c.xMinimum = c.yMinimum = -600; c.xMaximum = c.yMaximum = 600;
    c.linkRadius = 8; c.toolRadius = 6; c.minimumSinElbow = .1;
    for (std::size_t i = 0; i < c.axes.size(); ++i) {
        auto& a = c.axes[i];
        a.node = unsigned(i + 1); a.direction = i == 2 ? -1 : 1;
        a.motorDegPerUnit = i == 0 ? 90 : 180 / motion::Pi * 5;
        a.zeroMotorDeg = 123; a.minimum = i == 0 ? 0 : -3;
        a.maximum = i == 0 ? 200 : 3; a.tolerance = i == 0 ? .1 : .001;
        a.maxRpm = 100; a.pulsesPerRev = 3200; a.currentLimitMa = 2000;
    }
    return c;
}
}

void testMotion()
{
    auto config = fixture();
    motion::validate(config);
    check(!config.calibrated, "offline fixture must not be marked calibrated");
    const motion::Joints start{50, .6, 1.2, -1.8};
    for (double shift : {-.2, 0., .2}) {
        auto q = start; q[1] += shift; q[2] -= shift / 2;
        const auto pose = motion::forward(q, config);
        const auto restored = motion::inverse(pose, q, config);
        for (std::size_t i = 0; i < q.size(); ++i) {
            check(std::abs(restored[i] - q[i]) < 1e-9, "FK/IK round trip");
            check(std::abs(motion::jointPosition(motion::motorAngle(q[i], config.axes[i]), config.axes[i]) - q[i]) < 1e-9,
                  "joint/motor direction and zero conversion");
        }
        motion::checkPath(q, restored, config);
    }
    auto bad = config; bad.axes[2].node = bad.axes[0].node;
    rejected([&] { motion::validate(bad); });
    bad = config; bad.axes[0].motorDegPerUnit = 0;
    rejected([&] { motion::validate(bad); });
    rejected([&] { motion::inverse({1000, 0, 50, 0}, start, config); });
    rejected([&] { motion::inverse({0, 0, 50, 0}, start, config); });
    rejected([&] { motion::inverse({std::numeric_limits<double>::quiet_NaN(), 0, 50, 0}, start, config); });
    auto end = start; end[1] += .4;
    motion::checkPath(start, end, config);
    const auto schedule = motion::timing(start, end, config);
    check(schedule.seconds > 0 && schedule.rpm[1] <= config.axes[1].maxRpm, "bounded common motion timing");
    bad = config; bad.moveTimeoutSeconds = .2; bad.settleSeconds = .01;
    rejected([&] { motion::checkPath(start, end, bad); });
    auto middle = start; middle[1] += .2;
    const auto blocker = motion::forward(middle, config);
    bad = config; bad.forbidden.push_back({blocker.x, blocker.y, 1, 0, 200});
    motion::checkPath(start, start, bad); motion::checkPath(end, end, bad);
    rejected([&] { motion::checkPath(start, end, bad); });
    end = start; end[2] = -start[2];
    rejected([&] { motion::checkPath(start, end, config); });
    bad = config; bad.xMaximum = motion::forward(start, config).x;
    rejected([&] { motion::checkPath(start, start, bad); });

    motion::Filter filter(config.l1, config.l2, config.accelerationNoise, config.encoderSigma);
    filter.reset({start[1], start[2]});
    auto tcpPose = motion::forward(start, config);
    cv::Point2d tcp(tcpPose.x, tcpPose.y);
    const auto estimate = filter.update({start[1], start[2]}, .02, &tcp, .01, .5, .1);
    check(filter.usedXY(), "fresh independent XY must enter EKF");
    check(cv::norm(estimate - cv::Vec2d(start[1], start[2])) < 1e-9, "zero residual changed state");
    filter.update({start[1], start[2]}, .02, &tcp, .2, .5, .1);
    check(!filter.usedXY(), "stale XY must leave encoder-only KF");
    tcp.x += 1000;
    filter.update({start[1], start[2]}, .02, &tcp, 0, .5, .1);
    check(!filter.usedXY(), "outlier XY must be rejected");
    rejected([&] { filter.update({start[1], start[2]}, 0, nullptr, 0, .5, .1); });

    task::Config operation;
    operation.travelZ = 80; operation.pickupWait = .2; operation.releaseWait = .2;
    operation.maximumHoldSeconds = 40;
    operation.park = motion::forward(start, config); operation.park.z = 90;
    task::Request request;
    request.source = motion::forward(start, config); request.source.z = 20;
    end = start; end[1] += .08;
    request.destination = motion::forward(end, config); request.destination.z = 20;
    const auto normal = task::plan(request, start, operation, config);
    int on = 0, off = 0;
    for (const auto& step : normal) {
        on += step.action == task::Action::MagnetOn; off += step.action == task::Action::MagnetOff;
    }
    check(on == 1 && off == 1, "normal transfer magnet order");
    request.capture = true;
    end[1] += .08; request.captureSlot = motion::forward(end, config); request.captureSlot.z = 20;
    const auto capture = task::plan(request, start, operation, config);
    on = 0; off = 0;
    for (const auto& step : capture) { on += step.action == task::Action::MagnetOn; off += step.action == task::Action::MagnetOff; }
    check(on == 2 && off == 2, "capture must remove target and move own piece");
    auto shortHold = operation; shortHold.maximumHoldSeconds = .21;
    rejected([&] { task::plan(request, start, shortHold, config); });
    request.source.x = 1000;
    rejected([&] { task::plan(request, start, operation, config); }); // 后半失败也必须在开磁前发现。
}
