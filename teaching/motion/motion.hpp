#pragma once
#include "kinematics.hpp"
#include "filter.hpp"
#include "robot/motor.hpp"
#include <functional>
#include <utility>

namespace motion {

struct TcpObservation { cv::Point2d xy; double stamp = 0; bool valid = false; };
using Cancelled = std::function<bool()>;
using TcpObserver = std::function<TcpObservation(double actualZmm)>;
double seconds(); // steady_clock秒，与视觉测量时间戳相同时间基准。

class Motion {
public:
    Motion(robot::Can& can, std::array<robot::Motor*, 4> motors, const Config& config);
    void prepare(const Cancelled& cancelled = {});
    Joints read();
    void move(const Pose& target, const Cancelled& cancelled = {});
    bool stop() noexcept;
    const Joints& state() const { return joints_; }
    const Config& config() const { return config_; }
    Pose pose() const { return forward(joints_, config_); }
    bool usedXY() const { return filter_.usedXY(); }
    std::size_t xyUpdates() const { return xyUpdates_; }
    void setObserver(TcpObserver observer) { observer_ = std::move(observer); }
private:
    Joints sample(bool estimate);
    robot::Can& can_;
    std::array<robot::Motor*, 4> motors_;
    Config config_;
    Filter filter_;
    TcpObserver observer_;
    std::array<robot::MotorFeedback, 4> feedback_{};
    Joints joints_{};
    double lastSample_ = 0, lastObservation_ = 0;
    bool ready_ = false, active_ = false;
    std::size_t xyUpdates_ = 0;
};

} // namespace motion
