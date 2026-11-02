#include "motion.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>
#include <thread>

namespace motion {

double seconds()
{
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

Motion::Motion(robot::Can& can, std::array<robot::Motor*, 4> motors, const Config& config)
    : can_(can), motors_(motors), config_(config),
      filter_(config.l1, config.l2, config.accelerationNoise, config.encoderSigma)
{
    validate(config_);
    for (std::size_t i = 0; i < motors_.size(); ++i)
        if (!motors_[i] || motors_[i]->node() != config_.axes[i].node)
            throw std::runtime_error("四轴电机对象与关节地址映射不符");
}

void Motion::prepare(const Cancelled& cancelled)
{
    if (!config_.calibrated) throw std::runtime_error("机械零偏、节点、行程和工作区尚未确认标定");
    ready_ = false;
    try {
        for (auto* motor : motors_) {
            if (cancelled && cancelled()) throw std::runtime_error("准备已取消");
            motor->prepare();
        }
        sample(false);
        for (const auto& feedback : feedback_)
            if (feedback.rpm != 0) throw std::runtime_error("准备时检测到轴未静止");
        checkPath(joints_, joints_, config_);
        filter_.reset({joints_[1], joints_[2]});
        lastObservation_ = 0;
        ready_ = true;
    } catch (...) { stop(); throw; }
}

Joints Motion::sample(bool estimate)
{
    const double begin = seconds();
    for (std::size_t i = 0; i < motors_.size(); ++i) {
        feedback_[i] = motors_[i]->read();
        joints_[i] = jointPosition(feedback_[i].angleDeg, config_.axes[i]);
        if (feedback_[i].currentMa > config_.axes[i].currentLimitMa)
            throw std::runtime_error("电机采样相电流超过本轴配置阈值");
        if (!std::isfinite(joints_[i]) || joints_[i] < config_.axes[i].minimum || joints_[i] > config_.axes[i].maximum)
            throw std::runtime_error("实际编码器位置超出本轴机械限位");
    }
    double now = seconds();
    if (now - begin > .25) throw std::runtime_error("四轴反馈轮询过慢，无法维持失联保护期限");
    const double dt = now - lastSample_;
    if (estimate) {
        if (!active_ && dt > .3) filter_.reset({joints_[1], joints_[2]});
        else {
            TcpObservation observation;
            if (observer_) observation = observer_(joints_[0]); // 只读取最新缓存，调用者不能在此阻塞采图。
            now = seconds();
            if (now - begin > .25) throw std::runtime_error("末端观测阻塞了电机反馈");
            const bool fresh = observation.valid && std::isfinite(observation.stamp) &&
                observation.stamp > lastObservation_ && observation.stamp <= now &&
                now - observation.stamp <= config_.maximumObservationAge;
            const auto filtered = filter_.update({joints_[1], joints_[2]}, dt,
                fresh ? &observation.xy : nullptr, now - observation.stamp,
                config_.tcpSigmaMm, config_.maximumObservationAge);
            if (fresh) lastObservation_ = observation.stamp;
            if (filter_.usedXY()) ++xyUpdates_;
            // 估计用于反馈监督；不以滤波结果覆盖原始限位/到位判据，也不偷偷改变已检查路径。
            if (std::abs(filtered[0] - joints_[1]) > config_.estimateErrorLimit ||
                std::abs(filtered[1] - joints_[2]) > config_.estimateErrorLimit)
                throw std::runtime_error("运动估计与编码器差异超过配置限值");
        }
    }
    lastSample_ = now;
    return joints_;
}

Joints Motion::read()
{
    if (!ready_) throw std::runtime_error("请先准备四轴；故障后不继续维持旧运动");
    try { return sample(true); }
    catch (...) { stop(); throw; }
}

void Motion::move(const Pose& target, const Cancelled& cancelled)
{
    if (!ready_ || active_) throw std::runtime_error("四轴未准备或上次运动尚未结束");
    if (cancelled && cancelled()) throw std::runtime_error("运动已取消");
    read();
    for (const auto& feedback : feedback_)
        if (feedback.rpm != 0) throw std::runtime_error("上段尚未停稳，不能排入新目标");
    const Joints goal = inverse(target, joints_, config_);
    checkPath(joints_, goal, config_);
    // Emm每(256-acc)*50us变化1RPM；选择共同目标耗时，整数RPM量化后仍以反馈判完成。
    const auto profile = timing(joints_, goal, config_);
    if (profile.seconds < 1e-9) {
        for (const auto& feedback : feedback_)
            if (!(feedback.status & 2)) throw std::runtime_error("零位移目标仍未获得驱动到位状态");
        return;
    }
    const double begin = seconds();
    active_ = true;
    try {
        for (std::size_t i = 0; i < goal.size(); ++i) {
            if (cancelled && cancelled()) throw std::runtime_error("同步准备已取消");
            motors_[i]->move(motorAngle(goal[i], config_.axes[i]), profile.rpm[i], uint8_t(config_.acceleration), true);
        }
        if (seconds() - begin > .25) throw std::runtime_error("四轴缓存应答过慢，取消同步启动");
        if (cancelled && cancelled()) throw std::runtime_error("同步启动已取消");
        filter_.reset({joints_[1], joints_[2]});
        lastSample_ = seconds();
        robot::Motor::synchronize(can_); // 只有全部缓存命令已ACK，才发一次共同触发。
        double stableSince = 0;
        while (1) {
            if (cancelled && cancelled()) throw std::runtime_error("运动已取消");
            if (seconds() - begin >= config_.moveTimeoutSeconds) throw std::runtime_error("四轴到位超时");
            sample(true);
            checkPath(joints_, joints_, config_);
            bool arrived = true;
            for (std::size_t i = 0; i < goal.size(); ++i)
                arrived = arrived && std::abs(joints_[i] - goal[i]) <= config_.axes[i].tolerance &&
                    feedback_[i].rpm == 0 && (feedback_[i].status & 2);
            if (!arrived) stableSince = 0;
            else if (!stableSince) stableSince = seconds();
            else if (seconds() - stableSince >= config_.settleSeconds) break;
            std::this_thread::sleep_for(std::chrono::duration<double>(config_.feedbackSeconds));
        }
        active_ = false;
    } catch (...) { stop(); throw; }
}

bool Motion::stop() noexcept
{
    ready_ = false;
    bool stopped = true;
    // 先把停止请求送给所有轴，不能因等待第一轴确认而耽误其余轴停止。
    for (auto* motor : motors_) {
        try { motor->requestStop(); } catch (...) { stopped = false; }
    }
    for (auto* motor : motors_) {
        try { motor->stop(); } catch (...) { stopped = false; }
    }
    active_ = false;
    return stopped;
}

} // namespace motion
