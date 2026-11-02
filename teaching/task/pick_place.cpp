#include "pick_place.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>
#include <thread>

namespace task {

std::vector<Step> plan(const Request& request, const motion::Joints& start,
                       const Config& task, const motion::Config& arm)
{
    for (double value : {task.travelZ, task.pickupWait, task.releaseWait, task.maximumHoldSeconds})
        if (!std::isfinite(value)) throw std::runtime_error("取放参数不是有限数");
    if (task.pickupWait < 0 || task.releaseWait < 0 || task.maximumHoldSeconds <= task.pickupWait ||
        task.park.z < task.travelZ || task.travelZ < request.source.z || task.travelZ < request.destination.z ||
        (request.capture && task.travelZ < request.captureSlot.z))
        throw std::runtime_error("取放高度、停车位或电磁铁时间参数不正确");
    const double height = std::max(task.travelZ, start[0]);
    std::vector<Step> steps;
    motion::Joints current = start;
    bool holding = false;
    double heldSeconds = 0;
    auto move = [&](const motion::Pose& target) {
        const auto next = motion::inverse(target, current, arm);
        motion::checkPath(current, next, arm);
        if (holding) {
            heldSeconds += motion::timing(current, next, arm).seconds + arm.settleSeconds + .25;
            if (heldSeconds >= task.maximumHoldSeconds)
                throw std::runtime_error("预计带子运动超过电磁铁最长通电时间");
        }
        steps.push_back({Action::Move, target});
        current = next;
    };
    auto transfer = [&](const motion::Pose& source, const motion::Pose& destination) {
        auto above = motion::forward(current, arm); above.z = height;
        move(above);
        above = source; above.z = height; move(above);
        move(source);
        steps.push_back({Action::MagnetOn, source});
        holding = true; heldSeconds = task.pickupWait;
        move(above);
        above = destination; above.z = height; move(above);
        move(destination);
        steps.push_back({Action::MagnetOff, destination});
        holding = false;
        move(above);
    };
    // 清目标和搬己子作为同一完整计划预检；不在吃子执行一半才检查后段是否可达。
    if (request.capture) transfer(request.destination, request.captureSlot);
    transfer(request.source, request.destination);
    move(task.park);
    return steps;
}

PickPlace::PickPlace(motion::Motion& arm, robot::Magnet& magnet, const Config& config)
    : arm_(arm), magnet_(magnet), config_(config) {}

void PickPlace::wait(double duration, const motion::Cancelled& cancelled)
{
    const double deadline = motion::seconds() + duration;
    while (motion::seconds() < deadline) {
        if (magnet_.expired() || !magnet_.failure().empty() || (cancelled && cancelled()) ||
            (magnet_.on() && motion::seconds() - heldSince_ >= config_.maximumHoldSeconds))
            throw std::runtime_error("取放已取消或达到电磁铁最长通电时间");
        arm_.read();
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
}

void PickPlace::execute(const Request& request, const motion::Cancelled& cancelled)
{
    if (progress_.interrupted || progress_.carrying || magnet_.on() || magnet_.expired() || !magnet_.failure().empty())
        throw std::runtime_error("取放状态未核对为空，禁止重跑旧搬子任务");
    arm_.read();
    const auto steps = plan(request, arm_.state(), config_, arm_.config());
    progress_ = {};
    const auto stopRequested = [&] {
        return magnet_.expired() || !magnet_.failure().empty() || (cancelled && cancelled()) ||
            (magnet_.on() && motion::seconds() - heldSince_ >= config_.maximumHoldSeconds);
    };
    try {
        for (std::size_t i = 0; i < steps.size(); ++i) {
            progress_.nextStep = i;
            if (stopRequested()) throw std::runtime_error("取放已取消或达到电磁铁最长通电时间");
            // carrying表示“可能带子”，吸合指令成功不等于已证实吸住。
            if (steps[i].action == Action::MagnetOn) { progress_.carrying = true; heldSince_ = motion::seconds(); }
            if (observer_) observer_(progress_); // 动作前先保存意图，日志失败时不继续执行。
            switch (steps[i].action) {
            case Action::Move:
                arm_.move(steps[i].target, stopRequested);
                break;
            case Action::MagnetOn:
                magnet_.set(true);
                wait(config_.pickupWait, cancelled);
                break;
            case Action::MagnetOff:
                if (magnet_.expired() || !magnet_.failure().empty())
                    throw std::runtime_error("磁铁已超时或写入失败，必须复核实物状态");
                magnet_.set(false);
                progress_.carrying = false;
                wait(config_.releaseWait, cancelled);
                break;
            }
            progress_.nextStep = i + 1;
            if (observer_) observer_(progress_);
        }
        progress_.finished = true;
        if (observer_) observer_(progress_);
    } catch (...) {
        progress_.interrupted = true;
        arm_.stop();
        // 保留磁铁句柄及原始通电时刻，由主流程执行已确认的故障释放/总通电期限策略。
        try { if (observer_) observer_(progress_); } catch (...) {}
        throw;
    }
}

void PickPlace::acknowledgeEmpty()
{
    if (magnet_.on()) throw std::runtime_error("请先确认棋子已卸载并关闭电磁铁");
    if (magnet_.expired() || !magnet_.failure().empty())
        throw std::runtime_error("电磁铁保护已触发，请核对实物并重新启动程序");
    progress_ = {};
    heldSince_ = 0;
    if (observer_) observer_(progress_);
}

} // namespace task
