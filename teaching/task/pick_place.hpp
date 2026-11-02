#pragma once
#include "motion/motion.hpp"
#include "robot/magnet.hpp"
#include <vector>

namespace task {

struct Config {
    double travelZ = 0, pickupWait = 0, releaseWait = 0, maximumHoldSeconds = 0;
    motion::Pose park{};
};
enum class Action { Move, MagnetOn, MagnetOff };
struct Step { Action action; motion::Pose target; };
struct Request {
    motion::Pose source, destination, captureSlot;
    bool capture = false;
};
struct Progress {
    std::size_t nextStep = 0;
    bool carrying = false, interrupted = false, finished = false;
};
// 先生成完整取放序列，再核对每段路径；失败不返回部分计划。
std::vector<Step> plan(const Request& request, const motion::Joints& start,
                       const Config& task, const motion::Config& arm);

class PickPlace {
public:
    PickPlace(motion::Motion& arm, robot::Magnet& magnet, const Config& config);
    void execute(const Request& request, const motion::Cancelled& cancelled = {});
    const Progress& progress() const { return progress_; }
    double energizedAt() const { return heldSince_; }
    void setProgressObserver(std::function<void(const Progress&)> observer) { observer_ = std::move(observer); }
    // 仅在人工确认实物已卸载、磁铁已关后解除软件中断锁定；不自动续跑旧步骤。
    void acknowledgeEmpty();
private:
    void wait(double duration, const motion::Cancelled& cancelled);
    motion::Motion& arm_;
    robot::Magnet& magnet_;
    Config config_;
    Progress progress_;
    double heldSince_ = 0;
    std::function<void(const Progress&)> observer_;
};

} // namespace task
