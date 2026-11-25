// 按动作表控制三轴和电磁铁，用实测位置与速度确认每段运动完成。
#include "execute.hpp"
#include <stdexcept>

namespace scara
{
//---------执行数据区---------
constexpr double ArrivalAllowanceSeconds = 5;

static bool moveAxes(const AxisValues &target, const MotionLimits &limits, const volatile std::sig_atomic_t &running,
                     DriveFeedback &feedback, AxisValues &commanded);
static bool gripPiece(int dwellMs, const volatile std::sig_atomic_t &running, const AxisValues &position);
static const char *stopOutputs();

//---------动作执行区---------
bool Motion_execute(const std::vector<PickStep> &steps, const MotionLimits &limits,
                    const volatile std::sig_atomic_t &running)
{
    if (!running) return false;
    try
    {
        // 以实测位置起步。每段到位后，commanded 记住已完成的目标。
        DriveFeedback feedback;
        if (!Drive_read(feedback)) return false;
        AxisValues commanded = feedback.position;
        size_t completed = 0;
        while (running && completed < steps.size())
        {
            if (!Drive_read(feedback)) break;
            if (!running) break; // 等反馈期间，操作员也可以按下停止。
            const PickStep &step = steps[completed];
            bool ok = false;
            switch (step.action)
            {
            case PickAction::Move:
                ok = moveAxes(step.target, limits, running, feedback, commanded);
                break;
            case PickAction::LiftZ:
                ok = moveAxes({step.target[0], commanded[1], commanded[2]}, limits, running, feedback, commanded);
                break;
            case PickAction::Grip:
                ok = gripPiece(step.durationMs, running, commanded);
                break;
            case PickAction::Release:
            case PickAction::ReleaseOffBoard:
                ok = Drive_magnet(false);
                break;
            }
            if (!ok) break;
            ++completed;
        }
        if (completed == steps.size() && running) return true;
    }
    catch (...) { /* 驱动抛错也进入下面的停车流程。 */ }
    if (const char *error = stopOutputs()) throw std::runtime_error(error);
    return false;
}

//---------停止收尾区---------
bool Motion_stop()
{
    return stopOutputs() == nullptr;
}

// 两条命令各请求一次。即使关磁铁失败，仍要让三轴停车。
static const char *stopOutputs()
{
    bool magnetOff = false;
    bool motorsStopped = false;
    try { magnetOff = Drive_magnet(false); }
    catch (...) { /* 关断未确认，继续请求停车。 */ }
    try { motorsStopped = Drive_stop(); }
    catch (...) { /* 停车未确认，保留前一条命令的结果。 */ }

    if (!magnetOff && !motorsStopped) return "磁铁关断和机械臂停稳均未确认";
    if (!magnetOff) return "磁铁关断未确认";
    if (!motorsStopped) return "机械臂停稳未确认";
    return nullptr;
}

//---------轨迹执行与终点保持区---------
static bool moveAxes(const AxisValues &target, const MotionLimits &limits, const volatile std::sig_atomic_t &running,
                     DriveFeedback &feedback, AxisValues &commanded)
{
    if (!feedback.stationary) return false;
    JointTrajectory trajectory;
    if (!Trajectory_plan(commanded, target, limits, trajectory)) return false;
    if (trajectory.seconds == 0 && (feedback.position == target || feedback.reached == target)) return true;

    // 运行联动轨迹，直到发出一次完整的终点参考。
    const uint64_t start = feedback.timeMs;
    double elapsed = 0;
    while (running)
    {
        if (!Drive_write(Trajectory_sample(trajectory, elapsed))) return false;
        if (elapsed >= trajectory.seconds) break;
        if (!running || !Drive_read(feedback)) return false;
        elapsed = (feedback.timeMs - start) * 0.001;
        if (elapsed > trajectory.seconds + ArrivalAllowanceSeconds) return false;
    }

    // 只认终点发出后取到的新回报。上一段、还在走、或过期的反馈都不能交接。
    while (running && Drive_read(feedback))
    {
        if (!running) return false;
        if ((feedback.timeMs - start) * 0.001 > trajectory.seconds + ArrivalAllowanceSeconds) return false;
        if (feedback.stationary && feedback.reached == target)
        {
            commanded = target;
            return true;
        }
    }
    return false;
}

//---------定点吸持区---------
// 吸合后在原位停留，磁铁保持通电直到释放动作。
static bool gripPiece(int dwellMs, const volatile std::sig_atomic_t &running, const AxisValues &position)
{
    if (dwellMs < 0 || !running) return false;
    if (!Drive_magnet(true)) return false;
    if (!running) return false;
    if (dwellMs == 0) return true;

    // 从通电确认后的第一帧开始计时，通信等待时间不占用贴合时间。
    DriveFeedback feedback;
    if (!Drive_read(feedback)) return false;
    const uint64_t start = feedback.timeMs;
    const JointReference hold{position, {}};
    while (running && feedback.timeMs - start < static_cast<uint64_t>(dwellMs))
    {
        if (!Drive_write(hold)) return false;
        if (!running) return false;
        if (!Drive_read(feedback)) return false;
    }
    return running;
}

} // namespace scara
