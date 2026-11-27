// 按动作表控制三轴和电磁铁，用实测位置与速度确认每段运动完成。
#include "execute.hpp"
#include <stdexcept>

namespace scara
{
//---------执行数据区---------
constexpr double ArrivalAllowanceSeconds = 5;

static bool moveAxes(const AxisValues &start, const AxisValues &target, const MotionLimits &limits,
                     const volatile std::sig_atomic_t &running, DriveFeedback &feedback);
static bool gripPiece(int dwellMs, const volatile std::sig_atomic_t &running, const AxisValues &position);
static bool runSteps(const std::vector<PickStep> &steps, const MotionLimits &limits,
                     const volatile std::sig_atomic_t &running, AxisValues held);
static const char *stopOutputs();

//---------动作执行区---------
bool Motion_execute(const std::vector<PickStep> &steps, const MotionLimits &limits,
                    const volatile std::sig_atomic_t &running)
{
    if (!running) return false;
    try
    {
        // 第一次反馈提供三轴起点；持有起点后，执行失败统一关磁铁、停车。
        DriveFeedback feedback;
        if (!Drive_read(feedback)) return false;
        if (runSteps(steps, limits, running, feedback.position)) return true;
    }
    catch (...) { /* 驱动抛错也进入下面的停车流程。 */ }
    if (const char *error = stopOutputs()) throw std::runtime_error(error);
    return false;
}

// held 是上一个已到达的目标角，原地吸持和单独抬 Z 都沿用这一姿态。
static bool runSteps(const std::vector<PickStep> &steps, const MotionLimits &limits,
                     const volatile std::sig_atomic_t &running, AxisValues held)
{
    for (const PickStep &step : steps)
    {
        DriveFeedback feedback;
        if (!running) return false;
        // 等待期间也可能收到停止请求，执行本步前再查一次。
        if (!Drive_read(feedback) || !running) return false;
        switch (step.action)
        {
        case PickAction::LiftZ:
        case PickAction::Move:
        {
            AxisValues target = step.target;
            if (step.action == PickAction::LiftZ)
                target = {target[0], held[1], held[2]};
            if (!moveAxes(held, target, limits, running, feedback)) return false;
            held = target;
            break;
        }
        case PickAction::Grip:
            if (!gripPiece(step.durationMs, running, held)) return false;
            break;
        case PickAction::Release:
        case PickAction::ReleaseOffBoard:
            if (!Drive_magnet(false)) return false;
            break;
        default:
            return false;
        }
    }
    return running;
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
static bool moveAxes(const AxisValues &start, const AxisValues &target, const MotionLimits &limits,
                     const volatile std::sig_atomic_t &running, DriveFeedback &feedback)
{
    if (!feedback.stationary) return false;
    JointTrajectory trajectory;
    if (!Trajectory_plan(start, target, limits, trajectory)) return false;
    if (trajectory.seconds == 0 && (feedback.position == target || feedback.reached == target)) return true;

    const uint64_t startMs = feedback.timeMs;
    const double deadlineSeconds = trajectory.seconds + ArrivalAllowanceSeconds;
    double elapsedSeconds = 0;
    while (running && elapsedSeconds < trajectory.seconds)
    {
        JointReference reference = Trajectory_sample(trajectory, elapsedSeconds);
        if (!Drive_write(reference)) return false;
        if (!running || !Drive_read(feedback)) return false;
        elapsedSeconds = (feedback.timeMs - startMs) * 0.001;
    }

    // 发一次终点和零速参考，随后只读反馈，等三轴确认本段目标。
    if (!running || elapsedSeconds > deadlineSeconds) return false;
    const JointReference hold{target, {}};
    if (!Drive_write(hold)) return false;
    while (running && Drive_read(feedback))
    {
        elapsedSeconds = (feedback.timeMs - startMs) * 0.001;
        if (!running || elapsedSeconds > deadlineSeconds) return false;
        if (feedback.stationary && feedback.reached == target) return true;
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
        if (!running || !Drive_read(feedback)) return false;
    }
    return running;
}

} // namespace scara
