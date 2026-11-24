// 按动作表控制三轴和电磁铁，用实测位置与速度确认每段运动完成。
#include "execute.hpp"
#include <cmath>
#include <stdexcept>

namespace scara
{
//---------执行数据区---------
constexpr double PositionToleranceDeg = .12;
constexpr double StoppedSpeedDegS = .05;
constexpr uint64_t SettledMs = 250;
constexpr double ArrivalAllowanceSeconds = 5;

static bool moveAxes(const PickStep &step, const MotionLimits &limits, const volatile std::sig_atomic_t &running,
                     DriveFeedback &feedback, AxisValues &commanded);
static bool gripPiece(int dwellMs, const volatile std::sig_atomic_t &running,
                      DriveFeedback &feedback, const AxisValues &position);
static bool onTarget(const DriveFeedback &feedback, const AxisValues &target);

//---------动作执行区---------
bool Motion_execute(const std::vector<PickStep> &steps, const MotionLimits &limits,
                    const volatile std::sig_atomic_t &running)
{
    // 动作表从第一组反馈位置起步，此后用已完成段的 commanded 接续。
    DriveFeedback feedback;
    if (!running || !Drive_read(feedback)) return false;
    AxisValues commanded = feedback.position;
    bool done = true;
    try
    {
        for (const PickStep &step : steps)
        {
            done = running && Drive_read(feedback);
            if (!done) break;
            // Move 运行轨迹，Grip 原地吸持，两种释放动作都关断磁铁。
            if (step.action == PickAction::Move)
                done = moveAxes(step, limits, running, feedback, commanded);
            else if (step.action == PickAction::Grip)
                done = gripPiece(step.durationMs, running, feedback, commanded);
            else
                done = (step.action == PickAction::Release || step.action == PickAction::ReleaseOffBoard) && Drive_magnet(false);
            if (!done) break;
        }
    }
    catch (const std::exception &) { done = false; }
    if (done && running) return true;
    // 任一步中断，都经过同一处关磁铁和停车。只有无法确认停车才向上层报告异常。
    if (!Motion_stop()) throw std::runtime_error("机械臂停稳未确认");
    return false;
}

//---------停止收尾区---------
bool Motion_stop()
{
    // 分别请求关断磁铁和减速保持，只有两项都得到确认才返回成功。
    bool gripper = Drive_magnet(false);
    bool motors = Drive_stop();
    return gripper && motors;
}
//---------轨迹执行与终点保持区---------
static bool moveAxes(const PickStep &step, const MotionLimits &limits, const volatile std::sig_atomic_t &running,
                     DriveFeedback &feedback, AxisValues &commanded)
{
    if (!step.axes || (step.axes & ~AllAxes)) return false;
    // 沿用上段的命令终点，用本步目标替换选中的轴。
    AxisValues target = commanded;
    for (int axis = 0; axis < 3; ++axis)
    {
        if (std::abs(feedback.velocity[axis]) > StoppedSpeedDegS) return false;
        if (step.axes & (1u << axis)) target[axis] = step.target[axis];
    }
    // 从上段目标规划到本步目标。零行程且反馈已经到位时直接完成。
    JointTrajectory trajectory;
    if (!Trajectory_plan(commanded, target, limits, trajectory)) return false;
    if (trajectory.seconds == 0 && onTarget(feedback, target)) return true;

    // 走轨迹。按本段已用时间生成三轴参考，一次下发，再取下一周期反馈。
    uint64_t start = feedback.timeMs;
    double elapsed = 0;
    JointReference reference;
    while (running && elapsed < trajectory.seconds)
    {
        Trajectory_sample(trajectory, elapsed, reference);
        if (!Drive_write(reference) || !Drive_read(feedback)) return false;
        // 驱动时钟用毫秒，轨迹采样用秒。
        elapsed = (feedback.timeMs - start) * .001;
    }

    // 保持目标角。三轴都到位并连续停稳 250 ms 后交给下一步。
    JointReference hold;
    hold.position = target;
    // stableSince 记录连续合格样本的起始时刻，任一轴偏离都会重新计时。
    uint64_t stableSince = feedback.timeMs;
    const double holdDeadline = trajectory.seconds + ArrivalAllowanceSeconds;
    while (running && (feedback.timeMs - start) * .001 <= holdDeadline)
    {
        if (!Drive_write(hold)) return false;
        bool there = onTarget(feedback, target);
        if (there && feedback.timeMs - stableSince >= SettledMs)
        {
            // 反馈已经连续停稳，才把本段终点交给下一段作为起点。
            commanded = target;
            return true;
        }
        if (!Drive_read(feedback)) return false;
        // 上个样本有轴偏离，连续停稳从这个新样本起重新计时。
        if (!there) stableSince = feedback.timeMs;
    }
    return false;
}

//---------定点吸持区---------
// 吸合后先在原处停留，持续吸持到落点。释放只需关断 H 桥。
static bool gripPiece(int dwellMs, const volatile std::sig_atomic_t &running,
                      DriveFeedback &feedback, const AxisValues &position)
{
    if (dwellMs < 0 || !Drive_magnet(true)) return false;
    // 以驱动反馈的毫秒时间计时，停留期间持续发送当前位置和零速度。
    const uint64_t start = feedback.timeMs;
    JointReference hold;
    hold.position = position;
    while (feedback.timeMs - start < unsigned(dwellMs))
        if (!running || !Drive_write(hold) || !Drive_read(feedback))
            return false;
    return true;
}

//---------到位判定区---------
// 每一轴都要同时满足位置误差与剩余速度要求。
static bool onTarget(const DriveFeedback &feedback, const AxisValues &target)
{
    for (int axis = 0; axis < 3; ++axis)
    {
        double error = std::abs(feedback.position[axis] - target[axis]);
        double speed = std::abs(feedback.velocity[axis]);
        if (!(error <= PositionToleranceDeg && speed <= StoppedSpeedDegS)) return false;
    }
    return true;
}

} // namespace scara
