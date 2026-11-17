//按动作顺序执行：吸持、释放脉冲、整组三轴运动
#include "execute.hpp"
#include "common/events.hpp"
#include <cmath>
#include <iostream>
#include <stdexcept>

namespace scara
{
//---------动作执行区---------
void Motion_execute(const std::vector<PickStep> &steps, const MotionLimits &limits,
                    const volatile std::sig_atomic_t &running, bool &success)
{
    success = false;
    DriveFeedback feedback;
    if (!running || !Drive_read(feedback)) return;
    //第一段从反馈起步，后续段沿用上一命令终点，保持参考连续
    AxisValues commanded = feedback.position;
    try
    {
        for (const PickStep &step : steps)
        {
            if (!running || !Drive_read(feedback)) throw std::runtime_error("执行取消或驱动反馈中断");
            if (step.action == PickAction::Hold)
            {
                if (!Drive_release(0)) throw std::runtime_error("吸持状态未确认");
                continue;
            }
            //释放期间保持原位置，驱动端负责脉冲截止，主机等待结束或取消
            if (step.action == PickAction::ReleasePulse)
            {
                if (step.durationMs <= 0 || !Drive_release(step.durationMs) || !Drive_read(feedback))
                    throw std::runtime_error("释放脉冲未接受或反馈中断");
                uint64_t startMs = feedback.timeMs;
                JointReference hold;
                hold.position = commanded;
                while (feedback.timeMs - startMs < unsigned(step.durationMs))
                    if (!running || !Drive_write(hold) || !Drive_read(feedback))
                        throw std::runtime_error("释放已取消或驱动通信中断");
                if (!running || !Drive_release(0)) throw std::runtime_error("释放脉冲未结束");
                continue;
            }
            //其余合法动作均为运动；只改选中轴的目标，其余轴继续保持
            if (step.action != PickAction::Move || !step.axes || (step.axes & ~AllAxes))
                throw std::runtime_error("联动动作或轴选择无效");
            AxisValues target = commanded;
            bool alreadyThere = true;
            for (int axis = 0; axis < 3; ++axis)
            {
                if (std::abs(feedback.velocity[axis]) > .05) throw std::runtime_error("整组尚未停稳");
                if (step.axes & (1u << axis)) target[axis] = step.target[axis];
                alreadyThere = alreadyThere && std::abs(feedback.position[axis] - target[axis]) <= .12;
            }
            JointTrajectory trajectory;
            if (!Trajectory_plan(commanded, target, limits, trajectory))
                throw std::runtime_error("联动目标或限幅无效");
            if (trajectory.seconds == 0 && alreadyThere) continue;

            //同一时钟生成整组参考，三轴真实到位后累计250ms停稳时间
            uint64_t startMs = feedback.timeMs, settledSince = 0;
            bool settling = false;
            while (true)
            {
                if (!running) throw std::runtime_error("联动已取消");
                double elapsed = (feedback.timeMs - startMs) * .001;
                if (elapsed > trajectory.seconds + 5) throw std::runtime_error("整组到位超时");
                JointReference reference;
                Trajectory_sample(trajectory, elapsed, reference);
                if (!Drive_write(reference)) throw std::runtime_error("整组三轴参考未接受");
                bool atGoal = elapsed >= trajectory.seconds;
                for (int axis = 0; axis < 3; ++axis)
                    atGoal = atGoal && std::abs(feedback.position[axis] - target[axis]) <= .12 &&
                             std::abs(feedback.velocity[axis]) <= .05;
                if (!atGoal) settling = false;
                if (atGoal && !settling)
                {
                    settledSince = feedback.timeMs;
                    settling = true;
                }
                if (settling && feedback.timeMs - settledSince >= 250) break;
                if (!Drive_read(feedback)) throw std::runtime_error("联动反馈中断");
            }
            commanded = target;
        }
        if (!running) throw std::runtime_error("联动已取消");
        success = true;
    }
    catch (const std::exception &error)
    {
        //先停止整组，再通知显示；回调不会接管控制流程
        bool stopped = Motion_stop();
        Event_emit(stopped ? EventType::Motion : EventType::Stop, error.what());
        if (!stopped) throw std::runtime_error("联动失败且尚未确认整组停止");
    }
}

//---------停止收尾区---------
bool Motion_stop()
{
    //两项均执行，结束释放脉冲并确认J1/J2/J3减速保持
    bool gripper = Drive_release(0);
    bool motors = Drive_stop();
    return gripper && motors;
}
} // namespace scara
