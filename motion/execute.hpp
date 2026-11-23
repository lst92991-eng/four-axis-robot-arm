// 执行器接收取放计划，通过驱动读反馈、发三轴参考并控制电磁铁。
#pragma once
#include "plan.hpp"
#include <csignal>
#include <cstdint>

namespace scara
{
//---------动作执行与停车接口区---------
// 全部动作完成且停稳才返回 true。运动中断时关断磁铁并停车，停车无法确认时交给上层恢复。
bool Motion_execute(const std::vector<PickStep> &steps, const MotionLimits &limits,
                    const volatile std::sig_atomic_t &running);

// 停车会等待驱动确认，应从普通控制流程调用。
// 允许初始化前和重复调用，供正常退出及异常清理使用。
bool Motion_stop();

//---------驱动数据交接区---------
//本结构由实机IO或仿真IO填写，执行器按同一份数据推进动作
struct DriveFeedback
{
    // 同一周期的三轴实际角度与角速度，单位为度和度/秒。
    AxisValues position{}, velocity{};
    //主机单调毫秒时间，新反馈对应的新周期
    uint64_t timeMs = 0;
};
//取得可用的新周期反馈；超时、参考失效或驱动故障返回false
bool Drive_read(DriveFeedback &feedback);
//整组交付三轴参考，全部接受后返回true
bool Drive_write(const JointReference &reference);
// 正向通电吸持；断电放子。
bool Drive_magnet(bool on);
//请求三轴减速保持，确认实际停稳后返回true，超时返回false
bool Drive_stop();
} // namespace scara
