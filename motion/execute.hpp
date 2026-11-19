#pragma once
#include "plan.hpp"
#include <csignal>
#include <cstdint>

namespace scara
{
// 同步执行动作数组；全部完成且停稳后，才把success写成true。
// running变0时停止执行并返回false；不得继续下一步或重发整个数组。
// 正常返回false表示未执行或已停稳的失败；无法确认停止时抛出异常，由主入口收尾。
void Motion_execute(const std::vector<PickStep> &steps, const MotionLimits &limits,
                    const volatile std::sig_atomic_t &running, bool &success);

// 普通函数上下文调用，不能放进信号回调；真实确认停稳才返回true。
// 允许初始化前和重复调用，供正常退出及异常清理使用。
bool Motion_stop();

//---------驱动数据交接区---------
//本结构由实机IO或仿真IO填写，执行器按同一份数据推进动作
struct DriveFeedback
{
    //同一通信周期的J1/J2/J3实际位置和速度
    AxisValues position{}, velocity{};
    //主机单调毫秒时间，新反馈对应的新周期
    uint64_t timeMs = 0;
};
//取得可用的新周期反馈；超时、参考失效或驱动故障返回false
bool Drive_read(DriveFeedback &feedback);
//整组交付三轴参考，全部接受后返回true
bool Drive_write(const JointReference &reference);
//非零值开始指定毫秒的释放脉冲，0结束脉冲并恢复吸持
bool Drive_release(int durationMs);
//请求三轴减速保持，确认实际停稳后返回true，超时返回false
bool Drive_stop();
} // namespace scara
