// 执行器接收取放计划，通过驱动读反馈、发三轴参考并控制电磁铁。
#pragma once
#include "plan.hpp"
#include <csignal>
#include <cstdint>
#include <optional>

namespace scara
{
//---------动作执行与停车接口区---------
// 全部动作完成才返回 true。中途失败先关磁铁并停车，收尾未确认时报告具体设备。
bool Motion_execute(const std::vector<PickStep> &steps, const MotionLimits &limits,
                    const volatile std::sig_atomic_t &running);

// 关磁铁和三轴停稳都得到驱动确认才返回 true，应从普通控制流程调用。
// 允许初始化前和重复调用，供正常退出及异常清理使用。
bool Motion_stop();

//---------驱动数据交接区---------
//本结构由实机IO或仿真IO填写，执行器按同一份数据推进动作
struct DriveFeedback
{
    AxisValues position{};        // 同一周期三轴实际驱动角，单位度。
    bool stationary = false;      // 三轴的板端静止标志均有效。
    // 三轴均确认本次目标后，填回对应的原始下发角度；任一轴尚未到位时留空。
    std::optional<AxisValues> reached;
    //主机单调毫秒时间，新反馈对应的新周期
    uint64_t timeMs = 0;
};
// 读取新周期并维持驱动通信。失联、参考失效或故障返回 false。
bool Drive_read(DriveFeedback &feedback);
// 整组三轴接受后返回 true。驱动按应答及后续新状态的目标回读关联 reached；旧到位标志不复用。
bool Drive_write(const JointReference &reference);
// 正向通电吸持；断电放子。
bool Drive_magnet(bool on);
//请求三轴减速保持，确认实际停稳后返回true，超时返回false
bool Drive_stop();
} // namespace scara
