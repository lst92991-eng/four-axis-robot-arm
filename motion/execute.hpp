#pragma once
#include "plan.hpp"
#include <csignal>

namespace scara
{
// 同步执行动作数组；全部完成且停稳后，才把success写成true。
// running变0时停止执行并返回false；不得继续下一步或重发整个数组。
// 正常返回false表示未执行或已停稳的失败；无法确认停止时抛出异常，由主入口收尾。
void Motion_execute(const std::vector<PickStep> &steps, const volatile std::sig_atomic_t &running, bool &success);

// 普通函数上下文调用，不能放进信号回调；真实确认停稳才返回true。
// 允许初始化前和重复调用，供正常退出及异常清理使用。
bool Motion_stop();
} // namespace scara
