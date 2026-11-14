#include "execute.hpp"
#include <iostream>

namespace scara
{
// 后续直接在这两个函数中接SPI-CAN/H桥。当前没有硬件实现，不伪报完成或停稳。
void Motion_execute(const std::vector<PickStep> &, const volatile std::sig_atomic_t &, bool &success)
{
    success = false;
    std::cerr << "尚未接入SPI-CAN/H桥执行接口\n";
}

bool Motion_stop()
{
    return false;
}
} // namespace scara
