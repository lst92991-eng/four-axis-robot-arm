#include "can.hpp"
#include <linux/can.h>
#include <linux/can/error.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/socket.h>
#include <unistd.h>
#include <stdexcept>

namespace robot {

Can::Can(const char* interface)
{
    // 根据名称取得CAN接口编号。
    unsigned index = if_nametoindex(interface);
    if (index == 0)
        throw std::runtime_error("CAN接口不存在");

    // 创建非阻塞CAN通信句柄。
    fd_ = socket(PF_CAN, SOCK_RAW | SOCK_NONBLOCK, CAN_RAW);
    if (fd_ < 0)
        throw std::runtime_error("创建CAN连接失败");

    // 绑定到指定CAN接口。
    sockaddr_can address{};
    address.can_family = AF_CAN;
    address.can_ifindex = index;

    if (bind(fd_, reinterpret_cast<sockaddr*>(&address),
             sizeof(address)) < 0) {
        close(fd_);
        throw std::runtime_error("CAN绑定失败");
    }

    // 允许接收CAN总线错误信息。
    can_err_mask_t errors = CAN_ERR_MASK;

    if (setsockopt(fd_, SOL_CAN_RAW, CAN_RAW_ERR_FILTER,
                   &errors, sizeof(errors)) < 0) {
        close(fd_);
        throw std::runtime_error("CAN错误订阅失败");
    }
}

} // namespace robot
