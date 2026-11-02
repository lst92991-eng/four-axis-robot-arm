#include "can.hpp"
#include <linux/can.h>
#include <linux/can/error.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/socket.h>
#include <unistd.h>
#include <stdexcept>
#include <algorithm>
#include <cerrno>
#include <chrono>
#include <poll.h>
#include <fcntl.h>

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

    // 集成Pikafish子进程后新增：exec不继承机械设备句柄。
    if (fcntl(fd_, F_SETFD, FD_CLOEXEC) < 0) {
        close(fd_);
        throw std::runtime_error("CAN句柄继承设置失败");
    }
}

Can::~Can() { close(fd_); }

void Can::send(const can_frame& frame)
{
    if (frame.can_dlc > 8 || (frame.can_id & (CAN_RTR_FLAG | CAN_ERR_FLAG)) ||
        (!(frame.can_id & CAN_EFF_FLAG) && (frame.can_id & ~CAN_SFF_MASK)))
        throw std::runtime_error("要求有效的经典CAN数据帧");
    // 是否已经到达驱动无法由发送失败判断，因此不自动重发。
    if (::write(fd_, &frame, sizeof(frame)) != sizeof(frame))
        throw std::runtime_error("CAN发送失败");
}

bool Can::receive(can_frame& frame, int timeoutMs)
{
    if (timeoutMs <= 0) throw std::runtime_error("CAN接收期限必须为正");
    const auto end = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeoutMs);
    while (std::chrono::steady_clock::now() < end) {
        const auto left = std::chrono::duration_cast<std::chrono::milliseconds>(end - std::chrono::steady_clock::now()).count();
        pollfd pending{fd_, POLLIN, 0};
        const int result = poll(&pending, 1, int(std::max<int64_t>(1, left)));
        if (result < 0 && errno == EINTR) continue;
        if (result < 0 || (pending.revents & (POLLERR | POLLHUP | POLLNVAL)))
            throw std::runtime_error("CAN接口错误");
        if (!result) continue;
        const auto size = ::read(fd_, &frame, sizeof(frame));
        if (size < 0 && (errno == EAGAIN || errno == EINTR)) continue;
        if (size != sizeof(frame) || frame.can_dlc > 8)
            throw std::runtime_error("CAN接收长度错误");
        if (frame.can_id & CAN_ERR_FLAG) {
            // 仅兼容现场旧candleLight的精确空通知；ACK/协议/Bus-off仍报错。
            if (frame.can_id == (CAN_ERR_FLAG | CAN_ERR_CRTL) && frame.can_dlc == 8 &&
                std::all_of(frame.data, frame.data + 8, [](uint8_t b) { return b == 0; })) continue;
            throw std::runtime_error("CAN错误帧，ID=" + std::to_string(frame.can_id));
        }
        return true;
    }
    return false;
}

} // namespace robot
