#include "can.hpp"
#include <linux/can/error.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>
#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <stdexcept>

namespace robot {
namespace {
void require(bool ok, const char* message) {
    if (!ok) throw std::runtime_error(message);
}
} // namespace

Can::Can(const char* interface) {
    require(interface && *interface && std::strlen(interface) < IFNAMSIZ, "CAN接口名称错误");
    fd_ = socket(PF_CAN, SOCK_RAW | SOCK_NONBLOCK, CAN_RAW);
    require(fd_ >= 0, "创建SocketCAN失败");
    try {
        ifreq request{};
        std::strcpy(request.ifr_name, interface);
        require(ioctl(fd_, SIOCGIFINDEX, &request) == 0, "CAN接口不存在");
        sockaddr_can address{};
        address.can_family = AF_CAN;
        address.can_ifindex = request.ifr_ifindex;
        require(bind(fd_, reinterpret_cast<sockaddr*>(&address), sizeof(address)) == 0, "CAN绑定失败");
        can_err_mask_t errors = CAN_ERR_MASK;
        require(setsockopt(fd_, SOL_CAN_RAW, CAN_RAW_ERR_FILTER, &errors, sizeof(errors)) == 0, "CAN错误订阅失败");
    } catch (...) { close(fd_); throw; }
}

Can::~Can() { close(fd_); }

void Can::send(const can_frame& frame) {
    require(frame.can_dlc <= 8 && !(frame.can_id & (CAN_RTR_FLAG | CAN_ERR_FLAG)), "要求经典CAN数据帧");
    require((frame.can_id & CAN_EFF_FLAG) || !(frame.can_id & ~CAN_SFF_MASK), "标准帧ID超出11位");
    require(::write(fd_, &frame, sizeof(frame)) == sizeof(frame), "CAN发送失败，不自动重发");
}

bool Can::receive(can_frame& frame, int timeoutMs) {
    require(timeoutMs > 0, "CAN接收超时必须大于0");
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeoutMs);
    while (std::chrono::steady_clock::now() < deadline) {
        const auto left = std::chrono::duration_cast<std::chrono::milliseconds>(deadline - std::chrono::steady_clock::now()).count();
        pollfd pending{fd_, POLLIN, 0};
        const int result = poll(&pending, 1, int(std::max<int64_t>(1, left)));
        if (result < 0 && errno == EINTR) continue;
        require(result >= 0 && !(pending.revents & (POLLERR | POLLHUP | POLLNVAL)), "CAN接口错误");
        if (!result) continue;
        const auto size = ::read(fd_, &frame, sizeof(frame));
        if (size < 0 && (errno == EAGAIN || errno == EINTR)) continue;
        require(size == sizeof(frame) && frame.can_dlc <= 8, "CAN接收长度错误");
        if (frame.can_id & CAN_ERR_FLAG) {
            // 只兼容已验证的旧candleLight空通知；不屏蔽ACK/协议/Bus-off。
            if (frame.can_id == (CAN_ERR_FLAG | CAN_ERR_CRTL) && frame.can_dlc == 8 &&
                std::all_of(frame.data, frame.data + 8, [](uint8_t b) { return b == 0; })) continue;
            throw std::runtime_error("CAN错误帧，ID=" + std::to_string(frame.can_id));
        }
        return true;
    }
    return false;
}

} // namespace robot
