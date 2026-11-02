#pragma once
#include <linux/can.h>

namespace robot {

// 独占已配置的SocketCAN接口；速率和接线由部署步骤确认。
class Can {
public:
    explicit Can(const char* interface);
    ~Can();
    Can(const Can&) = delete;
    Can& operator=(const Can&) = delete;
    void send(const can_frame& frame);
    bool receive(can_frame& frame, int timeoutMs);

private:
    int fd_ = -1;
};

} // namespace robot
