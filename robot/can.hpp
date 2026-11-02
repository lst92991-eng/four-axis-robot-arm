#pragma once
#include <linux/can.h>

namespace robot {

// 单线程使用已配置的Linux SocketCAN；不设置速率、不理解电机协议。
class Can {
public:
    explicit Can(const char* interface);
    ~Can();
    Can(const Can&) = delete;
    Can& operator=(const Can&) = delete;
    void send(const can_frame& frame); // 发送一次；失败抛异常，不盲重发。
    bool receive(can_frame& frame, int timeoutMs); // 超时返回false，真实总线错误抛异常。
private:
    int fd_ = -1;
};

} // namespace robot
