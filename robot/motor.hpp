#pragma once
#include <cstdint>
#include <initializer_list>
#include "can.hpp"

namespace robot {

struct MotorFeedback {
    double angleDeg = 0;           // 电机侧累计角度，不是丝杆毫米位置。
    int rpm = 0;                   // 协议方向0为正、1为负；节点1当前正值上升。
    uint16_t currentMa = 0;
    uint8_t status = 0;
};

// 已验证节点1的Emm闭环协议；其他节点需分别核对配置。
// 单线程独占；不自动使能/回零/清故障，构造时不发送CAN帧。
class Motor {
public:
    explicit Motor(Can& can, unsigned node = 1);
    ~Motor();
    Motor(const Motor&) = delete;
    Motor& operator=(const Motor&) = delete;
    void prepare();               // 静止核对并临时启用500ms失联停机，不存Flash。
    void start(int signedRpm);     // 一次F6；符号选方向，节点1当前+上/-下，幅值1～3000。
    MotorFeedback read();         // 运行中建议每100ms读取一次，也维持心跳。
    void stop();                  // FE停止并连续确认零转速；不松轴。

private:
    can_frame exchange(std::initializer_list<uint8_t> bytes, uint8_t replyLength);
    can_frame query(uint8_t code, uint8_t length);
    void stopOnFailure() noexcept;
    Can& can_;                   // Can须先构造、后析构；同一总线串行调用。
    canid_t nodeId_;
    bool ready_ = false, moving_ = false;
    double startedMs_ = 0;
};

} // namespace robot
