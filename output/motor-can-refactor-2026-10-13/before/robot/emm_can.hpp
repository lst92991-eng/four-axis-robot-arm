#pragma once
#include <cstdint>
#include <initializer_list>
#include <linux/can.h>

namespace robot {

struct EmmFeedback {
    double angleDeg = 0;           // 电机侧累计角度，不是丝杆毫米位置。
    int rpm = 0;                   // 当前接线下：正值上升，负值下降。
    uint16_t currentMa = 0;
    uint8_t status = 0;
};

// 当前节点1、Emm闭环、500kbit/s。只操作已经配置好的Linux SocketCAN接口。
// 单线程独占；不自动使能/回零/清故障，构造时不发送CAN帧。
class EmmCan {
public:
    explicit EmmCan(const char* interface);
    ~EmmCan();
    EmmCan(const EmmCan&) = delete;
    EmmCan& operator=(const EmmCan&) = delete;
    void prepare();               // 静止核对并临时启用500ms失联停机，不存Flash。
    void start(int signedRpm);     // 一次F6；+上升/-下降，非零且幅值不超过3000。
    EmmFeedback read();            // 运行中建议每100ms读取一次，也维持心跳。
    void stop();                  // FE停止并连续确认零转速；不松轴。

private:
    can_frame exchange(std::initializer_list<uint8_t> bytes, uint8_t replyLength);
    can_frame query(uint8_t code, uint8_t length);
    void stopOnFailure() noexcept;
    int fd_ = -1;
    bool ready_ = false, moving_ = false;
    double startedMs_ = 0;
};

} // namespace robot
