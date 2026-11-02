#pragma once
#include "can.hpp"
#include <cstdint>
#include <initializer_list>
#include <vector>

namespace robot {
struct MotorFeedback {
    double angleDeg = 0; // 电机侧累计角度；Emm反馈65536计数/圈。
    int rpm = 0;
    uint16_t currentMa = 0;
    uint8_t status = 0; // bit0使能、bit1到位、bit2堵转、bit3堵转保护。
};

// 一条CAN总线串行使用；这里只处理Emm协议，不决定关节坐标或运动限值。
class Motor {
public:
    Motor(Can& can, unsigned node, unsigned pulsesPerTurn = 3200, int replyTimeoutMs = 80);
    ~Motor();
    Motor(const Motor&) = delete;
    Motor& operator=(const Motor&) = delete;
    void prepare(); // 核验配置和静止；临时500ms失联停机，不写Flash或使能。
    MotorFeedback read();
    void start(int signedRpm, uint8_t acceleration = 1);
    void move(double absoluteMotorDeg, unsigned rpm, uint8_t acceleration, bool synchronous = true);
    static void synchronize(Can& can); // 各轴FD缓存ACK齐全后，由运动模块调用一次。
    void requestStop(); // 先向各轴发出FE，避免等待某一轴时其他轴继续运动。
    void stop(); // FE应答及三次零速度确认；故障后仍可显式调用。
    unsigned node() const { return node_; }
    uint16_t firmwareVersion() const { return firmware_; }
private:
    void send(std::initializer_list<uint8_t> bytes);
    std::vector<uint8_t> exchange(std::initializer_list<uint8_t> bytes, unsigned length);
    std::vector<uint8_t> query(uint8_t code, unsigned length);
    void acknowledge(std::initializer_list<uint8_t> bytes);
    Can& can_;
    unsigned node_, pulsesPerTurn_;
    int timeoutMs_;
    uint16_t firmware_ = 0;
    bool ready_ = false, active_ = false;
};
} // namespace robot
