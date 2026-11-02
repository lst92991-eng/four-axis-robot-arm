#pragma once
#include <array>
#include <cstdint>
#include <functional>
#include <vector>

using Bytes = std::vector<uint8_t>;
struct X42Frame {
    uint32_t id;
    Bytes data;
};
struct X42Move {
    double angle = 0;              // 电机轴度数，已经由上层换算零点/速比/方向。
    uint16_t speed = 1, accel = 1; // 0.1RPM、RPM/s；对称加减速。
};
struct X42Sample {
    double angle = 0, rpm = 0, positionTime = -1, speedTime = -1, statusTime = -1;
    uint8_t status = 0, options = 0, firmware = 0, hardware = 0;
    uint32_t heartbeatMs = 0;
    bool acknowledged = false, arrived = false;
};
Bytes x42Position(const X42Move &move);
std::vector<X42Frame> x42Split(uint8_t node, const Bytes &command);
void x42Decode(const X42Frame &frame, std::array<X42Sample, 4> &samples, double time);
std::array<X42Move, 4> x42Plan(const std::array<double, 4> &from, const std::array<double, 4> &to,
                               double maxRpm, double accelRpm);
double x42Duration(double distanceDeg, const X42Move &move);

// Linux经典SocketCAN：构造只打开接口，不使能、不回零、不改持久参数。
class X42Can {
    int fd = -1;

public:
    explicit X42Can(const char *interface);
    ~X42Can();
    X42Can(const X42Can &) = delete;
    X42Can &operator=(const X42Can &) = delete;
    void send(const X42Frame &frame);
    bool read(X42Frame &frame, int timeoutMs);
    void command(uint8_t node, const Bytes &bytes);
    void stop(); // FE98立即停止；不失能，Z轴不能自动松轴。
    void move(const std::array<double, 4> &goalDeg, double maxRpm, double accelRpm,
              const std::function<void(const std::array<X42Sample, 4> &, double)> &feedback = {});
};
