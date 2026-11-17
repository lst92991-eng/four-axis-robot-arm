#pragma once
#include <array>
#include <cstdint>
#include <string>

namespace scara
{
//v3使用标准CAN标识符和64字节FD数据
struct CanFrame
{
    unsigned id = 0;
    std::array<uint8_t, 64> data{};
};

void Candle_open(const std::string &serial);
void Candle_send(const CanFrame &frame);
bool Candle_receive(CanFrame &frame, unsigned timeoutMs);
void Candle_close() noexcept;

//USB帧头与板端v3报文都用小端32位字段
inline uint32_t readWord(const uint8_t *p)
{
    return uint32_t(p[0]) | uint32_t(p[1]) << 8 | uint32_t(p[2]) << 16 | uint32_t(p[3]) << 24;
}
inline void writeWord(uint8_t *p, uint32_t value)
{
    for (int i = 0; i < 4; ++i) p[i] = uint8_t(value >> (8 * i));
}
}
