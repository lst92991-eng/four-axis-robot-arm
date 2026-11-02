#include "motor.hpp"
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <thread>

namespace robot {
namespace {
using Clock = std::chrono::steady_clock;
uint32_t number(const uint8_t* data, unsigned count)
{
    uint32_t value = 0;
    for (unsigned i = 0; i < count; ++i) value = (value << 8) | data[i];
    return value;
}
}

Motor::Motor(Can& can, unsigned node, unsigned pulsesPerTurn, int replyTimeoutMs)
    : can_(can), node_(node), pulsesPerTurn_(pulsesPerTurn), timeoutMs_(replyTimeoutMs)
{
    if (node < 1 || node > 255 || pulsesPerTurn < 200 || pulsesPerTurn > 102400 ||
        replyTimeoutMs < 1 || replyTimeoutMs > 100)
        throw std::runtime_error("电机地址、每圈脉冲或应答期限错误");
}

Motor::~Motor()
{
    // 析构只补发停止请求；正常退出由上层先停全轴并确认。
    if (active_) { try { requestStop(); } catch (...) {} }
}

void Motor::send(std::initializer_list<uint8_t> bytes)
{
    if (bytes.size() < 2 || bytes.size() > 32) throw std::runtime_error("Emm命令长度错误");
    const auto* data = bytes.begin();
    // CAN分包：每包重复功能码，后面最多7字节；扩展ID低8位是包号。
    for (unsigned offset = 1, packet = 0; offset < bytes.size(); offset += 7, ++packet) {
        can_frame frame{};
        frame.can_id = CAN_EFF_FLAG | (node_ << 8) | packet;
        frame.can_dlc = uint8_t(1 + std::min<size_t>(7, bytes.size() - offset));
        frame.data[0] = data[0];
        std::copy_n(data + offset, frame.can_dlc - 1, frame.data + 1);
        can_.send(frame);
    }
}

std::vector<uint8_t> Motor::exchange(std::initializer_list<uint8_t> bytes, unsigned length)
{
    if (length < 3 || length > 32) throw std::runtime_error("Emm应答长度错误");
    // 排除上次残留ACK/到位通知。持续繁忙则拒绝接管，不伪装获得新鲜应答。
    can_frame frame{};
    unsigned discarded = 0;
    while (can_.receive(frame, 1)) {
        if (++discarded == 32) throw std::runtime_error("CAN接收持续繁忙");
    }
    send(bytes);
    const auto end = Clock::now() + std::chrono::milliseconds(timeoutMs_);
    std::vector<uint8_t> result(length);
    result[0] = *bytes.begin();
    std::array<bool, 5> received{};
    const unsigned packets = (length - 1 + 6) / 7;
    unsigned count = 0;
    while (Clock::now() < end) {
        const auto left = std::chrono::duration_cast<std::chrono::milliseconds>(end - Clock::now()).count();
        if (!can_.receive(frame, int(std::max<int64_t>(1, left)))) break;
        if ((frame.can_id & ~canid_t(255)) != (CAN_EFF_FLAG | (node_ << 8)) ||
            !frame.can_dlc || frame.data[0] != result[0]) continue;
        // 旧位置到位通知不是新命令的接收ACK。
        if (length == 3 && frame.can_dlc == 3 && frame.data[1] == 0x9F) continue;
        const unsigned packet = frame.can_id & 255;
        if (packet >= packets) throw std::runtime_error("Emm应答包号错误");
        const unsigned offset = 1 + packet * 7, size = std::min(7u, length - offset);
        if (frame.can_dlc != size + 1) throw std::runtime_error("Emm应答包长度错误");
        if (received[packet]) {
            if (!std::equal(frame.data + 1, frame.data + 1 + size, result.begin() + offset))
                throw std::runtime_error("Emm重复包内容冲突");
            continue;
        }
        std::copy_n(frame.data + 1, size, result.begin() + offset);
        received[packet] = true;
        if (++count == packets) {
            if (result.back() != 0x6B) throw std::runtime_error("Emm校验码错误");
            return result;
        }
    }
    ready_ = false;
    throw std::runtime_error("电机应答超时，节点=" + std::to_string(node_));
}

std::vector<uint8_t> Motor::query(uint8_t code, unsigned length) { return exchange({code, 0x6B}, length); }

void Motor::acknowledge(std::initializer_list<uint8_t> bytes)
{
    if (exchange(bytes, 3)[1] != 2) {
        ready_ = false;
        throw std::runtime_error("驱动拒绝命令，节点=" + std::to_string(node_));
    }
}

void Motor::prepare()
{
    if (active_) throw std::runtime_error("先停止再核对驱动");
    ready_ = false;
    const auto mode = query(0x1A, 4), version = query(0x1F, 6);
    if (number(mode.data() + 1, 2) != 6 || number(version.data() + 1, 2) == 0)
        throw std::runtime_error("要求已核对的Emm闭环固件");
    firmware_ = uint16_t(number(version.data() + 1, 2));
    if ((query(0x3B, 3)[1] & 0x33) != 3 || (query(0x3A, 3)[1] & 13) != 1 ||
        number(query(0x35, 5).data() + 2, 2) != 0)
        throw std::runtime_error("驱动未校准、未使能、故障或仍在运动");
    const auto config = exchange({0x42, 0x6C, 0x6B}, 32);
    const unsigned steps = config[3] == 0x19 ? 200 : config[3] == 0x32 ? 400 : 0;
    const unsigned microsteps = config[8] ? config[8] : 256;
    if (config[1] != 33 || config[2] != 21 || steps * microsteps != pulsesPerTurn_ ||
        config[19] != node_ || config[20] != 0 || (config[21] != 1 && config[21] != 3))
        throw std::runtime_error("驱动细分、地址、校验或ACK方式与配置不符");
    if (number(query(0x16, 6).data() + 1, 4) != 500) {
        acknowledge({0x68, 0x38, 0, 0, 0, 1, 0xF4, 0x6B});
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    if (number(query(0x16, 6).data() + 1, 4) != 500)
        throw std::runtime_error("500ms临时失联停机回读失败");
    ready_ = true;
}

MotorFeedback Motor::read()
{
    if (!ready_) throw std::runtime_error("驱动未准备好或已锁定，请停止后重新核对");
    try {
        const auto position = query(0x36, 7), speed = query(0x35, 5);
        if (position[1] > 1 || speed[1] > 1) throw std::runtime_error("反馈方向格式错误");
        MotorFeedback feedback;
        feedback.angleDeg = number(position.data() + 2, 4) * 360.0 / 65536 * (position[1] ? -1 : 1);
        feedback.rpm = int(number(speed.data() + 2, 2)) * (speed[1] ? -1 : 1);
        feedback.currentMa = uint16_t(number(query(0x27, 4).data() + 1, 2));
        feedback.status = query(0x3A, 3)[1];
        if ((feedback.status & 13) != 1) throw std::runtime_error("驱动失能或堵转");
        return feedback;
    } catch (...) { ready_ = false; throw; }
}

void Motor::start(int signedRpm, uint8_t acceleration)
{
    if (!ready_ || active_ || signedRpm == 0 || signedRpm < -3000 || signedRpm > 3000)
        throw std::runtime_error("驱动状态或目标RPM错误");
    const unsigned rpm = unsigned(std::abs(signedRpm));
    active_ = true; // ACK丢失时仍需尝试停止。
    try { acknowledge({0xF6, uint8_t(signedRpm < 0), uint8_t(rpm >> 8), uint8_t(rpm), acceleration, 0, 0x6B}); }
    catch (...) { ready_ = false; throw; }
}

void Motor::move(double absoluteMotorDeg, unsigned rpm, uint8_t acceleration, bool synchronous)
{
    const double pulses = std::round(std::abs(absoluteMotorDeg) * pulsesPerTurn_ / 360.0);
    if (!ready_ || !std::isfinite(absoluteMotorDeg) || pulses > std::numeric_limits<uint32_t>::max() ||
        rpm < 1 || rpm > 3000) throw std::runtime_error("绝对位置或速度超出Emm范围");
    const auto target = uint32_t(pulses);
    active_ = true;
    try {
        acknowledge({0xFD, uint8_t(absoluteMotorDeg < 0), uint8_t(rpm >> 8), uint8_t(rpm), acceleration,
                     uint8_t(target >> 24), uint8_t(target >> 16), uint8_t(target >> 8), uint8_t(target),
                     1, uint8_t(synchronous), 0x6B});
    } catch (...) { ready_ = false; throw; }
}

void Motor::synchronize(Can& can)
{
    // 广播仅发一次。单个广播ACK不代表四轴到位；上层随后逐轴核对新反馈。
    can_frame frame{};
    frame.can_id = CAN_EFF_FLAG;
    frame.can_dlc = 3;
    frame.data[0] = 0xFF; frame.data[1] = 0x66; frame.data[2] = 0x6B;
    can.send(frame);
}

void Motor::requestStop() { send({0xFE, 0x98, 0, 0x6B}); }

void Motor::stop()
{
    try {
        acknowledge({0xFE, 0x98, 0, 0x6B});
        const auto end = Clock::now() + std::chrono::milliseconds(500);
        unsigned stable = 0;
        while (Clock::now() < end) {
            const auto speed = query(0x35, 5);
            if (speed[1] > 1) throw std::runtime_error("停止反馈方向格式错误");
            stable = number(speed.data() + 2, 2) == 0 ? stable + 1 : 0;
            if (stable == 3) { active_ = false; return; }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        throw std::runtime_error("未确认连续零速度");
    } catch (...) { ready_ = false; throw; }
}
} // namespace robot
