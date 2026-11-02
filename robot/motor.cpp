#include "motor.hpp"
#include <algorithm>
#include <chrono>
#include <stdexcept>
#include <thread>

namespace robot {
namespace {
constexpr double ReplyTimeoutMs = 100;
constexpr uint32_t WatchdogMs = 500;

double nowMs() {
    return std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}
void require(bool ok, const char* message) {
    if (!ok) throw std::runtime_error(message);
}
uint32_t number(const uint8_t* p, int count) {
    uint32_t value = 0;
    for (int i = 0; i < count; ++i) value = (value << 8) | p[i];
    return value;
}
} // namespace

Motor::Motor(Can& can, unsigned node) : can_(can), nodeId_(CAN_EFF_FLAG | (node << 8)) {
    require(node >= 1 && node <= 255, "电机地址必须为1～255");
}

Motor::~Motor() {
    if (moving_) stopOnFailure();
}

can_frame Motor::exchange(std::initializer_list<uint8_t> bytes, uint8_t replyLength) {
    require(bytes.size() >= 2 && bytes.size() <= 8, "CAN指令长度错误");
    can_frame tx{};
    tx.can_id = nodeId_;
    tx.can_dlc = uint8_t(bytes.size());
    std::copy(bytes.begin(), bytes.end(), tx.data);
    can_.send(tx);
    const double deadline = nowMs() + ReplyTimeoutMs;
    while (nowMs() < deadline) {
        can_frame rx{};
        if (!can_.receive(rx, int(std::max(1., deadline - nowMs())))) break;
        if (rx.can_id != nodeId_ || !rx.can_dlc || rx.data[0] != tx.data[0]) continue;
        require(rx.can_dlc == replyLength && rx.data[replyLength - 1] == 0x6B, "CAN应答格式错误");
        return rx;
    }
    throw std::runtime_error("电机应答超时，ID=" + std::to_string(nodeId_ & CAN_EFF_MASK) + ", 功能码=" + std::to_string(tx.data[0]));
}

can_frame Motor::query(uint8_t code, uint8_t length) {
    return exchange({code, 0x6B}, length);
}

void Motor::prepare() {
    require(!moving_, "先停止，再核对驱动");
    ready_ = false;
    const auto mode = query(0x1A, 4);
    require(number(mode.data + 1, 2) == 6, "要求当前Emm闭环/整数RPM配置");
    require((query(0x3B, 3).data[1] & 0x33) == 3, "编码器/校准或温度电流保护异常");
    require((query(0x3A, 3).data[1] & 13) == 1, "未使能或堵转，不自动清故障");
    const auto voltage = number(query(0x24, 4).data + 1, 2);
    require(voltage > 20000 && voltage < 27000, "当前24V台架母线电压异常");
    require(number(query(0x35, 5).data + 2, 2) == 0, "电机尚未静止");
    if (number(query(0x16, 6).data + 1, 4) != WatchdogMs) {
        // 68 38 后的00为不存Flash；此参数是F6失联停机的必要前提。
        require(exchange({0x68, 0x38, 0, 0, 0, 1, 0xF4, 0x6B}, 3).data[1] == 2, "失联停机设置被拒绝");
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    require(number(query(0x16, 6).data + 1, 4) == WatchdogMs, "失联停机回读失败");
    ready_ = true;
}

void Motor::start(int signedRpm) {
    require(ready_ && !moving_, "未准备就绪或已在运行，禁止周期重发F6");
    require(signedRpm != 0 && signedRpm >= -3000 && signedRpm <= 3000, "RPM必须为非零且幅值不超过3000");
    require(read().rpm == 0, "启动前电机尚未静止");
    require(number(query(0x16, 6).data + 1, 4) == WatchdogMs, "失联停机配置已变化");
    const unsigned rpm = unsigned(signedRpm < 0 ? -signedRpm : signedRpm);
    moving_ = true; // 发出后即使ACK丢失，也要尝试停止。
    startedMs_ = nowMs();
    try {
        const auto reply = exchange({0xF6, uint8_t(signedRpm < 0), uint8_t(rpm >> 8), uint8_t(rpm), 1, 0, 0x6B}, 3);
        require(reply.data[1] == 2, "F6速度命令被拒绝");
    } catch (...) {
        ready_ = false;
        stopOnFailure();
        throw;
    }
}

MotorFeedback Motor::read() {
    require(ready_, "请先prepare；故障后禁止继续用读取维持旧运动");
    try {
        require(!moving_ || nowMs() - startedMs_ < 34000, "达到本次34秒运行上限");
        const auto position = query(0x36, 7);
        const auto velocity = query(0x35, 5);
        require(position.data[1] <= 1 && velocity.data[1] <= 1, "反馈符号错误");
        MotorFeedback result;
        result.angleDeg = number(position.data + 2, 4) * 360.0 / 65536 * (position.data[1] ? -1 : 1);
        result.rpm = int(number(velocity.data + 2, 2)) * (velocity.data[1] ? -1 : 1);
        result.currentMa = uint16_t(number(query(0x27, 4).data + 1, 2));
        result.status = query(0x3A, 3).data[1];
        require((result.status & 13) == 1, "驱动失能或堵转");
        require(result.currentMa <= 2800, "超过本次台架2800mA停止阈值");
        return result; // 不以目标/实测转速偏差自动判定性能失败。
    } catch (...) {
        ready_ = false;
        if (moving_) stopOnFailure();
        throw;
    }
}

void Motor::stop() {
    try {
        require(exchange({0xFE, 0x98, 0, 0x6B}, 3).data[1] == 2, "停止未获确认");
        const double deadline = nowMs() + 750;
        int stable = 0;
        while (nowMs() < deadline) {
            stable = number(query(0x35, 5).data + 2, 2) == 0 ? stable + 1 : 0;
            if (stable == 3) { moving_ = false; return; }
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        throw std::runtime_error("停止后未确认连续零转速");
    } catch (...) { ready_ = false; throw; }
}

void Motor::stopOnFailure() noexcept {
    try { stop(); } catch (...) {} // 失败后不再喂心跳；500ms失联保护仍是最后一层。
}

} // namespace robot
