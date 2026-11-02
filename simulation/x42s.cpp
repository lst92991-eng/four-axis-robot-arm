// 依据X42S手册V1.0.5：经典扩展CAN，固定6B；只支持本项目所需X固件子集。
#include "x42s.hpp"
#include "debug.hpp"
#include <linux/can.h>
#include <linux/can/error.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <poll.h>
#include <unistd.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <stdexcept>
#include <thread>

namespace {
double now() {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch())
        .count();
}
} // namespace
Bytes x42Position(const X42Move &m) {
    if (!std::isfinite(m.angle) || std::abs(m.angle) * 10 > UINT32_MAX || m.speed == 0 ||
        m.speed > 30000 || m.accel == 0)
        throw std::runtime_error("X位置/速度/加速度超界");
    uint32_t angle = std::llround(std::abs(m.angle) * 10);
    return {0xFD,
            uint8_t(m.angle < 0),
            uint8_t(m.accel >> 8),
            uint8_t(m.accel),
            uint8_t(m.accel >> 8),
            uint8_t(m.accel),
            uint8_t(m.speed >> 8),
            uint8_t(m.speed),
            uint8_t(angle >> 24),
            uint8_t(angle >> 16),
            uint8_t(angle >> 8),
            uint8_t(angle),
            1,
            1,
            0x6B}; // 绝对位置、先缓存；全部确认收包后再FF66同步。
}
std::vector<X42Frame> x42Split(uint8_t node, const Bytes &command) {
    if (command.size() < 2 || command.size() > 15 || command.back() != 0x6B)
        throw std::runtime_error("X命令长度/校验错误");
    std::vector<X42Frame> result;
    size_t offset = 1;
    do {
        Bytes data{command[0]};
        size_t end = std::min(offset + 7, command.size());
        data.insert(data.end(), command.begin() + offset, command.begin() + end);
        result.push_back({uint32_t(node << 8) | uint32_t(result.size()), data});
        offset = end;
    } while (offset < command.size());
    return result;
}
void x42Decode(const X42Frame &frame, std::array<X42Sample, 4> &samples, double time) {
    int node = (frame.id >> 8) - 1;
    const auto &p = frame.data;
    if (node < 0 || node >= 4 || (frame.id & 255) || p.size() < 3 || p.back() != 0x6B)
        throw std::runtime_error("X应答节点/包号/长度/校验错误");
    auto &s = samples[node];
    if ((p[0] == 0x36 && p.size() == 7) || (p[0] == 0x35 && p.size() == 5)) {
        if (p[1] > 1)
            throw std::runtime_error("X反馈符号错误");
        uint32_t raw = 0;
        for (size_t i = 2; i + 1 < p.size(); ++i)
            raw = (raw << 8) | p[i];
        double value = (p[1] ? -.1 : .1) * raw;
        if (p[0] == 0x36) {
            s.angle = value;
            s.positionTime = time;
        } else {
            s.rpm = value;
            s.speedTime = time;
        }
    } else if (p[0] == 0x3A && p.size() == 3) {
        s.status = p[1];
        s.statusTime = time;
        if (s.status & 0x0C)
            throw std::runtime_error("电机堵转/堵转保护；停止且不自动解除");
    } else if (p[0] == 0x1A && p.size() == 3)
        s.options = p[1];
    else if (p[0] == 0x1F && p.size() == 4) {
        s.firmware = p[1];
        s.hardware = p[2];
    } else if (p[0] == 0x16 && p.size() == 6) {
        s.heartbeatMs = 0;
        for (int i = 1; i < 5; ++i)
            s.heartbeatMs = (s.heartbeatMs << 8) | p[i];
    } else if (p[0] == 0xFD && p.size() == 3 && p[1] == 0x9F)
        s.arrived = true;
    else if (p[0] == 0xFD && p.size() == 3 && p[1] == 0x02)
        s.acknowledged = true;
    else if ((p[0] == 0xFF || p[0] == 0xFE) && p.size() == 3 && p[1] == 0x02) {
    } else
        throw std::runtime_error("X驱动拒绝命令/不支持的应答");
}
double x42Duration(double distance, const X42Move &m) {
    const double v = m.speed * .6, a = m.accel * 6;
    return distance < v * v / a ? 2 * std::sqrt(distance / a) : distance / v + v / a;
}
std::array<X42Move, 4> x42Plan(const std::array<double, 4> &from, const std::array<double, 4> &to,
                               double maxRpm, double accelRpm) {
    if (!std::isfinite(maxRpm) || !std::isfinite(accelRpm) || maxRpm < .1 || maxRpm > 3000 ||
        accelRpm < 1 || accelRpm > 65535)
        throw std::runtime_error("X运动限值错误");
    double largest = 0;
    for (int i = 0; i < 4; ++i) {
        if (!std::isfinite(from[i]) || !std::isfinite(to[i]))
            throw std::runtime_error("X目标不是有限数");
        largest = std::max(largest, std::abs(to[i] - from[i]));
    }
    std::array<X42Move, 4> moves{};
    for (int i = 0; i < 4; ++i) {
        double ratio = largest > 0 ? std::abs(to[i] - from[i]) / largest : 0;
        moves[i] = {to[i], uint16_t(std::max(1., std::floor(maxRpm * 10 * ratio))),
                    uint16_t(std::max(1., std::floor(accelRpm * ratio)))};
    }
    return moves; // 速度和加速度均按位移缩放；量化会使同步进度有小偏差，需实测。
}
X42Can::X42Can(const char *interface) {
    fd = socket(PF_CAN, SOCK_RAW | SOCK_NONBLOCK, CAN_RAW);
    if (fd < 0)
        throw std::runtime_error("内核缺少SocketCAN");
    try {
        ifreq request{};
        if (std::strlen(interface) >= IFNAMSIZ)
            throw std::runtime_error("CAN接口名称过长");
        std::strcpy(request.ifr_name, interface);
        if (ioctl(fd, SIOCGIFINDEX, &request) != 0)
            throw std::runtime_error("CAN接口不存在；不会自动配置/刷固件");
        sockaddr_can address{};
        address.can_family = AF_CAN;
        address.can_ifindex = request.ifr_ifindex;
        if (bind(fd, reinterpret_cast<sockaddr *>(&address), sizeof(address)) != 0)
            throw std::runtime_error("CAN绑定失败");
        can_err_mask_t errors = CAN_ERR_MASK;
        if (setsockopt(fd, SOL_CAN_RAW, CAN_RAW_ERR_FILTER, &errors, sizeof(errors)) != 0)
            throw std::runtime_error("CAN错误帧订阅失败");
        // 不接收本socket自己的命令回显；只处理1..4节点的扩展数据帧及总线错误。
        std::array<can_filter, 4> filters{};
        for (int i = 0; i < 4; ++i)
            filters[i] = {CAN_EFF_FLAG | uint32_t((i + 1) << 8),
                          CAN_EFF_FLAG | CAN_RTR_FLAG | 0x1FFFFF00};
        if (setsockopt(fd, SOL_CAN_RAW, CAN_RAW_FILTER, filters.data(), sizeof(filters)) != 0)
            throw std::runtime_error("CAN过滤设置失败");
    } catch (...) {
        close(fd);
        fd = -1;
        throw;
    }
}
X42Can::~X42Can() {
    if (fd >= 0)
        close(fd);
}
void X42Can::send(const X42Frame &frame) {
    if (frame.data.empty() || frame.data.size() > 8 || frame.id > CAN_EFF_MASK)
        throw std::runtime_error("经典CAN帧格式错误");
    can_frame raw{};
    raw.can_id = frame.id | CAN_EFF_FLAG;
    raw.can_dlc = frame.data.size();
    std::copy(frame.data.begin(), frame.data.end(), raw.data);
    if (write(fd, &raw, sizeof(raw)) != sizeof(raw))
        throw std::runtime_error("CAN发送失败/队列已满");
    if (debug.can)
        debug.can(true, frame.id, frame.data.data(), frame.data.size());
}
bool X42Can::read(X42Frame &frame, int timeoutMs) {
    pollfd ready{fd, POLLIN, 0};
    int count = poll(&ready, 1, timeoutMs);
    if (count < 0 || (ready.revents & (POLLERR | POLLHUP | POLLNVAL)))
        throw std::runtime_error("CAN接口错误");
    if (!count)
        return false;
    can_frame raw{};
    if (::read(fd, &raw, sizeof(raw)) != sizeof(raw))
        throw std::runtime_error("CAN读取长度错误");
    if (raw.can_id & CAN_ERR_FLAG)
        throw std::runtime_error("CAN总线错误；不自动重启/续跑");
    if (!(raw.can_id & CAN_EFF_FLAG) || (raw.can_id & CAN_RTR_FLAG) || raw.can_dlc > 8)
        throw std::runtime_error("CAN应答必须是经典扩展数据帧");
    frame = {raw.can_id & CAN_EFF_MASK, Bytes(raw.data, raw.data + raw.can_dlc)};
    if (debug.can)
        debug.can(false, frame.id, frame.data.data(), frame.data.size());
    return true;
}
void X42Can::command(uint8_t node, const Bytes &bytes) {
    for (const auto &frame : x42Split(node, bytes))
        send(frame);
}
void X42Can::stop() {
    // 逐节点发送停止，不依赖地址1存在；链路故障时只能尽力，不能保证实物停住。
    for (int node = 1; node <= 4; ++node) {
        try {
            command(node, {0xFE, 0x98, 0, 0x6B});
        } catch (...) {
        }
    }
}
void X42Can::move(const std::array<double, 4> &goal, double maxRpm, double accelRpm,
                  const std::function<void(const std::array<X42Sample, 4> &, double)> &feedback) {
    std::array<X42Sample, 4> samples{};
    auto await = [&](int node, uint8_t code, const Bytes &bytes) {
        command(node, bytes);
        double deadline = now() + .2;
        X42Frame frame;
        while (now() < deadline) {
            if (!read(frame, 5))
                continue;
            x42Decode(frame, samples, now());
            if ((frame.id >> 8) == uint32_t(node) && frame.data[0] == code)
                return;
        }
        throw std::runtime_error("X节点应答超时");
    };
    try {
        X42Frame stale;
        int drained = 0;
        while (read(stale, 0) && drained < 64)
            ++drained;
        if (drained >= 64)
            throw std::runtime_error("CAN接收队列过载；不使用旧反馈开始运动");
        std::array<double, 4> from{};
        for (int node = 1; node <= 4; ++node) {
            await(node, 0x1A, {0x1A, 0x6B});
            if ((samples[node - 1].options & 0x82) || !(samples[node - 1].options & 4))
                throw std::runtime_error("要求X闭环固件且关闭缩小10倍输入；不自动改配置");
            await(node, 0x1F, {0x1F, 0x6B}); // 原始版本字节；不能据此猜完整补丁号。
            await(node, 0x16, {0x16, 0x6B});
            // 20ms查询周期预留延迟余量；真实接机前由用户核对/设置，不自动写参数。
            if (samples[node - 1].heartbeatMs < 100 || samples[node - 1].heartbeatMs > 1000)
                throw std::runtime_error("驱动心跳保护须为100..1000ms，建议先验证500ms");
            await(node, 0x3A, {0x3A, 0x6B});
            if (!(samples[node - 1].status & 1))
                throw std::runtime_error("驱动未使能；不自动使能");
            await(node, 0x36, {0x36, 0x6B});
            from[node - 1] = samples[node - 1].angle;
        }
        auto moves = x42Plan(from, goal, maxRpm, accelRpm);
        double duration = 0;
        for (int node = 1; node <= 4; ++node) {
            duration = std::max(
                duration, x42Duration(std::abs(goal[node - 1] - from[node - 1]), moves[node - 1]));
            await(node, 0xFD, x42Position(moves[node - 1])); // 收到四轴02，才允许广播启动。
            if (!samples[node - 1].acknowledged)
                throw std::runtime_error("X位置缓存未被确认");
        }
        command(0, {0xFF, 0x66, 0x6B});
        double start = now(), deadline = start + duration + 1, previous = start;
        int stable = 0;
        while (now() < deadline) {
            double cycle = now();
            for (int node = 1; node <= 4; ++node) {
                await(node, 0x36, {0x36, 0x6B});
                await(node, 0x35, {0x35, 0x6B});
                await(node, 0x3A, {0x3A, 0x6B});
            }
            double timestamp = now();
            bool done = true;
            for (int i = 0; i < 4; ++i) {
                const auto &s = samples[i];
                if (timestamp - s.positionTime >= .1 || timestamp - s.speedTime >= .1 ||
                    timestamp - s.statusTime >= .1)
                    throw std::runtime_error("四轴反馈过期");
                if (!(s.status & 1))
                    throw std::runtime_error("运行中驱动失能");
                done &=
                    (s.status & 2) && std::abs(s.angle - goal[i]) <= .2 && std::abs(s.rpm) <= .5;
            }
            if (feedback)
                feedback(samples, timestamp - previous);
            previous = timestamp;
            stable = done ? stable + 1 : 0;
            if (stable >= 3)
                return; // 状态、位置、速度连续满足；单独9F不代表实物已稳定。
            std::this_thread::sleep_until(std::chrono::steady_clock::time_point(
                std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                    std::chrono::duration<double>(cycle + .02))));
        }
        throw std::runtime_error("X四轴到位超时");
    } catch (...) {
        stop();
        throw;
    }
}
