// 只在unshare新建的网络命名空间、vcan0运行；不会接真实USB-CAN。
#include "x42s.hpp"
#include "motion_filter.hpp"
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <poll.h>
#include <unistd.h>
#include <atomic>
#include <chrono>
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <thread>
#include <cmath>

void require(bool ok, const char *message) {
    if (!ok)
        throw std::runtime_error(message);
}
struct Motors {
    int fd = -1, mode;
    std::atomic<bool> running{true};
    std::atomic<int> syncs{0}, stops{0};
    std::thread worker;
    explicit Motors(int fault) : mode(fault) {
        fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        ifreq request{};
        std::strcpy(request.ifr_name, "vcan0");
        require(ioctl(fd, SIOCGIFINDEX, &request) == 0, "测试vcan0不存在");
        sockaddr_can address{};
        address.can_family = AF_CAN;
        address.can_ifindex = request.ifr_ifindex;
        require(bind(fd, reinterpret_cast<sockaddr *>(&address), sizeof(address)) == 0,
                "测试bind失败");
        worker = std::thread([&] { run(); });
    }
    ~Motors() {
        running = false;
        worker.join();
        close(fd);
    }
    void reply(int node, Bytes p) {
        can_frame frame{};
        frame.can_id = CAN_EFF_FLAG | uint32_t(node << 8);
        frame.can_dlc = p.size();
        std::copy(p.begin(), p.end(), frame.data);
        if (write(fd, &frame, sizeof(frame)) != sizeof(frame))
            running = false;
    }
    void run() {
        std::array<Bytes, 4> packets;
        std::array<X42Move, 4> moves{};
        bool started = false;
        while (running) {
            pollfd ready{fd, POLLIN, 0};
            if (poll(&ready, 1, 10) <= 0)
                continue;
            can_frame f{};
            if (read(fd, &f, sizeof(f)) != sizeof(f))
                continue;
            uint32_t id = f.can_id & CAN_EFF_MASK;
            int node = id >> 8;
            Bytes p(f.data, f.data + f.can_dlc);
            if (id == 0 && p == Bytes({0xFF, 0x66, 0x6B})) {
                ++syncs;
                started = true;
                reply(1, {0xFF, 2, 0x6B});
                for (int n = 1; n <= 4; ++n)
                    reply(n, {0xFD, 0x9F, 0x6B});
                continue;
            }
            if (node < 1 || node > 4 || p.empty())
                continue;
            auto &packet = packets[node - 1];
            if (p[0] == 0xFD) {
                if (!(id & 255))
                    packet = p;
                else {
                    packet.insert(packet.end(), p.begin() + 1, p.end());
                    uint32_t raw = 0;
                    for (int j = 8; j < 12; ++j)
                        raw = (raw << 8) | packet.at(j);
                    moves[node - 1].angle = (packet[1] ? -.1 : .1) * raw;
                    if (mode == 1 && node == 3)
                        continue;
                    reply(node, {0xFD, uint8_t(mode == 2 && node == 2 ? 0xE2 : 2), 0x6B});
                }
            } else if (p[0] == 0x1A)
                reply(node, {0x1A, uint8_t(mode == 6 ? 6 : 4), 0x6B});
            else if (p[0] == 0x1F)
                reply(node, {0x1F, 0x20, 0x10, 0x6B});
            else if (p[0] == 0x16)
                reply(node, {0x16, 0, 0, uint8_t(mode == 7 ? 0 : 1), uint8_t(mode == 7 ? 0 : 0xF4),
                             0x6B});
            else if (p[0] == 0x3A)
                reply(node, {0x3A, uint8_t(mode == 3 && node == 2 ? 0x0D : 3), 0x6B});
            else if (p[0] == 0x35)
                reply(node, {0x35, 0, 0, 0, 0x6B});
            else if (p[0] == 0x36) {
                if (mode == 4 && node == 3)
                    continue;
                double angle = started && mode != 5 ? moves[node - 1].angle : 0;
                uint32_t raw = std::llround(std::abs(angle) * 10);
                reply(node, {0x36, uint8_t(angle < 0), uint8_t(raw >> 24), uint8_t(raw >> 16),
                             uint8_t(raw >> 8), uint8_t(raw), 0x6B});
            } else if (p[0] == 0xFE) {
                ++stops;
                reply(node, {0xFE, 2, 0x6B});
            }
        }
    }
};
int main(int argc, char **argv) {
    try {
        require(argc == 2 && std::string(argv[1]) == "vcan0", "只允许隔离的vcan0测试");
        auto frames = x42Split(1, x42Position({-3600, 10000, 511}));
        require(frames.size() == 2 && frames[0].id == 0x100 && frames[1].id == 0x101 &&
                    frames[0].data == Bytes({0xFD, 1, 1, 0xFF, 1, 0xFF, 0x27, 0x10}) &&
                    frames[1].data == Bytes({0xFD, 0, 0, 0x8C, 0xA0, 1, 1, 0x6B}),
                "厂家报文字节不符");
        bool refused = false;
        try {
            x42Position({0, 30001, 1});
        } catch (...) {
            refused = true;
        }
        require(refused, "超协议速度未拒绝");
        std::array<X42Sample, 4> samples{};
        for (const auto &bad :
             std::vector<X42Frame>{{0x100, {}}, {0x100, {0x36, 0x6B}}, {0x101, {0xFD, 2, 0x6B}}}) {
            refused = false;
            try {
                x42Decode(bad, samples, 0);
            } catch (...) {
                refused = true;
            }
            require(refused, "畸形反馈未拒绝");
        }
        MotionFilter filter(195, 275);
        filter.reset({.5, 1});
        filter.step({.5, 1}, .02);
        require(!filter.usedXY, "编码器KF被冒称EKF");
        cv::Point2d point(195 * std::cos(.5) + 275 * std::cos(1.5),
                          195 * std::sin(.5) + 275 * std::sin(1.5));
        filter.step({.5, 1}, .02, &point);
        require(filter.usedXY, "新鲜XY未进入EKF");
        filter.step({.5, 1}, .02, &point, .2);
        require(!filter.usedXY, "过期XY未拒绝");
        point.x += 1000;
        filter.step({.5, 1}, .02, &point);
        require(!filter.usedXY, "异常XY未拒绝");
        for (int mode = 0; mode <= 7; ++mode) {
            Motors motors(mode);
            X42Can bus("vcan0");
            int feedbacks = 0;
            auto start = std::chrono::steady_clock::now();
            bool failed = false;
            try {
                bus.move({40, -80, 20, 0}, 60, 120, [&](const auto &, double) { ++feedbacks; });
            } catch (const std::exception &) {
                failed = true;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(30));
            require(failed == (mode != 0), "故障没有按预期中止");
            if (!mode)
                require(feedbacks >= 3 && motors.syncs == 1, "正常反馈/同步未完成");
            else
                require(motors.stops == 4, "异常后未逐轴发送停止");
            if (mode == 1 || mode == 2)
                require(motors.syncs == 0, "收包不全仍启动了同步");
            require(
                std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count() < 3,
                "故障退出没有界限");
            std::cout << "case " << mode << " PASS\n";
        }
        std::cout << "SocketCAN/vcan、分包、四轴ACK、反馈到位、故障停止、KF/EKF PASS\n";
    } catch (const std::exception &e) {
        std::cerr << e.what() << '\n';
        return 1;
    }
}
