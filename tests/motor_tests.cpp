// 仅在显式指定隔离vcan0时运行；假节点验证报文，不代表机械动态响应。
#ifdef NDEBUG
#undef NDEBUG
#endif
#include "robot/motor.hpp"
#include <linux/can/raw.h>
#include <net/if.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>
#include <atomic>
#include <cassert>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <thread>

namespace {
enum Mode { Normal, WrongMode, BadWatchdog, DropStartAck, Stall, DropPosition, BadLength, CanError, DropStopAck };
uint64_t packed(std::initializer_list<uint8_t> data) {
    uint64_t value = 0;
    for (auto b : data) value = (value << 8) | b;
    return value;
}
class FakeMotor {
public:
    std::atomic<int> mode{Normal}, starts{0}, stops{0}, requests{0};
    std::atomic<uint64_t> lastStart{0};
    std::atomic<bool> stored{false};
    explicit FakeMotor(unsigned node = 1) : id_(CAN_EFF_FLAG | (node << 8)) {
        fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        assert(fd_ >= 0);
        ifreq request{};
        std::strcpy(request.ifr_name, "vcan0");
        assert(ioctl(fd_, SIOCGIFINDEX, &request) == 0);
        sockaddr_can address{};
        address.can_family = AF_CAN;
        address.can_ifindex = request.ifr_ifindex;
        assert(bind(fd_, reinterpret_cast<sockaddr*>(&address), sizeof(address)) == 0);
        can_filter filter{id_, CAN_EFF_FLAG | CAN_RTR_FLAG | CAN_EFF_MASK};
        assert(setsockopt(fd_, SOL_CAN_RAW, CAN_RAW_FILTER, &filter, sizeof(filter)) == 0);
        thread_ = std::thread([this] { run(); });
    }
    ~FakeMotor() { quit_ = true; thread_.join(); close(fd_); }
private:
    void reply(std::initializer_list<uint8_t> bytes) {
        can_frame notice{};
        notice.can_id = CAN_ERR_FLAG | 4;
        notice.can_dlc = 8;
        assert(write(fd_, &notice, sizeof(notice)) == sizeof(notice));
        can_frame frame{};
        frame.can_id = id_;
        frame.can_dlc = uint8_t(bytes.size());
        std::copy(bytes.begin(), bytes.end(), frame.data);
        assert(write(fd_, &frame, sizeof(frame)) == sizeof(frame));
    }
    void run() {
        uint32_t heartbeat = 0;
        unsigned rpm = 0;
        uint8_t direction = 0;
        auto last = std::chrono::steady_clock::now();
        while (!quit_) {
            if (heartbeat && std::chrono::steady_clock::now() - last > std::chrono::milliseconds(heartbeat)) rpm = 0;
            pollfd pending{fd_, POLLIN, 0};
            if (poll(&pending, 1, 10) <= 0) continue;
            can_frame frame{};
            if (::read(fd_, &frame, sizeof(frame)) != sizeof(frame)) continue;
            ++requests;
            last = std::chrono::steady_clock::now();
            const auto* d = frame.data;
            switch (d[0]) {
            case 0x1A: reply({0x1A, 0, uint8_t(mode == WrongMode ? 4 : 6), 0x6B}); break;
            case 0x3B: reply({0x3B, 3, 0x6B}); break;
            case 0x24: reply({0x24, 0x5D, 0xC0, 0x6B}); break;
            case 0x16: reply({0x16, uint8_t(heartbeat >> 24), uint8_t(heartbeat >> 16), uint8_t(heartbeat >> 8), uint8_t(heartbeat), 0x6B}); break;
            case 0x68:
                assert(frame.can_dlc == 8 && d[1] == 0x38);
                stored = d[2] != 0;
                if (mode != BadWatchdog) heartbeat = uint32_t(d[3]) << 24 | uint32_t(d[4]) << 16 | uint32_t(d[5]) << 8 | d[6];
                reply({0x68, 2, 0x6B});
                break;
            case 0x36:
                if (mode == DropPosition) break;
                if (mode == CanError) {
                    can_frame error{}; error.can_id = CAN_ERR_FLAG | 0x24; error.can_dlc = 8;
                    assert(write(fd_, &error, sizeof(error)) == sizeof(error));
                } else if (mode == BadLength) reply({0x36, 0, 0x6B});
                else reply({0x36, direction, 0, 0, 0x40, 0, 0x6B});
                break;
            case 0x35: reply({0x35, direction, uint8_t(rpm >> 8), uint8_t(rpm), 0x6B}); break;
            case 0x27: reply({0x27, 2, 0x58, 0x6B}); break;
            case 0x3A: reply({0x3A, uint8_t(mode == Stall ? 5 : (rpm ? 1 : 3)), 0x6B}); break;
            case 0xF6:
                assert(frame.can_dlc == 7);
                direction = d[1]; rpm = unsigned(d[2]) << 8 | d[3]; ++starts;
                lastStart = packed({d[0], d[1], d[2], d[3], d[4], d[5], d[6]});
                if (mode != DropStartAck) reply({0xF6, 2, 0x6B});
                break;
            case 0xFE:
                assert(frame.can_dlc == 4 && d[1] == 0x98 && d[2] == 0);
                rpm = 0; ++stops;
                if (mode != DropStopAck) reply({0xFE, 2, 0x6B});
                break;
            default: assert(false && "unexpected command");
            }
        }
    }
    canid_t id_;
    int fd_ = -1;
    std::atomic<bool> quit_{false};
    std::thread thread_;
};
template <typename F> void fails(F action) {
    bool caught = false;
    try { action(); } catch (const std::runtime_error&) { caught = true; }
    assert(caught);
}
} // namespace

void testMotor() {
    const char* interface = std::getenv("SCARA_TEST_CAN_INTERFACE");
    if (!interface) { std::cout << "Motor vcan test skipped (no explicit interface)\n"; return; }
    assert(std::strcmp(interface, "vcan0") == 0); // 运行命令必须先unshare隔离并新建vcan0。
    {
        // CAN层不依赖电机：标准/扩展帧透明收发，接收期限有限。
        robot::Can sender(interface), receiver(interface);
        can_frame sent{}, received{};
        for (canid_t id : {canid_t(0x123), canid_t(CAN_EFF_FLAG | 0x12345)}) {
            sent.can_id = id; sent.can_dlc = 2; sent.data[0] = 0xAA; sent.data[1] = 0x55;
            sender.send(sent);
            assert(receiver.receive(received, 50));
            assert(received.can_id == id && received.can_dlc == 2 && received.data[1] == 0x55);
        }
        assert(!receiver.receive(received, 20));
        sent.can_dlc = 9;
        fails([&] { sender.send(sent); });
        sent.can_dlc = 2; sent.can_id = 0x800;
        fails([&] { sender.send(sent); });
        fails([&] { robot::Motor invalid(sender, 0); });
        fails([&] { robot::Motor invalid(sender, 256); });
    }
    {
        // 虚拟地址7用于验证寻址；不表示实机的其他轴地址已经确认。
        FakeMotor motor(7);
        robot::Can can(interface);
        robot::Motor drive(can, 7);
        drive.prepare(); drive.start(5);
        assert(drive.read().rpm == 5);
        drive.stop();
    }
    {
        FakeMotor motor;
        robot::Can can(interface);
        robot::Motor drive(can);
        assert(motor.requests == 0); // 构造只打开接口，不发任何电机指令。
        fails([&] { drive.start(500); });
        drive.prepare();
        assert(!motor.stored);
        for (int rpm : {0, 3001, -3001}) fails([&] { drive.start(rpm); });
        drive.start(-500);
        assert(motor.lastStart == packed({0xF6, 1, 1, 0xF4, 1, 0, 0x6B}));
        auto value = drive.read();
        assert(value.rpm == -500 && value.angleDeg == -90 && value.currentMa == 600);
        fails([&] { drive.start(-500); });
        assert(motor.starts == 1);
        drive.stop();
        assert(drive.read().rpm == 0 && motor.stops == 1);
        drive.start(500);
        std::this_thread::sleep_for(std::chrono::milliseconds(650));
        assert(drive.read().rpm == 0); // 假节点的500ms失联停机。
        drive.stop();
    }
    {
        FakeMotor motor;
        robot::Can can(interface);
        robot::Motor drive(can);
        drive.prepare(); drive.start(5);
        motor.mode = DropStopAck;
        fails([&] { drive.stop(); });
        const int count = motor.requests;
        fails([&] { drive.read(); });
        assert(motor.requests == count); // 停止未确认后也不能继续喂心跳。
        motor.mode = Normal;
        drive.stop();
    }
    for (int mode : {WrongMode, BadWatchdog}) {
        FakeMotor motor; motor.mode = mode;
        robot::Can can(interface);
        robot::Motor drive(can);
        fails([&] { drive.prepare(); });
        fails([&] { drive.start(500); });
        assert(motor.starts == 0);
    }
    for (int mode : {DropStartAck, Stall, DropPosition, BadLength, CanError}) {
        FakeMotor motor;
        robot::Can can(interface);
        robot::Motor drive(can);
        drive.prepare();
        if (mode == DropStartAck) {
            motor.mode = mode;
            fails([&] { drive.start(500); });
        } else {
            drive.start(500);
            motor.mode = mode;
            fails([&] { drive.read(); });
        }
        assert(motor.stops >= 1);
        const int count = motor.requests;
        fails([&] { drive.read(); });
        assert(motor.requests == count); // 锁定后不继续喂心跳维持旧运动。
    }
    std::cout << "Motor protocol and stop checks passed on isolated vcan0\n";
}
