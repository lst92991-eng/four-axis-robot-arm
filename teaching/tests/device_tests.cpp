#include "robot/motor.hpp"
#include "robot/magnet.hpp"
#include "motion/motion.hpp"
#include <linux/can/error.h>
#include <linux/can/raw.h>
#include <linux/gpio.h>
#include <net/if.h>
#include <poll.h>
#include <sys/socket.h>
#include <sys/wait.h>
#include <sys/eventfd.h>
#include <sys/syscall.h>
#include <fcntl.h>
#include <unistd.h>
#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cerrno>
#include <cstdarg>
#include <cstdlib>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <limits>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <vector>

// 只在测试可执行文件内拦截指定GPIO ioctl；生产库没有测试开关或注入接口。
namespace fakeGpio {
std::atomic<bool> enabled{false}, value{false}, failOff{false}, failOn{false};
std::atomic<int> handle{-1}, writes{0};
}
extern "C" int ioctl(int fd, unsigned long request, ...) noexcept
{
    va_list arguments;
    va_start(arguments, request);
    void* argument = va_arg(arguments, void*);
    va_end(arguments);
    if (fakeGpio::enabled && request == GPIO_GET_LINEHANDLE_IOCTL) {
        char path[128]{}, name[64]{};
        std::snprintf(name, sizeof(name), "/proc/self/fd/%d", fd);
        if (readlink(name, path, sizeof(path) - 1) > 0 && std::strcmp(path, "/dev/null") == 0) {
            auto* lines = static_cast<gpiohandle_request*>(argument);
            lines->fd = eventfd(0, EFD_CLOEXEC);
            if (lines->fd < 0) return -1;
            fakeGpio::handle = lines->fd;
            fakeGpio::value = lines->default_values[0] != 0;
            return 0;
        }
    }
    if (fakeGpio::enabled && fd == fakeGpio::handle && request == GPIOHANDLE_SET_LINE_VALUES_IOCTL) {
        const bool value = static_cast<gpiohandle_data*>(argument)->values[0] != 0;
        ++fakeGpio::writes;
        if ((value && fakeGpio::failOn) || (!value && fakeGpio::failOff)) { errno = EIO; return -1; }
        fakeGpio::value = value;
        return 0;
    }
    return int(syscall(SYS_ioctl, fd, request, argument));
}

namespace {
void check(bool value, const char* message)
{
    if (!value) throw std::runtime_error(message);
}
template<class F> void fails(F action)
{
    bool caught = false;
    try { action(); } catch (const std::runtime_error&) { caught = true; }
    check(caught, "expected failure");
}

template<class F> bool until(F predicate, int timeoutMs = 700)
{
    const auto end = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeoutMs);
    while (std::chrono::steady_clock::now() < end) {
        if (predicate()) return true;
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    return false;
}

void magnetCases()
{
    fakeGpio::enabled = true;
    {
        robot::Magnet magnet;
        fails([&] { magnet.setMaximumOnSeconds(0); });
        magnet.setMaximumOnSeconds(.12);
        magnet.open("/dev/null", 0, true);
        check(!magnet.on() && !fakeGpio::value, "explicit GPIO open starts off");
        magnet.set(true);
        fails([&] { magnet.setMaximumOnSeconds(1); });
        // 持续重复开磁跨过原期限：必须在原deadline关闭，不能借重复命令无限延期。
        bool locked = false;
        for (int i = 0; i < 20; ++i) {
            std::this_thread::sleep_for(std::chrono::milliseconds(15));
            try { magnet.set(true); } catch (const std::runtime_error&) { locked = true; break; }
        }
        check(locked && magnet.expired() && !magnet.on() && !fakeGpio::value,
              "independent GPIO deadline expires without lease renewal");
        fails([&] { magnet.set(true); });
        magnet.set(false);
    }
    {
        robot::Magnet magnet;
        magnet.setMaximumOnSeconds(.03);
        magnet.open("/dev/null", 0, true);
        magnet.set(true); fakeGpio::failOff = true;
        check(until([&] { return magnet.expired(); }), "GPIO deadline thread runs independently");
        check(magnet.on() && !magnet.failure().empty(), "failed release retains unknown/on state and fault");
        fails([&] { magnet.set(true); });
        fakeGpio::failOff = false;
        magnet.set(false);
        check(!magnet.on() && !fakeGpio::value && magnet.expired(), "explicit off retry succeeds but expiry stays latched");
    }
    {
        robot::Magnet magnet;
        magnet.setMaximumOnSeconds(.03);
        magnet.open("/dev/null", 0, true);
        fakeGpio::failOn = true;
        fails([&] { magnet.set(true); });
        check(magnet.on() && !magnet.failure().empty(), "failed on is conservatively possibly energized");
        fakeGpio::failOn = false;
        check(until([&] { return magnet.expired(); }) && !magnet.on(), "failed on still has bounded release lease");
    }
    fakeGpio::enabled = false;
    fakeGpio::handle = -1;
    std::cout << "devices: simulated GPIO lease, no-renewal and release-failure cases passed\n";
}

// 测试进程只接受隔离namespace中的vcan0；不是动态电机模型。
class FakeNode {
public:
    enum Fault { None, WrongMode, WrongPulses, DropPosition, DropMoveAck, Stall, BadPacket, CanError };
    std::atomic<int> fault{None}, requests{0}, moves{0}, starts{0}, stops{0}, syncs{0};
    std::atomic<int> commandedRpm{0};
    std::atomic<uint32_t> pulses{0};
    std::atomic<bool> negative{false}, queued{false}, wroteFlash{false}, validMove{false};
    std::atomic<bool> syncBeforeAllAcks{false};
    FakeNode(unsigned node = 1, double initialDegrees = -90, std::atomic<unsigned>* groupAcks = nullptr)
        : node_(node), encoder_(int64_t(std::llround(initialDegrees * 65536 / 360))), groupAcks_(groupAcks)
    {
        fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        check(fd_ >= 0, "fake CAN socket");
        sockaddr_can address{};
        address.can_family = AF_CAN;
        address.can_ifindex = int(if_nametoindex("vcan0"));
        check(address.can_ifindex != 0 && bind(fd_, reinterpret_cast<sockaddr*>(&address), sizeof(address)) == 0,
              "vcan0 must exist in isolated namespace");
        thread_ = std::thread([this] { run(); });
    }
    ~FakeNode() { quit_ = true; thread_.join(); close(fd_); }
    std::vector<unsigned> stopOrder()
    {
        std::lock_guard<std::mutex> lock(eventsMutex_);
        return stopOrder_;
    }
private:
    void reply(const std::vector<uint8_t>& data, bool reverse = false)
    {
        can_frame notice{};
        notice.can_id = CAN_ERR_FLAG | CAN_ERR_CRTL;
        notice.can_dlc = 8;
        (void)::write(fd_, &notice, sizeof(notice));
        const unsigned packets = unsigned((data.size() - 1 + 6) / 7);
        for (unsigned i = 0; i < packets; ++i) {
            const unsigned packet = reverse ? packets - 1 - i : i, offset = 1 + packet * 7;
            can_frame frame{};
            frame.can_id = CAN_EFF_FLAG | (node_ << 8) | packet;
            frame.can_dlc = uint8_t(1 + std::min<size_t>(7, data.size() - offset));
            frame.data[0] = data[0];
            std::copy_n(data.data() + offset, frame.can_dlc - 1, frame.data + 1);
            (void)::write(fd_, &frame, sizeof(frame));
        }
    }
    void run()
    {
        unsigned heartbeat = 0;
        int rpm = 0;
        std::vector<uint8_t> command;
        auto last = std::chrono::steady_clock::now();
        while (!quit_) {
            if (heartbeat && std::chrono::steady_clock::now() - last > std::chrono::milliseconds(heartbeat)) rpm = 0;
            pollfd pending{fd_, POLLIN, 0};
            if (poll(&pending, 1, 10) <= 0) continue;
            can_frame f{};
            if (::read(fd_, &f, sizeof(f)) != sizeof(f) || f.can_dlc < 2) continue;
            if (node_ == 1 && f.data[0] == 0xFE && f.can_dlc == 4) {
                std::lock_guard<std::mutex> lock(eventsMutex_);
                stopOrder_.push_back((f.can_id >> 8) & 255);
            }
            if (f.can_id == CAN_EFF_FLAG && f.data[0] == 0xFF) {
                ++syncs;
                if (groupAcks_ && *groupAcks_ != 4) syncBeforeAllAcks = true;
                if (queued) encoder_ = int64_t(std::llround(double(pulses) * 65536 / 3200)) * (negative ? -1 : 1);
                queued = false;
                continue;
            }
            if ((f.can_id & ~canid_t(255)) != (CAN_EFF_FLAG | (node_ << 8))) continue;
            ++requests;
            last = std::chrono::steady_clock::now();
            const auto* d = f.data;
            switch (d[0]) {
            case 0x1A: reply({0x1A, 0, uint8_t(fault == WrongMode ? 4 : 6), 0x6B}); break;
            case 0x1F: reply({0x1F, 0, 200, 3, 20, 0x6B}); break;
            case 0x3B: reply({0x3B, 3, 0x6B}); break;
            case 0x3A: reply({0x3A, uint8_t(fault == Stall ? 5 : (rpm ? 1 : 3)), 0x6B}); break;
            case 0x42: {
                std::vector<uint8_t> config{0x42,33,21,25,2,3,2,0,16,1,0,4,176,11,184,19,136,5,7,1,0,1,1,0,8,8,152,7,208,0,8,0x6B};
                config[19] = uint8_t(node_);
                if (fault == WrongPulses) config[8] = 32;
                // 真正交换次序，验证按包号重组而非依赖到达顺序。
                reply(config, true);
                break;
            }
            case 0x16: reply({0x16, uint8_t(heartbeat >> 24), uint8_t(heartbeat >> 16), uint8_t(heartbeat >> 8), uint8_t(heartbeat), 0x6B}); break;
            case 0x68:
                wroteFlash = d[2] != 0;
                heartbeat = unsigned(d[3]) << 24 | unsigned(d[4]) << 16 | unsigned(d[5]) << 8 | d[6];
                reply({0x68, 2, 0x6B}); break;
            case 0x35: reply({0x35, uint8_t(rpm < 0), uint8_t(std::abs(rpm) >> 8), uint8_t(std::abs(rpm)), 0x6B}); break;
            case 0x36:
                if (fault != DropPosition) {
                    if (fault == CanError) {
                        can_frame error{}; error.can_id = CAN_ERR_FLAG | CAN_ERR_ACK; error.can_dlc = 8;
                        (void)::write(fd_, &error, sizeof(error));
                    } else if (fault == BadPacket) reply({0x36, 0, 0x6B});
                    else {
                        const uint32_t value = uint32_t(std::abs(encoder_));
                        reply({0x36, uint8_t(encoder_ < 0), uint8_t(value >> 24), uint8_t(value >> 16),
                               uint8_t(value >> 8), uint8_t(value), 0x6B});
                    }
                }
                break;
            case 0x27: reply({0x27, 2, 0x58, 0x6B}); break;
            case 0xF6:
                rpm = (int(d[2]) * 256 + d[3]) * (d[1] ? -1 : 1);
                commandedRpm = rpm; ++starts;
                reply({0xF6, 2, 0x6B}); break;
            case 0xFD:
                if ((f.can_id & 255) == 0) command.assign(d, d + f.can_dlc);
                else command.insert(command.end(), d + 1, d + f.can_dlc);
                if (command.size() == 12) {
                    ++moves;
                    pulses = uint32_t(command[5]) << 24 | uint32_t(command[6]) << 16 | uint32_t(command[7]) << 8 | command[8];
                    negative = command[1] != 0;
                    queued = command[10] != 0;
                    validMove = command[9] == 1 && command[11] == 0x6B && command[2] == 0 && command[3] == 60;
                    if (fault != DropMoveAck) {
                        if (groupAcks_) ++*groupAcks_;
                        reply({0xFD, 2, 0x6B});
                    }
                }
                break;
            case 0xFE:
                rpm = 0; queued = false; ++stops;
                reply({0xFE, 2, 0x6B}); break;
            default: break;
            }
        }
    }
    int fd_ = -1;
    unsigned node_;
    int64_t encoder_;
    std::atomic<unsigned>* groupAcks_;
    std::mutex eventsMutex_;
    std::vector<unsigned> stopOrder_;
    std::thread thread_;
    std::atomic<bool> quit_{false};
};

motion::Config motionFixture()
{
    // 仅用于vcan协议集成：没有动力学，所有数值都是测试假设。
    motion::Config c;
    c.l1 = 195; c.l2 = 275;
    c.xMinimum = c.yMinimum = -600; c.xMaximum = c.yMaximum = 600;
    c.linkRadius = 8; c.toolRadius = 6; c.minimumSinElbow = .1;
    c.calibrated = true; c.settleSeconds = .06;
    for (std::size_t i = 0; i < c.axes.size(); ++i) {
        auto& axis = c.axes[i];
        axis.node = unsigned(i + 1); axis.direction = 1;
        axis.motorDegPerUnit = i == 0 ? 90 : 180 / motion::Pi;
        axis.zeroMotorDeg = i == 0 ? -4500 : i == 2 ? -90 : 0;
        axis.minimum = i == 0 ? 0 : -3; axis.maximum = i == 0 ? 200 : 3;
        axis.tolerance = i == 0 ? .01 : .003;
        axis.maxRpm = 100; axis.pulsesPerRev = 3200; axis.currentLimitMa = 2000;
    }
    return c;
}

void fourAxisCases()
{
    for (int scenario = 0; scenario < 3; ++scenario) {
        std::atomic<unsigned> acknowledgements{0};
        FakeNode z(1, 0, &acknowledgements), shoulder(2, 0, &acknowledgements),
                 elbow(3, 0, &acknowledgements), wrist(4, 0, &acknowledgements);
        robot::Can can("vcan0");
        robot::Motor m1(can, 1), m2(can, 2), m3(can, 3), m4(can, 4);
        const auto config = motionFixture();
        motion::Motion arm(can, {&m1, &m2, &m3, &m4}, config);
        arm.prepare();
        const auto start = arm.pose();
        auto target = start; target.z += 1;
        if (scenario == 0) {
            const auto begin = std::chrono::steady_clock::now();
            arm.move(target);
            check(std::abs(arm.pose().z - target.z) < .01, "Motion final feedback position");
            check(std::chrono::duration<double>(std::chrono::steady_clock::now() - begin).count() >= config.settleSeconds,
                  "Motion must observe stable arrival over time");
            check(acknowledgements == 4 && z.syncs == 1 && shoulder.syncs == 1 && elbow.syncs == 1 && wrist.syncs == 1,
                  "all four FD ACK before one sync");
            check(!z.syncBeforeAllAcks && !shoulder.syncBeforeAllAcks && !elbow.syncBeforeAllAcks && !wrist.syncBeforeAllAcks,
                  "no node may see early sync");
            check(arm.stop(), "all axes stop normally");
        } else {
            if (scenario == 1) wrist.fault = FakeNode::DropMoveAck;
            // 取消恰好发生在四轴缓存完成后、同步触发前。
            fails([&] { arm.move(target, [&] { return scenario == 2 && acknowledgements == 4; }); });
            check(z.syncs == 0 && shoulder.syncs == 0 && elbow.syncs == 0 && wrist.syncs == 0,
                  "lost ACK/cancellation must not trigger any node");
            check(z.stops >= 2 && shoulder.stops >= 2 && elbow.stops >= 2 && wrist.stops >= 2,
                  "fault must request and confirm stop on every node");
            const auto order = z.stopOrder();
            check(order.size() >= 4 && order[0] == 1 && order[1] == 2 && order[2] == 3 && order[3] == 4,
                  "all stop requests precede the first blocking confirmation");
        }
    }
}
}

void testDevices()
{
    robot::Magnet unopened;
    check(!unopened.on(), "unopened magnet state");
    fails([&] { unopened.set(true); });
    magnetCases();
    const char* testInterface = std::getenv("SCARA_TEST_CAN_INTERFACE");
    if (!testInterface) { std::cout << "devices: vcan cases skipped (no explicit interface)\n"; return; }
    check(std::strcmp(testInterface, "vcan0") == 0, "only isolated vcan0 is allowed");
    {
        const int expectedFd = ::open("/dev/null", O_RDONLY);
        check(expectedFd >= 0, "fd probe"); close(expectedFd);
        robot::Can can(testInterface);
        check((fcntl(expectedFd, F_GETFD) & FD_CLOEXEC) != 0, "CAN close-on-exec");
        const std::string command = "test ! -e /proc/self/fd/" + std::to_string(expectedFd);
        const pid_t child = fork();
        check(child >= 0, "test fork");
        if (child == 0) { execl("/bin/sh", "sh", "-c", command.c_str(), static_cast<char*>(nullptr)); _exit(127); }
        int status = 0;
        check(waitpid(child, &status, 0) == child && WIFEXITED(status) && WEXITSTATUS(status) == 0, "CAN must not survive exec");
    }
    {
        robot::Can sender(testInterface), receiver(testInterface);
        can_frame frame{}, received{};
        frame.can_id = 0x123; frame.can_dlc = 1; frame.data[0] = 0xAA;
        sender.send(frame);
        check(receiver.receive(received, 40) && received.data[0] == 0xAA, "CAN transparent data");
        check(!receiver.receive(received, 5), "CAN timeout");
        frame.can_dlc = 9; fails([&] { sender.send(frame); });
        frame.can_dlc = 1; frame.can_id = 0x800; fails([&] { sender.send(frame); });
    }
    {
        FakeNode fake;
        robot::Can can(testInterface);
        robot::Motor drive(can, 1);
        check(fake.requests == 0, "constructor must not send");
        fails([&] { drive.start(5); });
        drive.prepare();
        check(drive.firmwareVersion() == 200 && !fake.wroteFlash, "prepare version/watchdog");
        const auto feedback = drive.read();
        check(feedback.angleDeg == -90 && feedback.currentMa == 600, "Emm encoder/current units");
        drive.start(-123);
        check(drive.read().rpm == -123 && fake.starts == 1, "speed starts once");
        fails([&] { drive.start(-123); });
        drive.stop();
        drive.move(-720, 60, 1);
        check(fake.validMove && fake.pulses == 6400 && fake.negative && fake.queued, "FD absolute degree to pulses / split packets");
        robot::Motor::synchronize(can);
        (void)drive.read();
        check(fake.syncs == 1 && !fake.queued, "single broadcast sync");
        drive.stop();
        drive.start(5);
        std::this_thread::sleep_for(std::chrono::milliseconds(550));
        check(drive.read().rpm == 0, "emulated watchdog stops");
        drive.stop();
        fails([&] { drive.move(std::numeric_limits<double>::quiet_NaN(), 60, 1); });
        fails([&] { drive.move(1e20, 60, 1); });
    }
    for (auto fault : {FakeNode::WrongMode, FakeNode::WrongPulses}) {
        FakeNode fake; fake.fault = fault;
        robot::Can can(testInterface); robot::Motor drive(can, 1);
        fails([&] { drive.prepare(); });
        check(fake.moves == 0 && fake.starts == 0, "rejected configuration must not move");
    }
    for (auto fault : {FakeNode::DropPosition, FakeNode::DropMoveAck, FakeNode::Stall, FakeNode::BadPacket, FakeNode::CanError}) {
        FakeNode fake;
        robot::Can can(testInterface); robot::Motor drive(can, 1);
        drive.prepare(); fake.fault = fault;
        if (fault == FakeNode::DropMoveAck) fails([&] { drive.move(90, 60, 1); });
        else fails([&] { drive.read(); });
        const int requests = fake.requests;
        fails([&] { drive.read(); });
        check(fake.requests == requests, "locked motor must not feed old motion");
        drive.requestStop(); drive.stop();
        check(fake.stops >= 2 && fake.syncs == 0, "fault stop without sync or restart");
    }
    fourAxisCases();
    std::cout << "devices: protocol, watchdog, fault and four-axis Motion integration cases passed\n";
}
