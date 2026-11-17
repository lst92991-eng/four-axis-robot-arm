//使用实测的v3速度/位置指令，按步骤执行一次指定格位搬运
#include "execute.hpp"
#include "candlelight.hpp"
#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <thread>

#ifdef _WIN32
#include <windows.h>
#else
#include <fcntl.h>
#include <spawn.h>
#include <sys/wait.h>
#include <unistd.h>
extern char **environ;
#endif

namespace scara
{
namespace
{
using Clock = std::chrono::steady_clock;
using namespace std::chrono_literals;
enum Command : uint8_t { Zero = 2, Move = 3, Stop = 4, Off = 5, Clear = 6, Ping = 7, Velocity = 12 };
constexpr unsigned Referenced = 2, Interlocked = 8, Configured = 16, Encoder = 32, Power = 64, Stationary = 256;
constexpr unsigned Ready = Referenced | Configured | Encoder;
constexpr uint32_t Builds[] = {0xd3b08bf8, 0x41f870bf, 0x41f870bf};
constexpr char Gpio[] = "/sys/class/gpio/gpio40";

struct Status
{
    //上电毫秒、编码器计数、v3运行标志和固件构建号
    Clock::time_point received{};
    uint32_t uptime = 0, counts = 0, build = 0;
    unsigned state = 0, fault = 0, flags = 0;
    //位置和本段参考用驱动板原始角度，速度用度/秒
    double position = 0, target = 0, velocity = 0;
};
struct Execution
{
    const PickSetup &setup;
    const volatile std::sig_atomic_t &running;
    std::array<Status, 3> axes{};
    std::array<uint32_t, 3> sequence{};
    Clock::time_point nextPing{};
    int active = -1;
    bool keepAlive = false, restarting = false;
    int rejectedPing = -1;
    std::array<uint32_t, 3> pingSequence{};
};
//GPIO脉冲在RV1106上计时，主机等待期间仍收CAN反馈
struct GpioProcess
{
#ifdef _WIN32
    HANDLE process = nullptr, output = nullptr;
#else
    pid_t process = -1;
    int output = -1;
#endif
    void start(const std::string &serial, const std::string &script);
    bool finished(std::string &text);
    ~GpioProcess();
};

bool stopped(const Status &status);
void supervise(Execution &x, bool allowLock = false);
bool receive(Execution &x, CanFrame &frame, unsigned timeoutMs);
bool command(Execution &x, int axis, Command kind, int32_t a = 0, int32_t b = 0);
void enableSession(Execution &x);
void moveZ(Execution &x, double goal);
void moveXY(Execution &x, const JointTarget &goal);
void requireXY(const Execution &x, const JointTarget &goal);
bool stopActive(Execution &x) noexcept;
std::string gpio(Execution &x, int pulseMs, bool inspect = false, bool cleanup = false);

//---------初始化与取放区---------
void checkPlan(const std::vector<PickStep> &steps, const PickSetup &setup)
{
    //首测范围沿用旧机旋转轴工作区，升降使用示教上下层和原速度上限
    const auto &z = setup.heights;
    if (!std::isfinite(setup.zDegreesPerMm) || setup.zDegreesPerMm <= 0 ||
        !std::isfinite(setup.zSpeed) || setup.zSpeed < 1 || setup.zSpeed > 180 ||
        std::max(std::abs(z.travelDeg), std::abs(z.pickupDeg)) > 1999900)
        throw std::runtime_error("示教高度或J1试走速度无效");
    int pulses = 0;
    for (const PickStep &step : steps)
    {
        if (step.action == PickAction::Move &&
            (!std::isfinite(step.target[0]) || step.target[0] < std::min(z.travelDeg, z.pickupDeg) ||
             step.target[0] > std::max(z.travelDeg, z.pickupDeg) ||
             !(std::abs(step.target[1]) <= 180 && std::abs(step.target[2]) <= 180)))
            throw std::runtime_error("目标超出已示教的Z高度或旋转轴试走范围");
        if (step.action == PickAction::ReleasePulse)
        {
            if (step.durationMs < 1 || step.durationMs > 1000)
                throw std::runtime_error("释放脉冲必须在1～1000ms范围");
            ++pulses;
        }
    }
    if (pulses != 1) throw std::runtime_error("单次取放需要一个释放脉冲");
}
} // namespace

void Motion_pick(const std::vector<PickStep> &steps, const PickSetup &setup, bool run,
                 const volatile std::sig_atomic_t &running)
{
    checkPlan(steps, setup);
    if (setup.adbSerial.empty() || setup.adbSerial.find_first_not_of(
            "0123456789abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ:-_.") != std::string::npos)
        throw std::runtime_error("ADB序列号无效");
    if (!running) throw std::runtime_error("取放已取消");
    Execution x{setup, running};
    Candle_open(setup.canSerial);
    bool magnetTouched = false;
    try
    {
        //GPIO先核对方向、极性和被动吸持电平，仅凭输出回读确认软件接线
        if (gpio(x, 0, true) != "out 0 0")
            throw std::runtime_error("GPIO40需要out、active_low=0以及吸持电平0");
        CanFrame frame;
        auto deadline = Clock::now() + 1s;
        while (running && Clock::now() < deadline &&
               std::any_of(x.axes.begin(), x.axes.end(), [](const Status &s) { return s.received == Clock::time_point{}; }))
            receive(x, frame, 20);
        supervise(x, true);
        for (int axis = 0; axis < 3; ++axis)
        {
            const Status &s = x.axes[axis];
            if (!stopped(s)) throw std::runtime_error("J" + std::to_string(axis + 1) + "尚未停稳");
            std::cout << "J" << axis + 1 << "  " << std::hex << s.build << std::dec << "  "
                      << s.position << "°  " << (s.flags & Interlocked ? "通信锁定" : "停稳") << '\n';
        }
        const auto &z = setup.heights;
        const double up = z.travelDeg > z.pickupDeg ? 1 : -1;
        const double tolerance = std::max(.10, setup.zDegreesPerMm * .12);
        if ((x.axes[0].position - z.travelDeg) * up < -tolerance)
            throw std::runtime_error("J1需要先由旧工具抬到本次示教的搬运高度");
        if (run)
        {
            enableSession(x);
            JointTarget xy{x.axes[1].position, x.axes[2].position};
            for (const PickStep &step : steps)
            {
                supervise(x);
                if (step.action == PickAction::Hold)
                {
                    magnetTouched = true;
                    gpio(x, 0);
                }
                else if (step.action == PickAction::ReleasePulse)
                {
                    requireXY(x, xy);
                    if (std::abs(x.axes[0].position - z.releaseDeg) > tolerance)
                        throw std::runtime_error("J1尚未到释放高度");
                    gpio(x, step.durationMs);
                    std::cout << "GPIO40释放后已回到0\n" << std::flush;
                }
                else
                {
                    //只有平面目标变化时才运行J2/J3，下降前检查吸头落点
                    JointTarget goal{step.target[1], step.target[2]};
                    if (goal.j2Deg != xy.j2Deg || goal.j3MotorDeg != xy.j3MotorDeg)
                    {
                        if ((x.axes[0].position - z.travelDeg) * up < -tolerance)
                            throw std::runtime_error("J1未抬到搬运高度");
                        moveXY(x, goal);
                    }
                    xy = goal;
                    if ((step.target[0] - z.travelDeg) * up < 0) requireXY(x, xy);
                    moveZ(x, step.target[0]);
                    std::cout << "停稳反馈(°)  " << x.axes[0].position << ' ' << x.axes[1].position << ' '
                              << x.axes[2].position << '\n' << std::flush;
                }
            }
            supervise(x);
            std::cout << "取放动作完成，请查看棋子是否落在目标格。\n";
        }
        else std::cout << "已读取三轴状态及GPIO40，等待现场执行取放。\n";
    }
    catch (const std::exception &error)
    {
        //中断后停止当前轴；失联时让驱动自身的500ms看门狗接管
        bool confirmed = stopActive(x);
        bool low = true;
        if (magnetTouched)
        {
            try { gpio(x, 0, false, true); }
            catch (...) { low = false; }
        }
        Candle_close();
        if (!confirmed || !low)
            throw std::runtime_error(std::string(error.what()) + (confirmed ? "" : "；未确认电机停稳") +
                                     (low ? "" : "；GPIO40低电平未确认"));
        throw;
    }
    Candle_close();
}

namespace
{
//---------CAN状态与命令区---------
bool stopped(const Status &s)
{
    return (s.flags & Stationary) && (s.state == 0 ? !(s.flags & Power) : s.state == 1 && (s.flags & Power));
}

bool receive(Execution &x, CanFrame &frame, unsigned timeoutMs)
{
    if (!Candle_receive(frame, timeoutMs)) return false;
    const auto &data = frame.data;
    if (frame.id >= 0x481 && frame.id <= 0x483 && x.keepAlive)
    {
        unsigned axis = frame.id - 0x481;
        if (data[0] == (Ping | 0x80) && data[1] == axis + 1 && data[3] == 3 && data[2] &&
            x.pingSequence[axis] && readWord(data.data() + 4) == x.pingSequence[axis])
            x.rejectedPing = int(axis);
    }
    if (frame.id < 0x201 || frame.id > 0x203) return true;
    unsigned node = frame.id - 0x200;
    if (data[0] != 0x40 || data[1] != node || data[2] != 0 || data[3] != 3) return true;
    Status &s = x.axes[node - 1];
    auto value = [&](int index) { return readWord(data.data() + 8 + index * 4); };
    uint32_t uptime = value(0);
    if (s.received != Clock::time_point{} && uint32_t(uptime - s.uptime) >= 0x80000000)
        x.restarting = true;
    if (s.received != Clock::time_point{} && uptime == s.uptime) return true;
    uint32_t state = value(9);
    s = {Clock::now(), uptime, value(5), value(10), state & 255, (state >> 8) & 255, state >> 16,
         int32_t(value(1)) / 1000.0, int32_t(value(2)) / 1000.0, int32_t(value(4)) / 1000.0};
    return true;
}

uint32_t send(Execution &x, int axis, Command kind, int32_t a = 0, int32_t b = 0)
{
    auto seq = ++x.sequence[axis];
    CanFrame frame;
    unsigned node = axis + 1;
    frame.id = 0x400 + node;
    frame.data[0] = kind;
    frame.data[1] = uint8_t(node);
    frame.data[3] = 3;
    writeWord(frame.data.data() + 4, seq);
    writeWord(frame.data.data() + 8, uint32_t(a));
    writeWord(frame.data.data() + 12, uint32_t(b));
    Candle_send(frame);
    return seq;
}

bool command(Execution &x, int axis, Command kind, int32_t a, int32_t b)
{
    //只接受同一节点、序号及命令的应答；ACK和实际到位分别确认
    uint32_t seq = send(x, axis, kind, a, b);
    auto deadline = Clock::now() + 250ms;
    CanFrame frame;
    while ((x.running || kind == Stop) && Clock::now() < deadline)
        if (receive(x, frame, 15) && frame.id == unsigned(0x481 + axis) &&
            frame.data[0] == (kind | 0x80) && frame.data[1] == axis + 1 && frame.data[3] == 3 &&
            readWord(frame.data.data() + 4) == seq)
            return frame.data[2] == 0;
    throw std::runtime_error("J" + std::to_string(axis + 1) + "命令应答超时或取消");
}

void supervise(Execution &x, bool allowLock)
{
    if (!x.running) throw std::runtime_error("取放已取消");
    if (x.restarting) throw std::runtime_error("驱动板重启，需核对机械参考");
    if (x.rejectedPing >= 0) throw std::runtime_error("J" + std::to_string(x.rejectedPing + 1) + "拒绝CAN保活");
    auto now = Clock::now();
    for (int axis = 0; axis < 3; ++axis)
    {
        const Status &s = x.axes[axis];
        std::string name = "J" + std::to_string(axis + 1);
        if (now - s.received > 250ms) throw std::runtime_error(name + "反馈过期或未收到");
        if (s.build != Builds[axis])
        {
            std::ostringstream info;
            info << name << "实际固件" << std::hex << s.build << "，已验证基准" << Builds[axis];
            throw std::runtime_error(info.str());
        }
        if (s.fault || s.state == 4) throw std::runtime_error(name + "驱动故障 " + std::to_string(s.fault));
        if ((s.flags & Ready) != Ready || (!allowLock && (s.flags & Interlocked)))
            throw std::runtime_error(name + "机械参考、编码器或通信锁定异常");
    }
    const auto &z = x.setup.heights;
    double margin = std::max(.10, .12 * x.setup.zDegreesPerMm);
    double direction = z.travelDeg > z.pickupDeg ? 1 : -1;
    double height = (x.axes[0].position - z.pickupDeg) * direction;
    if (height < -margin || height > std::abs(z.travelDeg - z.pickupDeg) + x.setup.zDegreesPerMm ||
        std::abs(x.axes[1].position) > 180.12 || std::abs(x.axes[2].position) > 180.12)
        throw std::runtime_error("实际反馈超出取放工作范围");
    if (x.keepAlive && now >= x.nextPing)
    {
        for (int axis = 0; axis < 3; ++axis) x.pingSequence[axis] = send(x, axis, Ping);
        x.nextPing = now + 100ms;
    }
}

template<class Condition>
void waitFor(Execution &x, double seconds, const char *reason, Condition done, bool allowLock = false)
{
    auto deadline = Clock::now() + std::chrono::duration<double>(seconds);
    CanFrame frame;
    while (Clock::now() < deadline)
    {
        supervise(x, allowLock);
        if (done()) return;
        receive(x, frame, 20);
    }
    throw std::runtime_error(reason);
}

void enableSession(Execution &x)
{
    //用PING对齐每块板的序号，保持旧面板的四次探测范围
    for (int axis = 0; axis < 3; ++axis)
    {
        int attempt = 0;
        while (!command(x, axis, Ping))
        {
            if (++attempt == 4) throw std::runtime_error("无法同步CAN序号");
            x.sequence[axis] += 0x3fffffff;
        }
    }
    x.keepAlive = true;

    //旧固件换主机可能已进入通信锁定。仅恢复停稳且无故障的原参考。
    for (int axis = 0; axis < 3; ++axis)
    {
        supervise(x, true);
        auto before = x.axes[axis];
        if (!(before.flags & Interlocked)) continue;
        if (!stopped(before)) throw std::runtime_error("通信锁定后的轴尚未停稳");
        if (axis == 0)
        {
            //旧J1先停出力，再接续刚读取的多圈角度，避免把当前高度重置成0
            if (before.state == 1 && !command(x, axis, Off)) throw std::runtime_error("J1未能退出保持");
            waitFor(x, 1, "J1未确认停止出力", [&] {
                const auto &s = x.axes[0];
                return s.uptime != before.uptime && stopped(s) && s.state == 0;
            }, true);
            int32_t delta = int32_t((x.axes[0].counts - before.counts + 8192) & 16383) - 8192;
            if (std::abs(delta) > 46 || std::abs(x.axes[0].position - before.position) > 1.01)
                throw std::runtime_error("J1退出保持时高度变化，需重新确认参考");
            if (!command(x, axis, Zero, int32_t(std::lround(x.axes[0].position * 1000))))
                throw std::runtime_error("J1多圈参考接续未确认");
        }
        else if (!command(x, axis, Clear)) throw std::runtime_error("旋转轴通信锁定未解除");
        waitFor(x, 1, "未收到恢复后的停稳反馈", [&] {
            const auto &s = x.axes[axis];
            return s.uptime != before.uptime && stopped(s) && !(s.flags & Interlocked);
        }, true);
    }
    supervise(x);
}

//---------状态确认与制动区---------
void startAxis(Execution &x, int axis, Command kind, int32_t a, int32_t b = 0)
{
    //确认未在移动；在发送之前就记住轴号，覆盖应答丢失但动作已启动的情况
    uint32_t seen = x.axes[axis].uptime;
    waitFor(x, 2, "驱动未进入可运动的停稳状态",
            [&] { return stopped(x.axes[axis]) && x.axes[axis].uptime != seen; });
    x.active = axis;
    if (!command(x, axis, kind, a, b))
        throw std::runtime_error("J" + std::to_string(axis + 1) + "拒绝运动指令");
}

void brake(Execution &x)
{
    int axis = x.active;
    if (!command(x, axis, Stop)) throw std::runtime_error("驱动拒绝减速命令");
    uint32_t seen = x.axes[axis].uptime;
    waitFor(x, 4, "减速后未确认停稳",
            [&] { return x.axes[axis].uptime != seen && stopped(x.axes[axis]); });
    x.active = -1;
}

bool stopActive(Execution &x) noexcept
{
    x.keepAlive = false;
    if (x.active < 0) return true;
    //取消后也尝试一次STOP，之后读新停稳反馈，期间不再续运动看门狗
    try
    {
        command(x, x.active, Stop);
        uint32_t seen = x.axes[x.active].uptime;
        auto deadline = Clock::now() + 4s;
        CanFrame frame;
        while (Clock::now() < deadline)
        {
            receive(x, frame, 20);
            const auto &s = x.axes[x.active];
            if (s.uptime != seen && stopped(s) && (s.flags & Ready) == Ready && !x.restarting) return true;
        }
    }
    catch (...) {}
    return false;
}

//---------升降与平面运动区---------
void moveZ(Execution &x, double goal)
{
    const auto &z = x.setup.heights;
    double perMm = x.setup.zDegreesPerMm;
    double up = z.travelDeg > z.pickupDeg ? 1 : -1;
    double tolerance = std::max(.10, .12 * perMm);
    auto atHeight = [&] {
        double error = goal - x.axes[0].position;
        return std::abs(error) <= tolerance || (goal == z.travelDeg && error * up <= 0);
    };
    for (int attempt = 0; attempt < 3 && !atHeight(); ++attempt)
    {
        if (!attempt && x.axes[0].state == 1)
        {
            //同旧工具，Z换向前退出保持，去掉上段的保持电流
            if (!command(x, 0, Off)) throw std::runtime_error("J1退出保持被拒绝");
            waitFor(x, 1, "J1未确认停止出力", [&] { return x.axes[0].state == 0 && stopped(x.axes[0]); });
        }
        double delta = goal - x.axes[0].position, sign = delta > 0 ? 1 : -1;
        if (atHeight()) return;
        double brakeGoal = goal;
        if (!attempt && delta * up < -2 * perMm) brakeGoal += up * .8 * perMm;
        double speed = std::min(attempt ? std::min(25., x.setup.zSpeed) : x.setup.zSpeed,
                                std::max(1., std::abs(delta) * 1.5));
        startAxis(x, 0, Velocity, int32_t(std::lround(sign * speed * 1000)));
        auto start = Clock::now();
        waitFor(x, std::abs(delta) / speed + speed / 60 + 4, "J1接近高度超时", [&] {
            const auto &s = x.axes[0];
            //加速度60、制动360度/秒²，按板端参考角提前制动
            double elapsed = std::chrono::duration<double>(Clock::now() - start).count();
            double v = std::min(speed, std::max({0., sign * s.velocity, elapsed * 60}));
            return s.state == 5 && sign * (brakeGoal - s.target) <= v * v / 720 + .033 * v;
        });
        brake(x);
    }
    if (!atHeight()) throw std::runtime_error("J1未停到指定高度");
}

double xyError(const Execution &x, const JointTarget &goal)
{
    cv::Point2d expected, actual;
    Arm_forward(x.setup.arm, goal, expected);
    Arm_forward(x.setup.arm, {x.axes[1].position, x.axes[2].position}, actual);
    return cv::norm(expected - actual);
}

void requireXY(const Execution &x, const JointTarget &goal)
{
    if (xyError(x, goal) > 2.5 || !stopped(x.axes[1]) || !stopped(x.axes[2]))
        throw std::runtime_error("吸头未在目标点2.5mm范围内停稳");
}

void moveXY(Execution &x, const JointTarget &goal)
{
    //J2用位置模式，先确认目标和反馈都到位
    const double distance = std::abs(goal.j2Deg - x.axes[1].position);
    if (distance > .10 || x.axes[1].state != 1)
    {
        startAxis(x, 1, Move, int32_t(std::lround(goal.j2Deg * 1000)), 8000);
        waitFor(x, distance / 8 + 16. / 30 + 5, "J2到位超时", [&] {
            const auto &s = x.axes[1];
            return (s.state == 1 || s.state == 2) && (s.flags & Stationary) &&
                   std::abs(s.target - goal.j2Deg) <= .002 && std::abs(s.position - goal.j2Deg) <= .12;
        });
        brake(x);
        //同步带在J2制动后可能带动肘轴，等250ms再取J3起点
        auto end = Clock::now() + 250ms;
        waitFor(x, .6, "J2停稳反馈超时", [&] { return Clock::now() >= end; });
    }
    //J3沿用先接近再微调，最后用实际两轴角度正解校核吸头位置
    for (int attempt = 0; attempt < 3; ++attempt)
    {
        double initial = x.axes[2].position, delta = goal.j3MotorDeg - initial;
        if (!attempt && std::abs(delta) <= 9) continue;
        if (attempt && (xyError(x, goal) <= 2.5 || std::abs(delta) < .10)) break;
        double sign = delta > 0 ? 1 : -1;
        double speed = !attempt ? 30 : std::clamp(1.5 * std::abs(delta), 3., 8.);
        startAxis(x, 2, Velocity, int32_t(std::lround(sign * speed * 1000)));
        waitFor(x, std::abs(delta) / (!attempt ? 5 : speed) + 4, "J3接近目标超时", [&] {
            const auto &s = x.axes[2];
            if (sign * (s.position - initial) < -5) throw std::runtime_error("J3反馈远离目标");
            double v = std::max(0., sign * s.velocity);
            double lead = !attempt ? std::max(5., .55 * v) : v * v / 60 + .05 * v;
            return s.state == 5 && sign * (goal.j3MotorDeg - s.position) <= lead;
        });
        brake(x);
    }
    requireXY(x, goal);
}

//---------吸放输出区---------
std::string gpio(Execution &x, int pulseMs, bool inspect, bool cleanup)
{
    std::string path = std::string(Gpio) + "/value";
    std::string script;
    if (inspect) script = "cat " + std::string(Gpio) + "/direction " + Gpio + "/active_low " + path;
    else if (!pulseMs) script = "echo 0 > " + path + " && cat " + path;
    else
    {
        //RV1106端按时归零，ADB退出或中断时也运行归零处理
        std::string seconds = std::to_string(pulseMs / 1000.0);
        script = "set -e; trap 'echo 0 > " + path + "' EXIT; trap 'exit 1' HUP INT TERM; "
                 "echo 1 > " + path + "; cat " + path + "; busybox sleep " + seconds +
                 "; echo 0 > " + path + "; cat " + path;
    }
    GpioProcess child;
    child.start(x.setup.adbSerial, script);
    auto deadline = Clock::now() + 2s + std::chrono::milliseconds(pulseMs);
    std::string output;
    CanFrame frame;
    while (!child.finished(output))
    {
        if (Clock::now() >= deadline || (!cleanup && !x.running))
            throw std::runtime_error("GPIO40操作未完成或已取消");
        if (x.keepAlive) supervise(x);
        if (cleanup) std::this_thread::sleep_for(10ms);
        else receive(x, frame, 15);
    }
    //只使用子进程真实回读的值；释放完成必须看到1、0两个电平
    std::istringstream words(output);
    std::string value, result;
    while (words >> value)
    {
        if (!result.empty()) result += ' ';
        result += value;
    }
    if (!inspect && result != (pulseMs ? "1 0" : "0")) throw std::runtime_error("GPIO40回读不符");
    return result;
}

//---------GPIO子进程区---------
#ifdef _WIN32
void GpioProcess::start(const std::string &serial, const std::string &script)
{
    if (serial == "local") throw std::runtime_error("Windows请通过ADB选择RV1106");
    SECURITY_ATTRIBUTES security{sizeof(security), nullptr, TRUE};
    HANDLE writer = nullptr;
    if (!CreatePipe(&output, &writer, &security, 0)) throw std::runtime_error("无法读取ADB输出");
    STARTUPINFOA info{};
    info.cb = sizeof(info);
    info.dwFlags = STARTF_USESTDHANDLES;
    info.hStdOutput = writer;
    info.hStdError = GetStdHandle(STD_ERROR_HANDLE);
    info.hStdInput = GetStdHandle(STD_INPUT_HANDLE);
    PROCESS_INFORMATION child{};
    std::string commandLine = "adb.exe ";
    if (serial != "-") commandLine += "-s " + serial + " ";
    commandLine += "shell \"" + script + "\"";
    BOOL started = SetHandleInformation(output, HANDLE_FLAG_INHERIT, 0) &&
                   CreateProcessA(nullptr, commandLine.data(), nullptr, nullptr, TRUE, CREATE_NO_WINDOW,
                                  nullptr, nullptr, &info, &child);
    CloseHandle(writer);
    if (!started) throw std::runtime_error("无法启动adb.exe，请检查PATH");
    process = child.hProcess;
    CloseHandle(child.hThread);
}
bool GpioProcess::finished(std::string &text)
{
    if (WaitForSingleObject(process, 0) == WAIT_TIMEOUT) return false;
    DWORD code, available = 0, length = 0;
    if (!GetExitCodeProcess(process, &code) || code != 0) throw std::runtime_error("ADB GPIO命令失败");
    char buffer[128];
    if (PeekNamedPipe(output, nullptr, 0, nullptr, &available, nullptr) && available &&
        ReadFile(output, buffer, std::min<DWORD>(sizeof(buffer), available), &length, nullptr))
        text.assign(buffer, length);
    CloseHandle(process);
    process = nullptr;
    return true;
}
GpioProcess::~GpioProcess()
{
    if (process)
    {
        TerminateProcess(process, 1);
        CloseHandle(process);
    }
    if (output) CloseHandle(output);
}
#else
void GpioProcess::start(const std::string &serial, const std::string &script)
{
    int pipeEnds[2];
    if (pipe2(pipeEnds, O_CLOEXEC | O_NONBLOCK) != 0) throw std::runtime_error("无法读取GPIO输出");
    output = pipeEnds[0];
    posix_spawn_file_actions_t actions;
    int error = posix_spawn_file_actions_init(&actions);
    if (!error)
    {
        error = posix_spawn_file_actions_adddup2(&actions, pipeEnds[1], STDOUT_FILENO);
        std::vector<std::string> args;
        if (serial == "local") args = {"/bin/sh", "-c", script};
        else if (serial == "-") args = {"adb", "shell", script};
        else args = {"adb", "-s", serial, "shell", script};
        std::vector<char *> argv;
        for (auto &arg : args) argv.push_back(arg.data());
        argv.push_back(nullptr);
        if (!error) error = posix_spawnp(&process, argv[0], &actions, nullptr, argv.data(), environ);
        posix_spawn_file_actions_destroy(&actions);
    }
    close(pipeEnds[1]);
    if (error) throw std::runtime_error("无法启动GPIO命令，请检查ADB或板端shell");
}
bool GpioProcess::finished(std::string &text)
{
    int status = 0;
    pid_t result = waitpid(process, &status, WNOHANG);
    if (!result || (result < 0 && errno == EINTR)) return false;
    if (result < 0) throw std::runtime_error("读取GPIO命令状态失败");
    process = -1;
    if (!WIFEXITED(status) || WEXITSTATUS(status)) throw std::runtime_error("GPIO命令运行失败");
    char buffer[128];
    ssize_t length;
    while ((length = read(output, buffer, sizeof(buffer))) > 0) text.append(buffer, length);
    return true;
}
GpioProcess::~GpioProcess()
{
    if (process > 0)
    {
        kill(process, SIGKILL);
        while (waitpid(process, nullptr, 0) < 0 && errno == EINTR) {}
    }
    if (output >= 0) close(output);
}
#endif
}
}
