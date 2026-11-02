#include "magnet.hpp"
#include <fcntl.h>
#include <linux/gpio.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <cstring>
#include <cstdint>
#include <cmath>
#include <stdexcept>

namespace robot {
Magnet::~Magnet()
{
    { std::lock_guard<std::mutex> lock(mutex_); quit_ = true; }
    changed_.notify_all();
    if (guard_.joinable()) guard_.join();
    // 不发关磁命令。释放句柄后的电平依赖驱动；上层须在人工处理完带子状态后退出。
    if (fd_ >= 0) close(fd_);
}

void Magnet::open(const std::string& chip, unsigned line, bool activeHigh, bool initialOn)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (fd_ >= 0) throw std::runtime_error("电磁铁GPIO已打开");
    if (initialOn && maximumOnSeconds_ <= 0) throw std::runtime_error("开磁前须配置通电期限");
    const int device = ::open(chip.c_str(), O_RDONLY | O_CLOEXEC);
    if (device < 0) throw std::runtime_error("无法打开已配置GPIO芯片");
    gpiohandle_request request{};
    request.lineoffsets[0] = line;
    request.flags = GPIOHANDLE_REQUEST_OUTPUT;
    request.default_values[0] = uint8_t(initialOn == activeHigh);
    request.lines = 1;
    std::strcpy(request.consumer_label, "scara-magnet");
    const int result = ioctl(device, GPIO_GET_LINEHANDLE_IOCTL, &request);
    close(device);
    if (result < 0) throw std::runtime_error("申请电磁铁GPIO输出失败");
    fd_ = request.fd;
    activeHigh_ = activeHigh;
    enabled_ = initialOn;
    if (fcntl(fd_, F_SETFD, FD_CLOEXEC) < 0) {
        if (enabled_) { try { write(false); } catch (...) {} }
        close(fd_); fd_ = -1;
        throw std::runtime_error("电磁铁句柄继承设置失败");
    }
    deadline_ = std::chrono::steady_clock::now() +
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(maximumOnSeconds_));
    try { guard_ = std::thread([this] { guard(); }); }
    catch (...) {
        // 计时线程未建立时不能留下无人监督的通电输出。
        if (enabled_) { try { write(false); } catch (...) {} }
        close(fd_); fd_ = -1; throw;
    }
}

void Magnet::setMaximumOnSeconds(double seconds)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (enabled_ || !std::isfinite(seconds) || seconds <= 0 || seconds > 3600)
        throw std::runtime_error("只允许静止时配置有效的电磁铁通电期限");
    maximumOnSeconds_ = seconds;
}

void Magnet::write(bool enabled)
{
    if (fd_ < 0) throw std::runtime_error("电磁铁GPIO未打开");
    gpiohandle_data output{};
    output.values[0] = uint8_t(enabled == activeHigh_);
    // 开磁写入若失败，输出可能已经改变；保守保留“可能通电”状态供上层停机处理。
    enabled_ = enabled_ || enabled;
    if (ioctl(fd_, GPIOHANDLE_SET_LINE_VALUES_IOCTL, &output) < 0) {
        failure_ = "电磁铁GPIO写入失败，通电状态不确定";
        throw std::runtime_error(failure_);
    }
    enabled_ = enabled;
}

void Magnet::set(bool enabled)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (enabled && (expired_ || !failure_.empty() || maximumOnSeconds_ <= 0))
        throw std::runtime_error("电磁铁通电期限未配置或已经故障锁定");
    const bool wasOn = enabled_;
    if (enabled && !wasOn) {
        deadline_ = std::chrono::steady_clock::now() +
            std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(maximumOnSeconds_));
    }
    try { write(enabled); }
    catch (...) { changed_.notify_all(); throw; }
    changed_.notify_all(); // 重复开磁不改变原来的deadline。
}

bool Magnet::on() const { std::lock_guard<std::mutex> lock(mutex_); return enabled_; }
bool Magnet::expired() const { std::lock_guard<std::mutex> lock(mutex_); return expired_; }
std::string Magnet::failure() const { std::lock_guard<std::mutex> lock(mutex_); return failure_; }

void Magnet::guard()
{
    std::unique_lock<std::mutex> lock(mutex_);
    while (!quit_) {
        changed_.wait(lock, [this] { return quit_ || (enabled_ && !expired_); });
        if (quit_) break;
        const auto deadline = deadline_;
        if (changed_.wait_until(lock, deadline, [this, deadline] {
                return quit_ || !enabled_ || deadline_ != deadline;
            })) continue;
        // 独立于取图/引擎/运动停止的等待。普通Linux调度仍可能延迟，不能当硬件急停。
        expired_ = true;
        try { write(false); } catch (...) {} // failure保留，允许操作者显式再尝试关磁。
    }
}
} // namespace robot
