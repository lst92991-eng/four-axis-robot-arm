#pragma once
#include <string>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <thread>

namespace robot {
// 配置指定GPIO芯片和线号；不猜板上接线，不导出GPIO或修改复用。
class Magnet {
public:
    Magnet() = default;
    ~Magnet();
    Magnet(const Magnet&) = delete;
    Magnet& operator=(const Magnet&) = delete;
    void open(const std::string& chip, unsigned line, bool activeHigh, bool initialOn = false);
    void setMaximumOnSeconds(double seconds);
    void set(bool enabled);
    bool on() const;
    bool expired() const;
    std::string failure() const;
private:
    void write(bool enabled); // 持锁调用；命令状态不是吸附传感器。
    void guard();
    int fd_ = -1;
    bool enabled_ = false, activeHigh_ = true, expired_ = false, quit_ = false;
    double maximumOnSeconds_ = 0;
    std::string failure_;
    std::chrono::steady_clock::time_point deadline_;
    mutable std::mutex mutex_;
    std::condition_variable changed_;
    std::thread guard_;
};
} // namespace robot
