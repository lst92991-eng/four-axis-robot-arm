#pragma once

#include <chrono>
#include <functional>
#include <string>
#include <vector>

// 同步、单实例使用；取消回调应简短，桌面入口对界面刷新节流。
class Pikafish {
public:
    Pikafish() = default;
    ~Pikafish() { close(); }
    Pikafish(const Pikafish&) = delete;
    Pikafish& operator=(const Pikafish&) = delete;

    bool start(const char* engineDir);
    bool legalMoves(const std::string& moves, std::vector<std::string>& out);
    bool think(const std::string& moves, int timeMs, std::string& best);
    // 从已确认的任意棋局开始，完整历史让引擎保留回合与重复局面上下文。
    bool legalMoves(const std::string& baseFen, const std::string& moves, std::vector<std::string>& out);
    bool think(const std::string& baseFen, const std::string& moves, int timeMs, std::string& best);
    // false表示仍持有未回收子进程，调用者不得开始相机/NPU阶段。
    bool close();
    std::string error;
    std::function<bool()> cancelled;

private:
    using Clock = std::chrono::steady_clock;
    int fd_ = -1;
    int pid_ = -1;
    std::string pending_;
    Clock::time_point deadline_;

    bool fail(const std::string& message);
    bool wait(short events);
    bool send(const std::string& text);
    bool readLine(std::string& line);
    bool position(const std::string& baseFen, const std::string& moves);
};
