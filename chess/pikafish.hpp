#pragma once

#include <chrono>
#include <functional>
#include <string>
#include <vector>

// 同步、单实例使用；搜索期间主线程只响应取消，不处理界面或机械臂事件。
class Pikafish
{
  public:
    Pikafish() = default;
    ~Pikafish()
    {
        close();
    }
    Pikafish(const Pikafish &) = delete;
    Pikafish &operator=(const Pikafish &) = delete;

    bool start(const char *engineDir);
    // 从已确认的任意棋局开始，完整历史让引擎保留回合与重复局面上下文。
    bool legalMoves(const std::string &baseFen, const std::string &moves, std::vector<std::string> &out);
    bool think(const std::string &baseFen, const std::string &moves, int timeMs, std::string &best);
    // false表示仍持有未回收子进程，调用者不得开始相机/NPU阶段。
    bool close();
    std::string error;
    std::function<bool()> cancelled;

  private:
    using Clock = std::chrono::steady_clock;
    int fd_ = -1;                // 主程序这一端的双向管道。
    int pid_ = -1;               // 当前引擎子进程，关闭时等待回收。
    std::string pending_;        // 管道中收到但尚未取走的字符。
    Clock::time_point deadline_; // 本次查询的截止时间，使用单调时钟。

    bool fail(const std::string &message);
    bool wait(short events);
    bool send(const std::string &text);
    bool readLine(std::string &line);
    bool position(const std::string &baseFen, const std::string &moves);
};
