#pragma once
#include "board.hpp"

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

    //初始化：创建进程、连接双向管道并完成UCI握手
    bool start(const char *engineDir);
    // Game由构造/重置和走法确认函数维护；发送它的起始FEN与完整已完成历史。
    bool legalMoves(const chess::Game &game, std::vector<std::string> &out);
    //搜索：发送当前局面和思考时间，输出一个UCI走法
    bool think(const chess::Game &game, int timeMs, std::string &best);
    // false表示仍持有未回收子进程，调用者不得开始相机/NPU阶段。
    bool close();
    //失败原因供主流程显示；取消由主程序的运行标志决定
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
    bool request(const chess::Game &game, const std::string &query);
};
