#include "pikafish.hpp"

#include <algorithm>
#include <cerrno>
#include <charconv>
#include <csignal>
#include <fcntl.h>
#include <poll.h>
#include <sys/resource.h>
#include <sys/socket.h>
#include <sys/wait.h>
#include <unistd.h>

static bool isMove(const std::string &s)
{
    chess::Move move;
    return chess::parseMove(s, move);
}

bool Pikafish::fail(const std::string &message)
{
    error = message;
    close();
    return false;
}

bool Pikafish::wait(short events)
{
    std::chrono::milliseconds remaining; // 查询截止之前剩余的毫秒数。
    int waitMs, result;
    pollfd pipe;

    while (true)
    {
        if (cancelled && cancelled()) return fail("用户取消");
        if (fd_ < 0) return fail("引擎尚未启动");
        remaining = std::chrono::duration_cast<std::chrono::milliseconds>(deadline_ - Clock::now());
        if (remaining.count() <= 0) return fail("等待引擎超时");
        if (events == POLLIN && pending_.find('\n') != std::string::npos) return true;
        waitMs = 100; // 分段等待，让取消请求也有机会被处理。
        if (remaining.count() < waitMs) waitMs = int(remaining.count());
        pipe.fd = fd_;
        pipe.events = events;
        pipe.revents = 0;
        result = poll(&pipe, 1, waitMs);
        if (result < 0 && errno != EINTR) return fail("引擎管道轮询失败");
        if (result > 0)
        {
            if (pipe.revents & events) return true;
            return fail("引擎已退出，请检查程序目录、NNUE和可用内存");
        }
    }
}

bool Pikafish::send(const std::string &text)
{
    size_t sent = 0;
    ssize_t written; // 本次实际写出的字节数；小于请求量时继续写剩余部分。
    while (sent < text.size())
    {
        if (!wait(POLLOUT)) return false;
        written = ::send(fd_, text.data() + sent, text.size() - sent, MSG_NOSIGNAL);
        if (written > 0)
            sent += size_t(written);
        else if (written == 0 || (errno != EINTR && errno != EAGAIN && errno != EWOULDBLOCK))
            return fail("无法向引擎写入命令");
    }
    return true;
}

bool Pikafish::readLine(std::string &line)
{
    size_t end;
    char buffer[2048];
    ssize_t received; // 这次读取到的字节数，不一定刚好是一行。

    while (true)
    {
        if (!wait(POLLIN)) return false;
        end = pending_.find('\n');
        if (end != std::string::npos)
        {
            line = pending_.substr(0, end);
            pending_.erase(0, end + 1);
            if (!line.empty() && line.back() == '\r') line.pop_back();
            return true;
        }
        received = recv(fd_, buffer, sizeof(buffer), 0);
        if (received > 0)
        {
            if (pending_.size() + size_t(received) > 16384) return fail("引擎输出行过长");
            pending_.append(buffer, size_t(received));
        }
        else if (received == 0 || (errno != EINTR && errno != EAGAIN && errno != EWOULDBLOCK))
        {
            return fail("引擎输出已中断，请检查NNUE和可用内存");
        }
    }
}

bool Pikafish::start(const char *engineDir)
{
    int pair[2]; // 主程序和引擎各持有一端的双向管道。
    int flags, nullfd;
    rlimit memory;
    std::string line;
    bool ready;

    if (!close()) return false;
    error.clear();
    if (cancelled && cancelled())
    {
        error = "用户取消";
        return false;
    }
    if (!engineDir || !*engineDir) return fail("引擎目录为空");
    if (socketpair(AF_UNIX, SOCK_STREAM, 0, pair) < 0) return fail("无法创建引擎管道");
    pid_ = fork();
    if (pid_ == 0)
    {
        ::close(pair[0]);
        // 只限制引擎地址空间，防止板上剩余内存不足时拖垮客户端。
        memory.rlim_cur = 140000UL * 1024;
        memory.rlim_max = 140000UL * 1024;
        if (setrlimit(RLIMIT_AS, &memory) < 0) _exit(126);
        if (chdir(engineDir) < 0) _exit(126);
        if (dup2(pair[1], STDIN_FILENO) < 0) _exit(126);
        if (dup2(pair[1], STDOUT_FILENO) < 0) _exit(126);
        if (pair[1] > STDERR_FILENO) ::close(pair[1]);
        nullfd = open("/dev/null", O_WRONLY);
        if (nullfd < 0 || dup2(nullfd, STDERR_FILENO) < 0) _exit(126);
        if (nullfd > STDERR_FILENO) ::close(nullfd);
        execl("./pikafish", "pikafish", static_cast<char *>(nullptr));
        _exit(127);
    }
    ::close(pair[1]);
    fd_ = pair[0];
    if (pid_ < 0) return fail("无法创建引擎子进程");
    flags = fcntl(fd_, F_GETFL);
    if (flags < 0 || fcntl(fd_, F_SETFL, flags | O_NONBLOCK) < 0) return fail("无法设置非阻塞管道");
    deadline_ = Clock::now() + std::chrono::seconds(5);
    ready = send("uci\n");
    while (ready && line != "uciok")
        ready = readLine(line);
    if (ready) ready = send("setoption name Threads value 1\nsetoption name Hash value 4\nisready\n");
    line.clear();
    while (ready && line != "readyok")
        ready = readLine(line);
    return ready;
}

bool Pikafish::position(const chess::Game &game)
{
    // FEN与历史已经由Game生成；此处只防止破坏UCI的单行命令。
    if (game.baseFen.empty() || game.baseFen.find_first_of("\r\n") != std::string::npos ||
        game.history.find_first_of("\r\n") != std::string::npos)
        return fail("棋局命令为空或包含换行");
    std::string command = "position fen " + game.baseFen;
    if (!game.history.empty()) command += " moves " + game.history;
    return send(command + '\n');
}

bool Pikafish::legalMoves(const chess::Game &game, std::vector<std::string> &out)
{
    out.clear();
    deadline_ = Clock::now() + std::chrono::seconds(3);
    if (!position(game) || !send("go perft 1\n")) return false;
    std::string line;
    while (readLine(line))
    {
        if (line.empty()) continue;
        if (line.compare(0, 16, "Nodes searched: ") == 0)
        {
            unsigned count = 0;
            const char *first = line.data() + 16;
            const char *last = line.data() + line.size();
            std::from_chars_result result;
            result = std::from_chars(first, last, count);
            if (result.ec != std::errc() || result.ptr != last || count != out.size())
                return fail("引擎合法走法计数不一致");
            return true;
        }
        std::string move = line.substr(0, 4);
        if (line.size() != 7 || line.compare(4, 3, ": 1") != 0 || !isMove(move) || out.size() >= 256 ||
            std::find(out.begin(), out.end(), move) != out.end())
            return fail("引擎合法走法格式不合法");
        out.push_back(move);
    }
    return false;
}

bool Pikafish::think(const chess::Game &game, int timeMs, std::string &best)
{
    best.clear();
    if (timeMs < 1 || timeMs > 60000) return fail("思考时间应为1到60000毫秒");
    deadline_ = Clock::now() + std::chrono::milliseconds(timeMs + 2000);
    if (!position(game) || !send("go movetime " + std::to_string(timeMs) + "\n")) return false;
    std::string line;
    while (readLine(line))
    {
        if (line.compare(0, 9, "bestmove ") == 0)
        {
            best = line.substr(9, line.find(' ', 9) - 9);
            if (!isMove(best)) return fail("引擎未返回有效走法");
            return true;
        }
        if (!line.empty() && line.compare(0, 5, "info ") != 0) return fail("引擎搜索输出格式不合法");
    }
    return false;
}

bool Pikafish::close()
{
    if (fd_ >= 0)
    {
        // 清理不检查取消标志；即使Ctrl-C已经生效，也必须回收唯一引擎。
        ::send(fd_, "stop\nquit\n", 10, MSG_NOSIGNAL | MSG_DONTWAIT);
        ::close(fd_);
        fd_ = -1;
    }
    pending_.clear();
    if (pid_ < 0) return true;
    for (int attempt = 0; attempt < 80; ++attempt)
    {
        int result = waitpid(pid_, nullptr, WNOHANG);
        if (result == pid_ || (result < 0 && errno == ECHILD))
        {
            pid_ = -1;
            return true;
        }
        if (attempt == 10) kill(pid_, SIGTERM);
        if (attempt == 30) kill(pid_, SIGKILL);
        poll(nullptr, 0, 10);
    }
    // 保留PID并禁止再次启动，避免未回收时出现两个引擎。
    error += "；引擎未能及时退出";
    return false;
}
