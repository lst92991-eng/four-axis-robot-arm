#include "pikafish.hpp"
#include "board.hpp"

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

static bool isMove(const std::string& s) {
    chess::Move move;
    return chess::parseMove(s, move);
}

bool Pikafish::fail(const std::string& message) {
    error = message;
    close();
    return false;
}

bool Pikafish::wait(short events) {
    while (1) {
        if (fd_ < 0) return fail("引擎尚未启动");
        // 已收到的整行立即读取；不为每一行重复绘制GUI，耗尽等待预算。
        if (events == POLLIN && pending_.find('\n') != std::string::npos) return true;
        if (cancelled && cancelled()) return fail("用户取消");
        auto left = std::chrono::duration_cast<std::chrono::milliseconds>(deadline_ - Clock::now()).count();
        if (left <= 0) return fail("等待引擎超时");
        pollfd p{fd_, events, 0};
        int result = poll(&p, 1, static_cast<int>(std::min<decltype(left)>(left, 100)));
        if (result < 0 && errno != EINTR) return fail("引擎管道轮询失败");
        if (result > 0) {
            if (p.revents & events) return true;
            return fail("引擎已退出，请检查程序目录、NNUE和可用内存");
        }
    }
}

bool Pikafish::send(const std::string& text) {
    size_t sent = 0;
    while (sent < text.size()) {
        if (!wait(POLLOUT)) return false;
        ssize_t n = ::send(fd_, text.data() + sent, text.size() - sent, MSG_NOSIGNAL);
        if (n > 0) sent += static_cast<size_t>(n);
        else if (n == 0 || (errno != EINTR && errno != EAGAIN && errno != EWOULDBLOCK))
            return fail("无法向引擎写入命令");
    }
    return true;
}

bool Pikafish::readLine(std::string& line) {
    while (1) {
        if (!wait(POLLIN)) return false;
        size_t end = pending_.find('\n');
        if (end != std::string::npos) {
            line = pending_.substr(0, end);
            pending_.erase(0, end + 1);
            if (!line.empty() && line.back() == '\r') line.pop_back();
            return true;
        }
        char buffer[2048];
        ssize_t n = recv(fd_, buffer, sizeof(buffer), 0);
        if (n > 0) {
            if (pending_.size() + static_cast<size_t>(n) > 16384) return fail("引擎输出行过长");
            pending_.append(buffer, static_cast<size_t>(n));
        } else if (n == 0 || (errno != EINTR && errno != EAGAIN && errno != EWOULDBLOCK)) {
            return fail("引擎输出已中断，请检查NNUE和可用内存");
        }
    }
}

bool Pikafish::start(const char* engineDir) {
    if (!close()) return false;
    error.clear();
    if (cancelled && cancelled()) { error = "用户取消"; return false; }
    if (!engineDir || !*engineDir) return fail("引擎目录为空");
    int pair[2];
    if (socketpair(AF_UNIX, SOCK_STREAM, 0, pair) < 0) return fail("无法创建引擎管道");
    pid_ = fork();
    if (pid_ == 0) {
        ::close(pair[0]);
        // 只限制引擎地址空间，防止板上剩余内存不足时拖垮客户端。
        rlimit memory{140000UL * 1024, 140000UL * 1024};
        if (setrlimit(RLIMIT_AS, &memory) < 0 || chdir(engineDir) < 0 ||
            dup2(pair[1], STDIN_FILENO) < 0 || dup2(pair[1], STDOUT_FILENO) < 0) _exit(126);
        if (pair[1] > STDERR_FILENO) ::close(pair[1]);
        int nullfd = open("/dev/null", O_WRONLY);
        if (nullfd < 0 || dup2(nullfd, STDERR_FILENO) < 0) _exit(126);
        if (nullfd > STDERR_FILENO) ::close(nullfd);
        execl("./pikafish", "pikafish", static_cast<char*>(nullptr));
        _exit(127);
    }
    ::close(pair[1]);
    fd_ = pair[0];
    if (pid_ < 0) return fail("无法创建引擎子进程");
    int flags = fcntl(fd_, F_GETFL);
    if (flags < 0 || fcntl(fd_, F_SETFL, flags | O_NONBLOCK) < 0) return fail("无法设置非阻塞管道");
    deadline_ = Clock::now() + std::chrono::seconds(5);
    std::string line;
    if (!send("uci\n")) return false;
    do {
        if (!readLine(line)) return false;
    } while (line != "uciok");
    if (!send("setoption name Threads value 1\nsetoption name Hash value 4\nisready\n")) return false;
    do {
        if (!readLine(line)) return false;
    } while (line != "readyok");
    return true;
}

bool Pikafish::position(const std::string& baseFen, const std::string& moves) {
    if (!baseFen.empty()) {
        chess::Board board;
        bool redToMove;
        std::string detail;
        if (baseFen.find_first_of("\r\n") != std::string::npos) return fail("基准FEN不能包含换行");
        if (!chess::parseFen(baseFen, board, redToMove, detail)) return fail("基准FEN不合法：" + detail);
    }
    // 历史只能包含已经确认合法的四字符走法，最多1024个半回合。
    if (!moves.empty()) {
        if (moves.size() > 5119 || (moves.size() + 1) % 5 != 0) return fail("走棋历史长度不合法");
        for (size_t i = 0; i < moves.size(); i += 5) {
            if (!isMove(moves.substr(i, 4)) || (i + 4 < moves.size() && moves[i + 4] != ' '))
                return fail("走棋历史格式不合法");
        }
    }
    return send((baseFen.empty() ? std::string("position startpos") : "position fen " + baseFen) +
                (moves.empty() ? std::string() : " moves " + moves) + "\n");
}

bool Pikafish::legalMoves(const std::string& moves, std::vector<std::string>& out) {
    return legalMoves({}, moves, out);
}

bool Pikafish::legalMoves(const std::string& baseFen, const std::string& moves, std::vector<std::string>& out) {
    out.clear();
    deadline_ = Clock::now() + std::chrono::seconds(3);
    if (!position(baseFen, moves) || !send("go perft 1\n")) return false;
    std::string line;
    while (readLine(line)) {
        if (line.empty()) continue;
        if (line.compare(0, 16, "Nodes searched: ") == 0) {
            unsigned count = 0;
            auto result = std::from_chars(line.data() + 16, line.data() + line.size(), count);
            if (result.ec != std::errc() || result.ptr != line.data() + line.size() || count != out.size())
                return fail("引擎合法走法计数不一致");
            return true;
        }
        std::string move = line.substr(0, 4);
        if (line.size() != 7 || line.compare(4, 3, ": 1") != 0 || !isMove(move) || out.size() >= 256 ||
            std::find(out.begin(), out.end(), move) != out.end()) return fail("引擎合法走法格式不合法");
        out.push_back(move);
    }
    return false;
}

bool Pikafish::think(const std::string& moves, int timeMs, std::string& best) {
    return think({}, moves, timeMs, best);
}

bool Pikafish::think(const std::string& baseFen, const std::string& moves, int timeMs, std::string& best) {
    best.clear();
    if (timeMs < 1 || timeMs > 60000) return fail("思考时间应为1到60000毫秒");
    deadline_ = Clock::now() + std::chrono::milliseconds(timeMs + 2000);
    if (!position(baseFen, moves) || !send("go movetime " + std::to_string(timeMs) + "\n")) return false;
    std::string line;
    while (readLine(line)) {
        if (line.compare(0, 9, "bestmove ") == 0) {
            best = line.substr(9, line.find(' ', 9) - 9);
            return isMove(best) ? true : fail("引擎未返回有效走法");
        }
        if (!line.empty() && line.compare(0, 5, "info ") != 0) return fail("引擎搜索输出格式不合法");
    }
    return false;
}

bool Pikafish::close() {
    if (fd_ >= 0) {
        // 清理不检查取消标志；即使Ctrl-C已经生效，也必须回收唯一引擎。
        ::send(fd_, "stop\nquit\n", 10, MSG_NOSIGNAL | MSG_DONTWAIT);
        ::close(fd_);
        fd_ = -1;
    }
    pending_.clear();
    if (pid_ < 0) return true;
    for (int attempt = 0; attempt < 80; ++attempt) {
        int result = waitpid(pid_, nullptr, WNOHANG);
        if (result == pid_ || (result < 0 && errno == ECHILD)) {
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
