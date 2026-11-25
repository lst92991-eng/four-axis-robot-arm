// RV1106 通过 UCI 查询皮卡鱼。一轮查询使用同一局面取得合法表和电脑应手。
#include "pikafish.hpp"
#include <algorithm>
#include <cerrno>
#include <chrono>
#include <poll.h>
#include <sstream>
#include <sys/resource.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <sys/wait.h>
#include <unistd.h>

namespace
{
//---------引擎进程与通信数据区---------
using Clock = std::chrono::steady_clock;
// engineFd 是主程序持有的通信端，enginePid 用于退出时回收子进程。
int engineFd = -1, enginePid = -1;
// 一次查询共用运行标志、当前阶段的截止时间和未读完的回复文本。
struct Uci
{
    const volatile std::sig_atomic_t &running;
    Clock::time_point deadline;
    std::string buffer;
};
bool openEngine(const char *directory);
bool writeText(Uci &io, const std::string &text);
bool readLine(Uci &io, std::string &line);
bool handshake(Uci &io, const char *command, const char *reply);
bool readLegal(Uci &io, chess::MoveList &legal);

bool askEngine(Uci &io, const chess::Game &game, int thinkMs, chess::MoveList &legal, chess::Move &best);
} // namespace

//---------查询入口与结果交接区---------
bool Pikafish_query(const char *directory, const chess::Game &game, int thinkMs,
                    const volatile std::sig_atomic_t &running, chess::MoveList &legal, chess::Move &best)
{
    legal.clear();
    best = {};
    // 先回收上次的引擎进程，再检查 FEN 和历史中的换行符，保证 position 是一条命令。
    if (!Pikafish_close() || !running || !directory || !*directory) return false;
    if (game.baseFen.empty() || game.baseFen.find_first_of("\r\n") != std::string::npos ||
        game.history.find_first_of("\r\n") != std::string::npos) return false;
    Uci io{running, Clock::now() + std::chrono::seconds(5), {}};
    bool ok = openEngine(directory) && askEngine(io, game, thinkMs, legal, best);
    // 子进程已回收且查询成功，合法表和应手才交给对局。
    if (Pikafish_close() && ok) return true;
    legal.clear();
    best = {};
    return false;
}

namespace
{
//---------一手棋与引擎的对话---------
bool askEngine(Uci &io, const chess::Game &game, int thinkMs, chess::MoveList &legal, chess::Move &best)
{
    // 先用 uciok 确认协议，再设为单线程、4 MiB 哈希表，等引擎回复 readyok。
    if (!handshake(io, "uci\n", "uciok")) return false;
    if (!handshake(io, "setoption name Threads value 1\nsetoption name Hash value 4\nisready\n", "readyok"))
        return false;

    // 局面由起始 FEN 和已确认棋谱组成，本回合只发一次。
    std::string position = "position fen " + game.baseFen;
    if (!game.history.empty()) position += " moves " + game.history;
    io.deadline = Clock::now() + std::chrono::seconds(3);
    // go perft 1 列出当前所有合法走法，readLegal 把各行转成数字起终点。
    if (!writeText(io, position + "\ngo perft 1\n") || !readLegal(io, legal)) return false;
    // 玩家回合只需要高亮可走位置。合法表为空时，交给棋局判负。
    if (legal.empty() || game.redToMove == game.humanRed) return true;

    // 轮到电脑，读取 bestmove 后的起点和终点。
    // 等待 bestmove 的时限包含指定的思考时间和 2 秒通信余量。
    io.deadline = Clock::now() + std::chrono::milliseconds(thinkMs + 2000);
    if (!writeText(io, "go movetime " + std::to_string(thinkMs) + "\n")) return false;
    std::string line;
    while (readLine(io, line))
    {
        std::istringstream reply(line);
        std::string kind;
        if (!(reply >> kind) || kind == "info") continue;
        if (kind != "bestmove") return false;
        std::string move;
        reply >> move;
        return chess::parseMove(move, best);
    }
    return false;
}

//---------合法表与握手回复区---------
// perft 1 为每一手输出一行，例如 a3a4: 1，末行给出总数。
bool readLegal(Uci &io, chess::MoveList &legal)
{
    std::string line;
    while (readLine(io, line))
    {
        if (line.empty()) continue;
        // 末行的节点总数必须等于前面收到的走法条数。
        if (line.front() == 'N') return line == "Nodes searched: " + std::to_string(legal.size());
        std::string_view record(line);
        if (record.size() != 7 || record.substr(4) != ": 1") return false;
        chess::Move move;
        if (!chess::parseMove(record.substr(0, 4), move)) return false;
        if (legal.size() == 256 || std::find(legal.begin(), legal.end(), move) != legal.end()) return false;
        legal.push_back(move);
    }
    return false;
}

// 发送握手命令，逐行跳过引擎自述，直到看到这一步约定的回复。
bool handshake(Uci &io, const char *command, const char *reply)
{
    if (!writeText(io, command)) return false;
    std::string line;
    while (readLine(io, line))
        if (line == reply) return true;
    return false;
}

//---------启动进程，与标准输入输出相连---------
bool openEngine(const char *directory)
{
    int ends[2];
    // 通信对的一端留在主程序，另一端接到引擎的标准输入和输出。
    if (socketpair(AF_UNIX, SOCK_STREAM, 0, ends) < 0) return false;
    enginePid = fork();
    if (enginePid == 0)
    {
        ::close(ends[0]);
        // 在子进程限制地址空间并切到权重目录，再启动该目录中的引擎。
        rlimit memory{140000UL * 1024, 140000UL * 1024};
        if (setrlimit(RLIMIT_AS, &memory) < 0 || chdir(directory) < 0) _exit(126);
        if (dup2(ends[1], STDIN_FILENO) < 0 || dup2(ends[1], STDOUT_FILENO) < 0) _exit(126);
        if (ends[1] > STDERR_FILENO) ::close(ends[1]);
        execl("./pikafish", "pikafish", static_cast<char *>(nullptr));
        _exit(127);
    }
    // 父进程关闭子进程使用的一端，保留自己的通信描述符。
    ::close(ends[1]);
    engineFd = ends[0];
    // 引擎沉默时每 100 ms 回到主程序，检查取消与本次查询的总时限。
    timeval tick{0, 100000};
    if (enginePid < 0) return false;
    for (int option : {SO_RCVTIMEO, SO_SNDTIMEO})
        if (setsockopt(engineFd, SOL_SOCKET, option, &tick, sizeof(tick)) != 0) return false;
    return true;
}

//---------命令写全，回复逐行读出---------
// 系统中断或暂时没有数据时，允许在本阶段的截止时间内重试。
bool retryIO()
{
    return errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK;
}

bool writeText(Uci &io, const std::string &text)
{
    // 一次发送可能只写出部分字节，从 sent 处接着发剩余命令。
    size_t sent = 0;
    while (sent < text.size() && io.running && Clock::now() < io.deadline)
    {
        ssize_t n = ::send(engineFd, text.data() + sent, text.size() - sent, MSG_NOSIGNAL);
        if (n > 0) sent += size_t(n);
        else if (n == 0 || !retryIO()) return false;
    }
    return sent == text.size();
}

bool readLine(Uci &io, std::string &line)
{
    char packet[2048];
    while (io.running && Clock::now() < io.deadline)
    {
        // 先从缓冲区取一行，换行符后已收到的内容留给下一次读取。
        size_t end = io.buffer.find('\n');
        if (end != std::string::npos)
        {
            line = io.buffer.substr(0, end);
            io.buffer.erase(0, end + 1);
            if (!line.empty() && line.back() == '\r') line.pop_back();
            return true;
        }
        // 还没收齐一行时继续接收，引擎退出或缓冲超限时结束本次读取。
        ssize_t n = recv(engineFd, packet, sizeof(packet), 0);
        if (n < 0 && retryIO()) continue;
        if (n <= 0 || io.buffer.size() + size_t(n) > 16384) return false;
        io.buffer.append(packet, size_t(n));
    }
    return false;
}
} // namespace

//---------引擎退出与回收区---------
bool Pikafish_close()
{
    if (engineFd >= 0)
    {
        // 发送停止搜索和退出命令，再释放主程序的通信端。
        ::send(engineFd, "stop\nquit\n", 10, MSG_NOSIGNAL | MSG_DONTWAIT);
        ::close(engineFd);
        engineFd = -1;
    }
    if (enginePid < 0) return true;
    // 先等待子进程自行退出，仍在运行时依次发送终止信号和强制退出信号。
    for (int attempt = 0; attempt < 80; ++attempt)
    {
        int done = waitpid(enginePid, nullptr, WNOHANG);
        if (done == enginePid || (done < 0 && errno == ECHILD))
        {
            enginePid = -1;
            return true;
        }
        if (attempt == 10) kill(enginePid, SIGTERM);
        if (attempt == 30) kill(enginePid, SIGKILL);
        poll(nullptr, 0, 10);
    }
    return false;
}
