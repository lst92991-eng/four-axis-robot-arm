// 编译实际Pikafish接口，POSIX调用替身只模拟本地协议；不创建进程、不连接设备。
#include "chess/pikafish.hpp"
#include "chess/board.hpp"
#include <algorithm>
#include <cerrno>
#include <cstring>
#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <vector>
#include <fcntl.h>
#include <poll.h>
#include <sys/resource.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <sys/wait.h>
#include <unistd.h>

namespace mock
{
std::string input, output, position;
std::string perft, search;
std::vector<std::string> commands;
int openFds = 0, polls = 0, sends = 0, reads = 0, cancelAfterReads = 0;
volatile std::sig_atomic_t running = 1;
int sendLimit = 3, readLimit = 4;
bool exited = false, silent = false, eof = false, interruptSend = false, interruptRead = false;
bool forkFails = false;

void reset()
{
    input.clear();
    output.clear();
    position.clear();
    commands.clear();
    openFds = 0;
    running = 1;
    cancelAfterReads = 0;
    polls = 0;
    sends = 0;
    reads = 0;
    sendLimit = 3;
    readLimit = 4;
    exited = false;
    silent = false;
    eof = false;
    interruptSend = false;
    interruptRead = false;
    forkFails = false;
    perft = "a3a4: 1\nc3c4: 1\n\nNodes searched: 2\n";
    search = "info depth 1 score cp 5\nbestmove a6a5 ponder a4a5\n";
}

void command(const std::string &line)
{
    commands.push_back(line);
    if (line == "quit") exited = true;
    if (silent) return;
    if (line == "uci") output += "id name protocol-fixture\nuciok\r\n";
    if (line == "isready") output += "readyok\n";
    if (line.rfind("position ", 0) == 0) position = line;
    if (line == "go perft 1") output += perft;
    if (line.rfind("go movetime ", 0) == 0) output += search;
}
} // namespace mock

extern "C"
{
    int socketpair(int, int, int, int pair[2])
    {
        pair[0] = 21;
        pair[1] = 22;
        mock::openFds = 2;
        return 0;
    }
    int setsockopt(int, int level, int option, const void *value, unsigned count)
    {
        const auto *tick = static_cast<const timeval *>(value);
        return level == SOL_SOCKET && (option == SO_RCVTIMEO || option == SO_SNDTIMEO) &&
               count == sizeof(timeval) && tick->tv_sec == 0 && tick->tv_usec == 100000 ? 0 : -1;
    }
    int fork()
    {
        return mock::forkFails ? -1 : 9001;
    }
    int close(int)
    {
        --mock::openFds;
        return 0;
    }
    int chdir(const char *)
    {
        return 0;
    }
    int dup2(int, int target)
    {
        return target;
    }
    int execl(const char *, const char *, ...)
    {
        return -1;
    }
    int kill(int, int)
    {
        mock::exited = true;
        return 0;
    }
    [[noreturn]] void scara_test_exit(int)
    {
        std::abort();
    }
    int fcntl(int, int, ...)
    {
        return 0;
    }
    int open(const char *, int, ...)
    {
        return 23;
    }
    int setrlimit(int, const rlimit *)
    {
        return 0;
    }
    int waitpid(int pid, int *, int)
    {
        return mock::exited ? pid : 0;
    }
    int poll(pollfd *descriptor, unsigned long, int)
    {
        ++mock::polls;
        if (!descriptor) return 0;
        descriptor->revents = 0;
        if (descriptor->events & POLLOUT)
            descriptor->revents = POLLOUT;
        else if (!mock::output.empty())
            descriptor->revents = POLLIN;
        else if (mock::eof)
            descriptor->revents = POLLHUP;
        return descriptor->revents ? 1 : 0;
    }
    ssize_t send(int, const void *data, size_t size, int flags)
    {
        ++mock::sends;
        if (mock::interruptSend)
        {
            mock::interruptSend = false;
            errno = EINTR;
            return -1;
        }
        size_t count = (flags & MSG_DONTWAIT) ? size : std::min(size, size_t(mock::sendLimit));
        mock::input.append(static_cast<const char *>(data), count);
        size_t end = mock::input.find('\n');
        while (end != std::string::npos)
        {
            mock::command(mock::input.substr(0, end));
            mock::input.erase(0, end + 1);
            end = mock::input.find('\n');
        }
        return count;
    }
    ssize_t recv(int, void *data, size_t size, int)
    {
        ++mock::reads;
        if (mock::cancelAfterReads && mock::reads >= mock::cancelAfterReads) mock::running = 0;
        if (mock::eof && mock::output.empty()) return 0;
        if (mock::interruptRead)
        {
            mock::interruptRead = false;
            errno = EINTR;
            return -1;
        }
        if (mock::output.empty())
        {
            errno = EAGAIN;
            return -1;
        }
        size_t count = std::min({size, mock::output.size(), size_t(mock::readLimit)});
        std::memcpy(data, mock::output.data(), count);
        mock::output.erase(0, count);
        return count;
    }
}

static void check(bool value, const char *message)
{
    if (!value) throw std::runtime_error(message);
}

int main()
{
    try
    {
        chess::Board board;
        chess::initialBoard(board);
        std::string fen;
        chess::Move best;
        chess::fen(board, true, 0, 1, fen);
        chess::Game game;
        chess::MoveList legal;
        mock::reset();
        mock::interruptSend = true;
        mock::interruptRead = true;
        check(Pikafish_query("fixture", game, 120, mock::running, legal, best) && legal == chess::MoveList{{6,0,5,0},{6,2,5,2}},
              "Perft decode failed");
        chess::Move human;
        chess::Observation observed;
        chess::parseMove("a3a4", human);
        observed.board = game.board;
        chess::movePiece(observed.board, human);
        observed.valid = true;
        check(chess::inferMove(game.board, observed, {{6,0,5,0}}, human), "Fixture move confirmation failed");
        chess::Game_commit(game, human);
        check(Pikafish_query("fixture", game, 120, mock::running, legal, best) && best == chess::Move{3,0,4,0}, "Best move decode failed");
        check(mock::position == "position fen " + fen + " moves a3a4", "Full FEN/history was not sent");
        check(std::count(mock::commands.begin(), mock::commands.end(), mock::position) == 1,
              "One engine reply sent the position twice");
        check(Pikafish_close() && mock::openFds == 0, "Normal close leaked descriptors");

        mock::reset();
        mock::perft = "a3a4: 1\nNodes searched: 2\n";
        check(!Pikafish_query("fixture", game, 20, mock::running, legal, best) && legal.empty() && mock::openFds == 0,
              "Count mismatch accepted");
        mock::reset();
        mock::perft = "a3a4: 1\na3a4: 1\nNodes searched: 2\n";
        check(!Pikafish_query("fixture", game, 20, mock::running, legal, best) && legal.empty(), "Duplicate move accepted");
        mock::reset();
        mock::search = "bestmove invalid\n";
        check(!Pikafish_query("fixture", game, 20, mock::running, legal, best), "Invalid bestmove accepted");
        mock::reset();
        auto corrupt = game;
        corrupt.baseFen += "\nquit";
        check(!Pikafish_query("fixture", corrupt, 20, mock::running, legal, best) && mock::openFds == 0,
              "Multiline FEN was sent to the engine");
        mock::reset();
        mock::forkFails = true;
        check(!Pikafish_query("fixture", game, 20, mock::running, legal, best) && mock::openFds == 0, "Failed start leaked descriptors");
        mock::reset();
        mock::silent = true;
        mock::cancelAfterReads = 8;
        check(!Pikafish_query("fixture", game, 20, mock::running, legal, best) && mock::openFds == 0, "Cancellation did not stop/reap the engine");
        mock::reset();
        check(Pikafish_query("fixture", game, 20, mock::running, legal, best), "Restart after cancellation failed");
        mock::eof = true;
        mock::silent = true;
        check(!Pikafish_query("fixture", game, 20, mock::running, legal, best) && mock::openFds == 0, "EOF did not stop the query");
        std::cout
            << "PASS UCI handshake, partial I/O, full history, perft, bestmove, cancellation and cleanup; simulated POSIX\n";
        return 0;
    }
    catch (const std::exception &error)
    {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
