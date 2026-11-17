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
#include <sys/wait.h>
#include <unistd.h>

namespace mock
{
std::string input, output, position;
std::string perft, search;
std::vector<std::string> commands;
int openFds = 0, polls = 0, sends = 0, reads = 0;
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
        std::string fen, best;
        chess::fen(board, true, 0, 1, fen);
        chess::Game game;
        std::vector<std::string> legal;
        Pikafish engine;
        mock::reset();
        mock::interruptSend = true;
        mock::interruptRead = true;
        check(engine.start("fixture"), "Handshake with split/interrupted I/O failed");
        check(engine.legalMoves(game, legal) && legal == std::vector<std::string>{"a3a4", "c3c4"},
              "Perft decode failed");
        chess::Move human;
        chess::Observation observed;
        std::string detail;
        chess::parseMove("a3a4", human);
        chess::applyMove(game.board, true, human, observed.board, detail);
        observed.valid = true;
        check(chess::inferMove(game.board, observed, game.redToMove, {"a3a4"}, human, game.error), "Fixture move confirmation failed");
        chess::Game_commit(game, observed.board, human);
        check(engine.think(game, 120, best) && best == "a6a5", "Best move decode failed");
        check(mock::position == "position fen " + fen + " moves a3a4", "Full FEN/history was not sent");
        check(engine.close() && mock::openFds == 0, "Normal close leaked descriptors");

        mock::reset();
        mock::perft = "a3a4: 1\nNodes searched: 2\n";
        check(engine.start("fixture") && !engine.legalMoves(game, legal) && mock::openFds == 0,
              "Count mismatch accepted");
        mock::reset();
        mock::perft = "a3a4: 1\na3a4: 1\nNodes searched: 2\n";
        check(engine.start("fixture") && !engine.legalMoves(game, legal), "Duplicate move accepted");
        mock::reset();
        mock::search = "bestmove invalid\n";
        check(engine.start("fixture") && !engine.think(game, 20, best), "Invalid bestmove accepted");
        mock::reset();
        auto corrupt = game;
        corrupt.baseFen += "\nquit";
        check(engine.start("fixture") && !engine.think(corrupt, 20, best) && mock::openFds == 0,
              "Multiline FEN was sent to the engine");
        mock::reset();
        mock::forkFails = true;
        check(!engine.start("fixture") && mock::openFds == 0, "Failed start leaked descriptors");
        mock::reset();
        mock::silent = true;
        engine.cancelled = [] { return mock::polls > 8; };
        check(!engine.start("fixture") && mock::openFds == 0, "Cancellation did not stop/reap the engine");
        engine.cancelled = {};
        mock::reset();
        check(engine.start("fixture"), "Restart after cancellation failed");
        mock::eof = true;
        mock::silent = true;
        check(!engine.think(game, 20, best) && mock::openFds == 0, "EOF did not stop the query");
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
