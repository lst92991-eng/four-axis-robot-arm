#include "chess/board.hpp"
#include "chess/pikafish.hpp"

#include <filesystem>
#include <cstdlib>
#include <fstream>
#include <stdexcept>
#include <string>
#include <unistd.h>

namespace {
void check(bool value, const char* message)
{
    if (!value) throw std::runtime_error(message);
}

chess::Observation observation(const chess::Board& board)
{
    return {board, true, true, false};
}

chess::Board moved(const chess::Board& before, bool red, const std::string& text)
{
    chess::Move move;
    chess::Board after;
    std::string error;
    check(chess::parseMove(text, move), "test move parse");
    check(chess::applyMove(before, red, move, after, error), "test move application");
    return after;
}

void terminalTests()
{
    // 两局已由4762ca3原始源码的真实Pikafish核验perft=0：前者Checkers=e8，后者为空。
    // 环境变量可指定独立本地引擎再次查询；不设置时只测试棋局对已知空合法表的处理。
    const char* positions[][3] = {
        {"3nkn3/R6R1/9/9/4P4/9/9/9/9/4K4 w - - 0 1", "h8e8",
         "3nkn3/R3R4/9/9/4P4/9/9/9/9/4K4 b - - 0 1"},  // 将死
        {"4k4/3R1r3/5R3/9/4P4/9/9/9/9/4K4 w - - 0 1", "f7f8",
         "4k4/3R1R3/9/9/4P4/9/9/9/9/4K4 b - - 0 1"}    // 吃车后困毙
    };
    const char* directory = std::getenv("SCARA_TEST_PIKAFISH_DIR");
    Pikafish engine;
    if (directory) check(engine.start(directory), "start independent terminal-test engine");
    for (const auto& position : positions) {
        chess::Board board, before;
        bool red = true;
        std::string error;
        check(chess::parseFen(position[0], before, red, error) && red, "terminal predecessor FEN");
        check(chess::parseFen(position[2], board, red, error) && !red, "terminal FEN with black to move");
        check(board[0][4] == 'k' && board[9][4] == 'K', "terminal state retains both generals");
        std::vector<std::string> legal{position[1]};
        if (directory) check(engine.legalMoves(position[0], "", legal), "query actual predecessor legal moves");
        chess::Game game;
        chess::Move move;
        check(game.reset(before) && game.acceptHuman(observation(board), legal, move), "terminal move is legal and observed");
        check(game.baseFen() == position[0] && game.history() == position[1], "keep baseline and terminal move history");
        legal.clear();
        if (directory) check(engine.legalMoves(game.baseFen(), game.history(), legal), "query terminal through complete history");
        check(legal.empty(), "mate and stalemate both have zero legal moves");
        check(game.judge(legal) && game.result() == chess::Result::RedWon,
              "black loses for both mate and stalemate");
    }
    check(engine.close(), "terminal-test engine is reaped");
}

void protocolTests()
{
    // 只启动本测试的本地协议替身，不连接正在运行的板端引擎。
    char pattern[] = "/tmp/scara-chess-XXXXXX";
    char* name = mkdtemp(pattern);
    check(name != nullptr, "temporary engine directory");
    struct Directory {
        std::filesystem::path path;
        ~Directory() { std::error_code error; std::filesystem::remove_all(path, error); }
    } directory{name};
    const auto executable = directory.path / "pikafish";
    {
        std::ofstream script(executable);
        script << "#!/bin/sh\n"
                  "while IFS= read -r line; do\n"
                  "case \"$line\" in\n"
                  "uci) printf 'uciok\\n' ;;\n"
                  "isready) printf 'readyok\\n' ;;\n"
                  "position*) printf '%s\\n' \"$line\" > seen-position ;;\n"
                  "'go perft 1') printf 'e3e4: 1\\nNodes searched: 1\\n' ;;\n"
                  "'go movetime '*) printf 'info depth 1\\nbestmove e3e4\\n' ;;\n"
                  "quit) exit 0 ;;\n"
                  "esac\ndone\n";
    }
    std::filesystem::permissions(executable, std::filesystem::perms::owner_all);
    Pikafish engine;
    check(engine.start(name), "fake engine handshake");
    std::vector<std::string> legal;
    check(engine.legalMoves("", legal) && legal == std::vector<std::string>{"e3e4"}, "legacy perft interface");
    std::string seen;
    { std::ifstream file(directory.path / "seen-position"); std::getline(file, seen); }
    check(seen == "position startpos", "legacy startpos command");
    auto board = chess::initialBoard();
    board[9][0] = '.';
    const std::string base = chess::fen(board, true);
    check(engine.legalMoves(base, "c3c4 c6c5", legal), "FEN and history perft");
    { std::ifstream file(directory.path / "seen-position"); std::getline(file, seen); }
    check(seen == "position fen " + base + " moves c3c4 c6c5", "preserve custom FEN plus full history");
    std::string best;
    check(engine.think(base, "c3c4 c6c5", 1, best) && best == "e3e4", "FEN search interface");
    check(!engine.legalMoves(base + "\nquit", "", legal), "reject FEN command injection");
    check(engine.start(name), "restart local fake engine");
    check(!engine.legalMoves("a0a0", legal), "reject zero-length move in history");
}
} // namespace

void testChess()
{
    using namespace chess;
    const Board start = initialBoard();
    const std::string startFen = "rnbakabnr/9/1c5c1/p1p1p1p1p/9/9/P1P1P1P1P/1C5C1/9/RNBAKABNR w - - 0 1";
    check(fen(start, true) == startFen, "initial FEN and board direction");
    Move move;
    check(parseMove("a0i9", move) && move.fromRow == 9 && move.fromCol == 0 &&
          move.toRow == 0 && move.toCol == 8 && uci(move) == "a0i9", "UCI rank direction");
    check(!parseMove("a0a0", move) && !parseMove("j0a0", move) &&
          !parseMove("a0i10", move) && uci(Move{}).empty(), "reject invalid coordinates");
    Board parsed;
    bool red = false;
    std::string error;
    check(parseFen(startFen, parsed, red, error) && parsed == start && red, "FEN roundtrip");
    check(!parseFen("9/9 w - - 0 1", parsed, red, error), "reject incomplete FEN");
    check(!parseFen(startFen + " extra", parsed, red, error), "reject extra FEN fields");
    Board unknown = start; unknown[4][0] = '?';
    check(!validBoard(unknown, error) && fen(unknown, true).empty(), "unknown is not empty");
    Board missingKing = start; missingKing[0][4] = '.';
    check(!validBoard(missingKing, error), "missing general is a vision/setup error");
    Board tooMany = start; tooMany[4][0] = 'R';
    check(!validBoard(tooMany, error), "reject impossible piece inventory");

    const Board afterHuman = moved(start, true, "e3e4");
    auto seen = observation(afterHuman);
    Game game;
    seen.stable = false;
    check(!game.acceptHuman(seen, {"e3e4"}, move) && game.history().empty(), "unstable frame not committed");
    seen = observation(afterHuman); seen.complete = false;
    check(!game.acceptHuman(seen, {"e3e4"}, move), "incomplete frame rejected");
    seen = observation(afterHuman); seen.duplicate = true;
    check(!game.acceptHuman(seen, {"e3e4"}, move), "duplicate detection rejected");
    check(!game.acceptHuman(observation(unknown), {"e3e4"}, move), "unknown frame rejected");
    check(!game.acceptHuman(observation(missingKing), {"e3e4"}, move) &&
          game.result() == Result::Playing, "missing general must not end game");
    check(!inferMove(start, observation(afterHuman), true, {"e3e4", "e3e4"}, move, error), "duplicate legal list rejected");
    check(!inferMove(start, observation(start), true, {"e3e4"}, move, error), "unchanged image is not a move");
    check(!game.acceptHuman(observation(afterHuman), {"c3c4"}, move), "reject nonlegal board transition");
    check(game.acceptHuman(observation(afterHuman), {"c3c4", "e3e4"}, move) &&
          uci(move) == "e3e4" && !game.redToMove(), "accept unique legal human move");
    check(!game.prepareRobot(observation(start), "a6a5", {"a6a5"}, move), "pre-motion board must match");
    check(!game.prepareRobot(observation(afterHuman), "c6c5", {"a6a5"}, move), "reject unlisted engine choice");
    check(game.prepareRobot(observation(afterHuman), "a6a5", {"a6a5"}, move) &&
          game.pending() && game.board() == afterHuman && game.history() == "e3e4", "plan does not commit motion");
    const Board afterRobot = moved(afterHuman, false, "a6a5");
    check(!game.judge({}), "no terminal judgment during pending motion");
    check(!game.confirmRobot(observation(afterRobot), false), "failed actuator cannot commit");
    check(!game.confirmRobot(observation(afterHuman), true), "wrong after-image cannot commit");
    check(game.pending() && game.history() == "e3e4", "failed confirmation preserves confirmed history");
    check(game.confirmRobot(observation(afterRobot), true) && !game.pending() &&
          game.redToMove() && game.history() == "e3e4 a6a5", "commit only after arm and vision confirmation");
    check(!game.confirmRobot(observation(afterRobot), true), "duplicate completion cannot commit twice");

    Board capture = start;
    capture[6][0] = '.'; capture[4][0] = 'P';
    Board captured = moved(capture, true, "a5a6");
    check(inferMove(capture, observation(captured), true, {"a5a6", "c3c4"}, move, error) &&
          uci(move) == "a5a6" && captured[3][0] == 'P' && captured[4][0] == '.', "infer capture by full board difference");
    check(game.reset(capture) && game.baseFen() == fen(capture, true) && game.history().empty(), "custom confirmed baseline");
    check(game.judge({}) && game.result() == Result::BlackWon, "red with no legal move loses even with king present");
    check(game.reset(start, false) && game.judge({}) && game.result() == Result::RedWon, "black stalemate loses");
    check(game.reset(start) && game.agreeDraw() && game.result() == Result::Draw, "agreed draw");
    check(game.reset(start) && game.resign(false) && game.result() == Result::RedWon, "explicit resignation");

    // 合法马往返让相同盘面和轮次出现三次；只暂停，不擅自宣布和棋。
    check(game.reset(start), "reset repetition game");
    const std::string cycle[] = {"b0c2", "b9c7", "c2b0", "c7b9"};
    for (int i = 0; i < 8; ++i) {
        const std::string next = cycle[i % 4];
        const Board after = moved(game.board(), game.redToMove(), next);
        if (game.redToMove()) check(game.acceptHuman(observation(after), {next}, move), "repetition human move");
        else {
            check(game.prepareRobot(observation(game.board()), next, {next}, move), "repetition robot plan");
            check(game.confirmRobot(observation(after), true), "repetition robot confirmation");
        }
        check(game.judge({cycle[(i + 1) % 4]}), "repetition judgment");
    }
    check(game.result() == Result::ReviewRequired, "third repetition requires review, not automatic draw");
    check(game.agreeDraw() && game.result() == Result::Draw, "participants may agree draw after review pause");
    terminalTests();
    protocolTests();
}
