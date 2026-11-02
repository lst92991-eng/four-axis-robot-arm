#pragma once

#include <array>
#include <string>
#include <vector>

namespace chess {

// row=0 是黑方底线；col=0 是红方视角最左列。空位为 '.'，未知为 '?'。
using Board = std::array<std::array<char, 9>, 10>;
struct Move { int fromRow = -1, fromCol = -1, toRow = -1, toCol = -1; };
struct Observation {
    Board board{};
    bool complete = false;
    bool stable = false;
    bool duplicate = false;
};

Board initialBoard();
bool parseMove(const std::string& text, Move& move);
std::string uci(const Move& move);
bool validBoard(const Board& board, std::string& error);
std::string fen(const Board& board, bool redToMove, int halfmoves = 0, int fullmove = 1);
bool parseFen(const std::string& text, Board& board, bool& redToMove, std::string& error);
// 只检查搬子数据，不重复实现象棋规则；调用者必须先查引擎合法走法表。
bool applyMove(const Board& before, bool redToMove, const Move& move,
               Board& after, std::string& error);
bool inferMove(const Board& before, const Observation& after, bool redToMove,
               const std::vector<std::string>& legal, Move& move, std::string& error);

enum class Result { Playing, RedWon, BlackWon, Draw, ReviewRequired };

class Game {
public:
    Game();
    bool reset(const Board& board, bool redToMove = true, bool humanRed = true);
    bool acceptHuman(const Observation& after, const std::vector<std::string>& legal, Move& move);
    bool prepareRobot(const Observation& before, const std::string& choice,
                      const std::vector<std::string>& legal, Move& move);
    bool confirmRobot(const Observation& after, bool armSucceeded);
    // 无合法走法即负，包括困毙。三次重复只暂停，不冒充长将/追逐竞赛裁判。
    bool judge(const std::vector<std::string>& legal);
    bool agreeDraw();
    bool resign(bool red);

    const Board& board() const { return board_; }
    const Board& expectedBoard() const { return expected_; }
    const std::string& baseFen() const { return baseFen_; }
    const std::string& history() const { return history_; }
    bool redToMove() const { return redToMove_; }
    bool pending() const { return pending_; }
    Result result() const { return result_; }
    std::string error;

private:
    Board board_{}, expected_{};
    bool redToMove_ = true, humanRed_ = true, pending_ = false;
    Move pendingMove_;
    Result result_ = Result::Playing;
    std::string baseFen_, history_;
    std::vector<std::string> positions_;
    int moveCount_ = 0;

    bool fail(const std::string& message);
    bool canMove();
    void commit(const Board& after, const Move& move);
};

} // namespace chess
