#pragma once

#include <array>
#include <string>
#include <vector>

namespace chess
{

// row=0 是黑方底线；col=0 是红方视角最左列。空位为 '.'，未知为 '?'。
using Board = std::array<std::array<char, 9>, 10>;
struct Move
{
    int fromRow = -1, fromCol = -1, toRow = -1, toCol = -1;
};
struct Observation
{
    Board board{};
    bool complete = false;
    bool stable = false;
    bool duplicate = false;
};

void initialBoard(Board &board);
bool parseMove(const std::string &text, Move &move);
bool uci(const Move &move, std::string &text);
bool validBoard(const Board &board, std::string &error);
bool fen(const Board &board, bool redToMove, int halfmoves, int fullmove, std::string &text);
// 只检查搬子数据，不重复实现象棋规则；调用者必须先查引擎合法走法表。
bool applyMove(const Board &before, bool redToMove, const Move &move, Board &after, std::string &error);
bool inferMove(const Board &before, const Observation &after, bool redToMove, const std::vector<std::string> &legal,
               Move &move, std::string &error);

enum class Result
{
    Playing,
    RedWon,
    BlackWon,
    Draw,
    ReviewRequired
};

// 权威盘面与待确认盘面分开保存；外部读取状态，通过下面的函数推进对局。
struct Game
{
    Game();
    Board board{};         // 已确认的权威盘面。
    Board expected{};      // 待执行走法完成后应看到的盘面。
    bool redToMove = true; // 当前轮到红方。
    bool humanRed = true;  // 人类使用红方。
    bool pending = false;  // 正在等待机器动作及新盘面确认。
    Move pendingMove;
    Result result = Result::Playing;
    std::string baseFen; // 起始局面，不能每回合用新盘面覆盖。
    std::string history; // 已完成走法，按UCI格式保留完整顺序。
    std::string error;
    std::vector<std::string> positions;
    int moveCount = 0;
};

bool Game_reset(Game &game, const Board &board, bool redToMove = true, bool humanRed = true);
bool Game_acceptHuman(Game &game, const Observation &after, const std::vector<std::string> &legal, Move &move);
bool Game_prepareRobot(Game &game, const Observation &before, const std::string &choice,
                       const std::vector<std::string> &legal, Move &move);
bool Game_confirmRobot(Game &game, const Observation &after, bool armSucceeded);
// 无合法走法即负；复杂重复局面仍交人工裁定。
bool Game_judge(Game &game, const std::vector<std::string> &legal);
// 同意和棋或认输时，提交明确结果；正在搬子时不能结束对局。
bool Game_finish(Game &game, Result outcome);

} // namespace chess
