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
    int fromRow = -1; // 起点行，从黑方底线起算。
    int fromCol = -1; // 起点列，从红方视角左侧起算。
    int toRow = -1;
    int toCol = -1;
};
struct Observation
{
    Board board{};
    //已完成识别归格和不同帧稳定确认，棋局变化由合法走法核对
    bool valid = false;
};

void initialBoard(Board &board);
bool parseMove(const std::string &text, Move &move);
bool uci(const Move &move, std::string &text);
bool validBoard(const Board &board, std::string &error);
//已确认盘面生成FEN，供引擎读取行棋方和棋子布局
void fen(const Board &board, bool redToMove, int halfmoves, int fullmove, std::string &text);
//按已经查过合法表的走法更新起点和终点，并核对搬子数据
bool applyMove(const Board &before, bool redToMove, const Move &move, Board &after, std::string &error);
//before为已确认盘面，legal来自成功的Pikafish查询；按变化的两格识别人类走法
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

//只保存已确认棋局；机器人动作的预期盘面由应用回合局部保存
struct Game
{
    Game();
    Board board{};         // 已确认的权威盘面。
    bool redToMove = true; // 当前轮到红方。
    bool humanRed = true;  // 人类使用红方。
    Result result = Result::Playing;
    std::string baseFen; // 起始局面，供引擎重放完整历史。
    std::string history; // 已完成走法，按UCI格式保留完整顺序。
    std::string error;
    //保存每一步完成后的局面，含开局；步数为size()-1，用于重复局面裁定
    std::vector<std::string> positions;
};

bool Game_reset(Game &game, const Board &board, bool redToMove = true, bool humanRed = true);
//调用者已确认走法和实际盘面，在这里统一写入历史并换边
void Game_commit(Game &game, const Board &after, const Move &move);
// legal须来自成功的Pikafish::legalMoves；无合法走法即负，复杂重复局面交人工裁定。
bool Game_judge(Game &game, const std::vector<std::string> &legal);
//上层在允许认输或和棋的阶段提交结果
bool Game_finish(Game &game, Result outcome);

} // namespace chess
