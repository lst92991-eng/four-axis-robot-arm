// 棋局的数据类型与规则接口，供视觉归格、引擎查询和回合管理共用。
#pragma once

#include <array>
#include <string>
#include <string_view>
#include <vector>

namespace chess
{

//---------盘面与走法数据区---------
// 所有业务坐标都按 (行,列)。行 0 是黑方底线，行 9 是红方底线；列 0–8 从红方视角左到右。
// 空位为 '.'，未知为 '?'。仅在 UCI 协议与历史棋谱入口使用字母格号。
constexpr int Rows = 10, Columns = 9;
// 模型的顺序为将、士、象、马、车、炮、卒。小写是黑方，大写是红方。
inline const std::string PieceSymbols = "kabnrcpKABNRCP";
using Board = std::array<std::array<char, Columns>, Rows>;
// 一手走法由起点和终点两个交点组成，默认的 -1 表示尚未选择。
struct Move
{
    int fromRow = -1; // 起点行，从黑方底线起算。
    int fromCol = -1; // 起点列，从红方视角左侧起算。
    int toRow = -1;
    int toCol = -1;
};
// 合法表按起点和终点的四个下标查找选中的走法。
bool operator==(const Move &left, const Move &right);
bool validSquares(const Move &move);
using MoveList = std::vector<Move>;
//---------盘面操作区---------
// 返回黑方在上、红方在下的标准开局。
Board initialBoard();
// 开局时核对棋子编码、数量上限以及将帅所在的九宫。
bool validBoard(const Board &board);
// 外部走法先核对坐标、起点和目标。已确认的走法直接在数组上搬一枚棋子。
bool validTransfer(const Board &board, bool redToMove, const Move &move);
void movePiece(Board &board, const Move &move);
enum class InferredMove
{
    Unchanged,
    Legal,
    Invalid
};
// 用已确认盘面的两处变化确定起终点，找到唯一的合法走法时写入 move。
InferredMove inferMove(const Board &before, const Board &after, const MoveList &legal, Move &move);

//---------棋局结果与历史区---------
// ReviewRequired 表示暂停并交人工裁定，程序在此结束本轮对弈。
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
    //保存每一步完成后的局面，含开局；步数为size()-1，用于重复局面裁定
    std::vector<Board> positions;
};

// 核对起始盘面后，保存双方身份并开始一份空棋谱。
bool Game_reset(Game &game, const Board &board, bool redToMove = true, bool humanRed = true);
//调用者已确认走法和实际盘面，在这里统一写入历史并换边
void Game_commit(Game &game, const Move &move);
// 从已确认的上一局面重放棋谱中的一手 UCI 走法。
bool Game_replay(Game &game, const std::string &text);
// 无合法走法则本方负，复杂重复局面交人工裁定。
void Game_judge(Game &game, const MoveList &legal);

//---------引擎和棋谱的格式转换区---------
// 在 UCI 的四字符走法与两个数字交点之间转换。
bool parseMove(std::string_view text, Move &move);
std::string uci(const Move &move);

} // namespace chess
