// 用数字行列保存中国象棋盘面，在引擎协议和棋谱边界转换走法。
#include "board.hpp"

#include <algorithm>

namespace chess
{

//---------初始化区---------
// 标准开局按实盘从黑方底线向红方逐行排。点号表示空交点。
Board initialBoard()
{
    constexpr char ranks[Rows][Columns+1] = {
        "rnbakabnr", ".........", ".c.....c.", "p.p.p.p.p", ".........",
        ".........", "P.P.P.P.P", ".C.....C.", ".........", "RNBAKABNR"
    };
    Board board;
    for (int row = 0; row < Rows; ++row)
        std::copy_n(ranks[row], Columns, board[row].begin());
    return board;
}

//默认构造即建立标准局面，字段初值保存其余回合状态
Game::Game()
{
    Game_reset(*this, initialBoard());
}

// 以核对过的实盘作为开局，刷新行棋方、起始 FEN 和历史记录。
bool Game_reset(Game &game, const Board &board, bool redToMove, bool humanRed)
{
    if (!validBoard(board)) return false;
    game.board = board;
    game.redToMove = redToMove;
    game.humanRed = humanRed;
    game.result = Result::Playing;
    game.history.clear();
    // 起始 FEN 留给引擎回放，positions 的第一项是这局的起始盘面。
    game.baseFen = startFen(game.board, redToMove);
    game.positions = {board};
    return true;
}

//---------搬子数据核对区---------
// 起点、终点的行列四项全部相同，两条走法才相等。
bool operator==(const Move &left, const Move &right)
{
    return left.fromRow == right.fromRow && left.fromCol == right.fromCol &&
           left.toRow == right.toRow && left.toCol == right.toCol;
}
// 检查数组下标时，行的范围是 0–9，列的范围是 0–8。
static bool inside(int row, int col)
{
    return row >= 0 && row < Rows && col >= 0 && col < Columns;
}
// 两个交点都要在盘内，而且起点与终点要分开。
bool validSquares(const Move &m)
{
    return inside(m.fromRow, m.fromCol) && inside(m.toRow, m.toCol) &&
           (m.fromRow != m.toRow || m.fromCol != m.toCol);
}
// 棋盘字符用大写表示红方，用小写表示黑方。
static bool redPiece(char piece)
{
    return piece >= 'A' && piece <= 'Z';
}
// 将帅都在第 3–5 列，黑将限于上三行，红帅限于下三行。
static bool kingInPalace(char piece, int row, int col)
{
    bool column = col >= 3 && col <= 5;
    if (piece == 'K') return column && row >= 7;
    if (piece == 'k') return column && row <= 2;
    return true;
}

//---------开局盘面核对区---------
// 统计各类棋子，并核对将帅在九宫内。
bool validBoard(const Board &board)
{
    // 每方依次有 1 将、2 士、2 象、2 马、2 车、2 炮、5 卒。
    const int maximum[] = {1, 2, 2, 2, 2, 2, 5};
    int counts[14] = {};
    for (int row = 0; row < Rows; ++row)
        for (int col = 0; col < Columns; ++col)
        {
            const char piece = board[row][col];
            if (piece == '.') continue;
            // 类别表的前七项是黑棋，后七项是红棋。同一兵种共用一个数量上限。
            size_t index = PieceSymbols.find(piece);
            if (index == std::string::npos) return false;
            ++counts[index];
            if (counts[index] > maximum[index % 7]) return false;
            if (!kingInPalace(piece, row, col)) return false;
        }
    // 两方还要各有一个将或帅，才能用这张盘面开局。
    return counts[0] == 1 && counts[7] == 1;
}

//---------搬子与识别手走区---------
// 引擎或旧棋谱给出的外部数据，在送给棋盘之前核对一次。
bool validTransfer(const Board &board, bool redToMove, const Move &move)
{
    if (!validSquares(move)) return false;
    // 按当前行棋方核对起点颜色，并检查目标格可被占用。
    char piece = board[move.fromRow][move.fromCol];
    char target = board[move.toRow][move.toCol];
    if (piece == '.' || redPiece(piece) != redToMove) return false;
    if (target == 'K' || target == 'k') return false;
    if (target != '.' && redPiece(target) == redToMove) return false;
    return kingInPalace(piece, move.toRow, move.toCol);
}

// 把已核对的走法应用到传入的棋盘。目标格原有的棋子会被覆盖。
void movePiece(Board &board, const Move &move)
{
    board[move.toRow][move.toCol] = board[move.fromRow][move.fromCol];
    board[move.fromRow][move.fromCol] = '.';
}

// 人手走棋后，把引擎给的每一手在棋盘副本上走一次，找出与相机实盘相同的那一手。
bool inferMove(const Board &before, const Board &seen, const MoveList &legal, Move &move)
{
    move = {};
    const Move *match = nullptr;
    for (const Move &step : legal)
    {
        if (!validSquares(step)) continue;
        Board trial = before;
        movePiece(trial, step);
        if (trial != seen) continue;
        if (match) return false; // 两项都吻合，无法唯一记谱。
        match = &step;
    }
    if (!match || before == seen) return false;
    move = *match;
    return true;
}

//---------回合交接区---------
//落子已经确认后，更新权威盘面、完整历史和重复局面记录
void Game_commit(Game &game, const Move &move)
{
    movePiece(game.board, move);
    //追加已确认的一步，再交换行棋方，供下一次引擎查询使用
    if (!game.history.empty()) game.history += ' ';
    game.history += uci(move);
    game.redToMove = !game.redToMove;
    // 保存换边后的盘面，供后续按相同行棋方检查重复局面。
    game.positions.push_back(game.board);
}

// 接续棋谱时逐手转换坐标、核对搬子数据，再复用正常记谱流程。
bool Game_replay(Game &game, const std::string &text)
{
    Move move;
    if (!parseMove(text, move) || !validTransfer(game.board, game.redToMove, move)) return false;
    Game_commit(game, move);
    return true;
}

//合法表为空时本方负，重复局面交人工裁定
void Game_judge(Game &game, const MoveList &legal)
{
    // 已有终局结果时保留判定。当前行棋方无合法着法时，对方获胜。
    if (game.result != Result::Playing) return;
    if (legal.empty())
    {
        game.result = game.redToMove ? Result::BlackWon : Result::RedWon;
        return;
    }
    // 每隔两手才轮到同一方，相同盘面出现三次交裁定。
    int repeats = 0;
    for (int step = int(game.positions.size()) - 1; step >= 0; step -= 2)
        if (game.positions[step] == game.board) ++repeats;
    // 出现三次相同局面，或记录超过 1024 张盘面时，转入人工裁定。
    if (repeats >= 3 || game.positions.size() > 1024) game.result = Result::ReviewRequired;
}

} // namespace chess
