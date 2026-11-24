// 用数字行列保存中国象棋盘面，在引擎协议和棋谱边界转换走法。
#include "board.hpp"

#include <algorithm>

namespace chess
{
//---------初始化区---------
//填写标准开局，行0为黑方底线
void initialBoard(Board &board)
{
    for (auto &rank : board) rank.fill('.');
    // 底线从左到右是车、马、象、士、将、士、象、马、车。大写为红方。
    constexpr char blackHome[] = "rnbakabnr", redHome[] = "RNBAKABNR";
    std::copy_n(blackHome, Columns, board[0].begin());
    std::copy_n(redHome, Columns, board[Rows - 1].begin());
    // 两方的炮都在第 1、7 列，分别位于第 2 行和第 7 行。
    for (int col : {1, 7})
    {
        board[2][col] = 'c';
        board[7][col] = 'C';
    }
    // 黑卒、红兵各五枚，从第 0 列起隔一列放一枚。
    for (int col = 0; col < Columns; col += 2)
    {
        board[3][col] = 'p';
        board[6][col] = 'P';
    }
}

//默认构造即建立标准局面，字段初值保存其余回合状态
Game::Game()
{
    initialBoard(board);
    fen(board, redToMove, 0, 1, baseFen);
    positions = {board};
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
    fen(game.board, redToMove, 0, 1, game.baseFen);
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
namespace
{
// 棋盘字符用大写表示红方，用小写表示黑方。
bool redPiece(char piece)
{
    return piece >= 'A' && piece <= 'Z';
}
// 将帅都在第 3–5 列，黑将限于上三行，红帅限于下三行。
bool kingInPalace(char piece, int row, int col)
{
    bool column = col >= 3 && col <= 5;
    if (piece == 'K') return column && row >= 7;
    if (piece == 'k') return column && row <= 2;
    return true;
}

} // namespace

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
            if (kingInPalace(piece, row, col) == false) return false;
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
    bool ownPiece = piece != '.' && redPiece(piece) == redToMove;
    bool openTarget = target == '.' || redPiece(target) != redToMove;
    return ownPiece && openTarget && target != 'K' && target != 'k' && kingInPalace(piece, move.toRow, move.toCol);
}

// 把已核对的走法应用到传入的棋盘。目标格原有的棋子会被覆盖。
void movePiece(Board &board, const Move &move)
{
    board[move.toRow][move.toCol] = board[move.fromRow][move.fromCol];
    board[move.fromRow][move.fromCol] = '.';
}

// 实盘只有起点变空、终点变为所走棋子时，才能推断出一手棋。
InferredMove inferMove(const Board &before, const Observation &after, const MoveList &legal,
                       Move &move)
{
    move = {};
    if (!after.valid) return InferredMove::Invalid;

    Move candidate;
    for (int row = 0; row < Rows; ++row)
        for (int col = 0; col < Columns; ++col)
        {
            char piece = after.board[row][col];
            if (before[row][col] == piece) continue;
            if (piece == '.')
            {
                if (candidate.fromRow >= 0) return InferredMove::Invalid;
                candidate.fromRow = row;
                candidate.fromCol = col;
            }
            else
            {
                if (candidate.toRow >= 0) return InferredMove::Invalid;
                candidate.toRow = row;
                candidate.toCol = col;
            }
        }
    if (candidate.fromRow < 0 && candidate.toRow < 0) return InferredMove::Unchanged;
    if (candidate.fromRow < 0 || candidate.toRow < 0) return InferredMove::Invalid;

    // 吃子时终点原有棋子被覆盖，观察到的新棋子仍须与起点原棋子一致。
    char piece = before[candidate.fromRow][candidate.fromCol];
    if (PieceSymbols.find(piece) == std::string::npos || after.board[candidate.toRow][candidate.toCol] != piece)
        return InferredMove::Invalid;
    if (std::count(legal.begin(), legal.end(), candidate) != 1) return InferredMove::Invalid;

    move = candidate;
    return InferredMove::Legal;
}

//---------回合交接区---------
//落子已经确认后，更新权威盘面、完整历史和重复局面记录
void Game_commit(Game &game, const Move &move)
{
    std::string moveText;

    movePiece(game.board, move);
    //追加已确认的一步，再交换行棋方，供下一次引擎查询使用
    uci(move, moveText);
    if (!game.history.empty()) game.history += ' ';
    game.history += moveText;
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

//---------引擎和棋谱的格式转换区---------
//输入四字符UCI走法，输出黑方底线起算的数组行列
bool parseMove(const std::string &text, Move &move)
{
    // 先把输出置为无效走法，四个字符全部核对后再写回结果。
    move = {};
    if (text.size() != 4) return false;
    // UCI从红方底线记作0；棋局数组从黑方底线记作第0行。
    Move candidate;
    candidate.fromRow = Rows - 1 - (text[1] - '0');
    candidate.fromCol = text[0] - 'a';
    candidate.toRow = Rows - 1 - (text[3] - '0');
    candidate.toCol = text[2] - 'a';
    if (!validSquares(candidate)) return false;
    move = candidate;
    return true;
}

//输入起点终点行列，输出引擎使用的四字符走法
bool uci(const Move &move, std::string &text)
{
    text.clear();
    //格式转换入口统一核对下标，随后才能访问棋盘数组
    if (!validSquares(move)) return false;
    text += char('a' + move.fromCol);
    text += char('0' + Rows - 1 - move.fromRow);
    text += char('a' + move.toCol);
    text += char('0' + Rows - 1 - move.toRow);
    return true;
}

//将已确认的90格压缩成FEN，附上行棋方与回合计数
void fen(const Board &board, bool redToMove, int halfmoves, int fullmove, std::string &result)
{
    result.clear();
    //FEN逐行输出，连续空格压缩为一个数字
    for (int row = 0; row < Rows; ++row)
    {
        int empty = 0;
        for (int col = 0; col < Columns; ++col)
        {
            if (board[row][col] == '.')
            {
                ++empty;
                continue;
            }
            // 遇到棋子先输出前面连续空格的数量，再写棋子字符。
            if (empty)
            {
                result += char('0' + empty);
                empty = 0;
            }
            result += board[row][col];
        }
        // 一行末尾的空格也要输出。斜线在 FEN 中划分相邻两行。
        if (empty) result += char('0' + empty);
        if (row + 1 < Rows) result += '/';
    }
    result += redToMove ? " w - - " : " b - - ";
    //引擎需要行棋方和计数；完整历史在请求时另外追加
    result += std::to_string(halfmoves);
    result += ' ';
    result += std::to_string(fullmove);
}

} // namespace chess
