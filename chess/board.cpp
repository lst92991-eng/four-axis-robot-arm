#include "board.hpp"

#include <algorithm>

namespace chess
{
//---------初始化区---------
//填写标准开局，行0为黑方底线
void initialBoard(Board &board)
{
    const char *rows[] = {"rnbakabnr", ".........", ".c.....c.", "p.p.p.p.p", ".........",
                          ".........", "P.P.P.P.P", ".C.....C.", ".........", "RNBAKABNR"};
    //逐格复制字符，后续识别和棋局都使用同一个10×9数组
    for (int row = 0; row < 10; ++row)
        for (int col = 0; col < 9; ++col)
            board[row][col] = rows[row][col];
}

//默认构造即建立标准局面，字段初值保存其余回合状态
Game::Game()
{
    initialBoard(board);
    fen(board, redToMove, 0, 1, baseFen);
    positions = {baseFen};
}

//指定合法开局及双方身份，清除上一局的历史和待执行状态
bool Game_reset(Game &game, const Board &board, bool redToMove, bool humanRed)
{
    if (!validBoard(board, game.error)) return false;
    //局部对象使用默认回合状态，开局准备完成后统一替换
    Game prepared;
    //先复制调用者的盘面，支持把game.board自身作为新的开局输入
    prepared.board = board;
    prepared.redToMove = redToMove;
    prepared.humanRed = humanRed;
    fen(board, redToMove, 0, 1, prepared.baseFen);
    prepared.positions = {prepared.baseFen};
    game = prepared;
    return true;
}

//---------搬子数据核对区---------
namespace
{
//数组边界与棋子颜色在搬子、格式转换处共用
bool inside(int row, int col)
{
    return row >= 0 && row < 10 && col >= 0 && col < 9;
}
bool redPiece(char piece)
{
    return piece >= 'A' && piece <= 'Z';
}
bool reject(std::string &error, const std::string &message)
{
    error = message;
    return false;
}

bool kingInPalace(char piece, int row, int col)
{
    bool column = col >= 3 && col <= 5;
    if (piece == 'K') return column && row >= 7;
    if (piece == 'k') return column && row <= 2;
    return true;
}

// 盘面在入口已验证；一次搬子只需检查本次起点、终点与将帅位置。
bool applyKnownBoard(const Board &before, bool redToMove, const Move &move, Board &after, std::string &error)
{
    char piece = before[move.fromRow][move.fromCol];
    char target = before[move.toRow][move.toCol];
    if (piece == '.' || redPiece(piece) != redToMove || (target != '.' && redPiece(target) == redToMove))
        return reject(error, "走法起点、轮次或目标棋子不匹配");
    if (target == 'K' || target == 'k') return reject(error, "不能把拿走将帅当作合法终局");
    if (kingInPalace(piece, move.toRow, move.toCol) == false)
        return reject(error, "将帅不在九宫内，请核对识别和棋盘方向");
    after = before;
    after[move.fromRow][move.fromCol] = '.';
    after[move.toRow][move.toCol] = piece;
    return true;
}

} // namespace

//---------走法与盘面转换区---------
//输入四字符UCI走法，输出黑方底线起算的数组行列
bool parseMove(const std::string &text, Move &move)
{
    bool filesValid, ranksValid;

    move = {};
    if (text.size() != 4) return false;
    filesValid = text[0] >= 'a' && text[0] <= 'i' && text[2] >= 'a' && text[2] <= 'i';
    ranksValid = text[1] >= '0' && text[1] <= '9' && text[3] >= '0' && text[3] <= '9';
    if (!filesValid || !ranksValid || (text[0] == text[2] && text[1] == text[3])) return false;

    // UCI从红方底线记作0；棋局数组从黑方底线记作第0行。
    move.fromRow = 9 - (text[1] - '0');
    move.fromCol = text[0] - 'a';
    move.toRow = 9 - (text[3] - '0');
    move.toCol = text[2] - 'a';
    return true;
}

//输入起点终点行列，输出引擎使用的四字符走法
bool uci(const Move &move, std::string &text)
{
    text.clear();
    //格式转换入口统一核对下标，随后才能访问棋盘数组
    if (!inside(move.fromRow, move.fromCol) || !inside(move.toRow, move.toCol) ||
        (move.fromRow == move.toRow && move.fromCol == move.toCol))
        return false;
    text += char('a' + move.fromCol);
    text += char('0' + 9 - move.fromRow);
    text += char('a' + move.toCol);
    text += char('0' + 9 - move.toRow);
    return true;
}

//统计14类棋子并核对将帅，识别结果在进入棋局时调用
bool validBoard(const Board &board, std::string &error)
{
    error.clear();
    const std::string pieces = "KABNRCPkabnrcp";
    const int maximum[] = {1, 2, 2, 2, 2, 2, 5, 1, 2, 2, 2, 2, 2, 5};
    //计数顺序与pieces字符串一致，黑红双方分别核对
    int counts[14] = {};
    for (int row = 0; row < 10; ++row)
        for (int col = 0; col < 9; ++col)
        {
            const char piece = board[row][col];
            if (piece == '.') continue;
            size_t index = pieces.find(piece);
            if (index == std::string::npos) return reject(error, "棋盘存在未知或无效棋子，不能当作空位");
            ++counts[index];
            if (counts[index] > maximum[index]) return reject(error, "同类棋子数量超过初始数量");
            if (kingInPalace(piece, row, col) == false) return reject(error, "将帅不在九宫内，请核对识别和棋盘方向");
        }
    if (counts[0] != 1 || counts[7] != 1) return reject(error, "将帅缺失属于待核对盘面，不能据此宣布胜负");
    return true;
}

//将已确认的90格压缩成FEN，附上行棋方与回合计数
void fen(const Board &board, bool redToMove, int halfmoves, int fullmove, std::string &result)
{
    result.clear();
    //FEN逐行输出，连续空格压缩为一个数字
    for (int row = 0; row < 10; ++row)
    {
        int empty = 0;
        for (int col = 0; col < 9; ++col)
        {
            if (board[row][col] == '.')
            {
                ++empty;
                continue;
            }
            if (empty)
            {
                result += char('0' + empty);
                empty = 0;
            }
            result += board[row][col];
        }
        if (empty) result += char('0' + empty);
        if (row != 9) result += '/';
    }
    result += redToMove ? " w - - " : " b - - ";
    //引擎需要行棋方和计数；完整历史在请求时另外追加
    result += std::to_string(halfmoves);
    result += ' ';
    result += std::to_string(fullmove);
}

//对外的搬子入口核对盘面和坐标，输出移动后的90格
bool applyMove(const Board &before, bool redToMove, const Move &move, Board &after, std::string &error)
{
    std::string moveText;
    if (!validBoard(before, error)) return false;
    if (!uci(move, moveText)) return reject(error, "走法坐标不合法");
    return applyKnownBoard(before, redToMove, move, after, error);
}

//从本次盘面变化直接得到起点和终点，再与引擎合法表核对
bool inferMove(const Board &before, const Observation &after, bool redToMove, const std::vector<std::string> &legal,
               Move &move, std::string &error)
{
    move = {};
    if (!after.valid) return reject(error, "盘面不完整、不稳定或检测重复，等待重新识别");
    Move candidate;
    //起点变空，终点变成移动的棋子；最终整盘比较排除额外改动
    for (int row = 0; row < 10; ++row)
        for (int col = 0; col < 9; ++col)
            if (before[row][col] != after.board[row][col])
            {
                if (after.board[row][col] == '.')
                {
                    candidate.fromRow = row;
                    candidate.fromCol = col;
                }
                else
                {
                    candidate.toRow = row;
                    candidate.toCol = col;
                }
            }
    std::string text;
    //合法表已由引擎接口解析；本次识别结果必须恰好对应其中一条
    if (!uci(candidate, text) || std::count(legal.begin(), legal.end(), text) != 1)
        return reject(error, "盘面未对应唯一合法走法，请重新观察");
    //只推演这一条走法，确认90格的结果完整一致
    Board expected;
    if (!applyKnownBoard(before, redToMove, candidate, expected, error)) return false;
    if (expected != after.board) return reject(error, "盘面未对应唯一合法走法，请重新观察");
    move = candidate;
    return true;
}

//---------回合交接区---------
//落子已经确认后，更新权威盘面、完整历史和重复局面记录
void Game_commit(Game &game, const Board &after, const Move &move)
{
    std::string moveText, position;

    game.board = after;
    //追加已确认的一步，再交换行棋方，供下一次引擎查询使用
    uci(move, moveText);
    if (!game.history.empty()) game.history += ' ';
    game.history += moveText;
    game.redToMove = !game.redToMove;
    fen(game.board, game.redToMove, 0, 1, position);
    game.positions.push_back(position);
    //提交完成后清除上一手的棋局提示
    game.error.clear();
}

//合法表为空时本方负，重复局面交人工裁定
bool Game_judge(Game &game, const std::vector<std::string> &legal)
{
    game.error.clear();
    if (game.result != Result::Playing) return reject(game.error, "棋局已结束或等待人工裁定");
    if (legal.empty())
        game.result = game.redToMove ? Result::BlackWon : Result::RedWon;
    else if (std::count(game.positions.begin(), game.positions.end(), game.positions.back()) >= 3 ||
             game.positions.size() > 1024)
    {
        game.result = Result::ReviewRequired;
        game.error = "重复局面或历史达到上限，暂停人工裁定；未实现完整长将追逐裁判";
    }
    return true;
}

//将认输或双方同意的和棋写入结果，保留已完成历史
bool Game_finish(Game &game, Result outcome)
{
    game.error.clear();
    if (game.result != Result::Playing && game.result != Result::ReviewRequired)
        return reject(game.error, "棋局已经结束");
    if (outcome != Result::RedWon && outcome != Result::BlackWon && outcome != Result::Draw)
        return reject(game.error, "需要明确的胜负或和棋结果");
    game.result = outcome;
    return true;
}

} // namespace chess
