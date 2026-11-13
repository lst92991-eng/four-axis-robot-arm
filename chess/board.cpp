#include "board.hpp"

#include <algorithm>

namespace chess
{
namespace
{
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
    std::string moveText;
    if (uci(move, moveText) == false) return reject(error, "走法坐标不合法");
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

bool observed(const Observation &value, std::string &error)
{
    if (!value.valid) return reject(error, "盘面不完整、不稳定或检测重复，等待重新识别");
    return validBoard(value.board, error);
}

} // namespace

void initialBoard(Board &board)
{
    const char *rows[] = {"rnbakabnr", ".........", ".c.....c.", "p.p.p.p.p", ".........",
                          ".........", "P.P.P.P.P", ".C.....C.", ".........", "RNBAKABNR"};
    for (int row = 0; row < 10; ++row)
        for (int col = 0; col < 9; ++col)
            board[row][col] = rows[row][col];
}

bool parseMove(const std::string &text, Move &move)
{
    bool filesValid, ranksValid;

    move = {};
    if (text.size() != 4) return false;
    filesValid = text[0] >= 'a' && text[0] <= 'i' && text[2] >= 'a' && text[2] <= 'i';
    ranksValid = text[1] >= '0' && text[1] <= '9' && text[3] >= '0' && text[3] <= '9';
    if (!filesValid || !ranksValid || text.substr(0, 2) == text.substr(2, 2)) return false;

    // UCI从红方底线记作0；棋局数组从黑方底线记作第0行。
    move.fromRow = 9 - (text[1] - '0');
    move.fromCol = text[0] - 'a';
    move.toRow = 9 - (text[3] - '0');
    move.toCol = text[2] - 'a';
    return true;
}

bool uci(const Move &move, std::string &text)
{
    text.clear();
    if (!inside(move.fromRow, move.fromCol) || !inside(move.toRow, move.toCol) ||
        (move.fromRow == move.toRow && move.fromCol == move.toCol))
        return false;
    text += char('a' + move.fromCol);
    text += char('0' + 9 - move.fromRow);
    text += char('a' + move.toCol);
    text += char('0' + 9 - move.toRow);
    return true;
}

bool validBoard(const Board &board, std::string &error)
{
    error.clear();
    const std::string pieces = "KABNRCPkabnrcp";
    const int maximum[] = {1, 2, 2, 2, 2, 2, 5, 1, 2, 2, 2, 2, 2, 5};
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

void fen(const Board &board, bool redToMove, int halfmoves, int fullmove, std::string &result)
{
    result.clear();
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
    result += std::to_string(halfmoves);
    result += ' ';
    result += std::to_string(fullmove);
}

bool applyMove(const Board &before, bool redToMove, const Move &move, Board &after, std::string &error)
{
    if (!validBoard(before, error)) return false;
    return applyKnownBoard(before, redToMove, move, after, error);
}

bool inferMove(const Board &before, const Observation &after, bool redToMove, const std::vector<std::string> &legal,
               Move &move, std::string &error)
{
    int matches = 0;
    Move candidate;

    move = {};
    if (!observed(after, error) || !validBoard(before, error)) return false;
    for (size_t index = 0; index < legal.size(); ++index)
    {
        Move current;
        Board predicted;
        bool parsed = parseMove(legal[index], current);
        // 只查当前走法之前的部分，防止同一走法被重复计数。
        auto currentEntry = legal.begin() + index;
        bool duplicate = std::find(legal.begin(), currentEntry, legal[index]) != currentEntry;
        if (parsed == false || duplicate) return reject(error, "合法走法列表格式错误或重复");
        if (!applyKnownBoard(before, redToMove, current, predicted, error)) return false;
        if (predicted == after.board)
        {
            candidate = current;
            ++matches;
        }
    }
    if (matches != 1)
        return reject(error, matches == 0 ? "盘面未对应唯一合法走法，请重新观察" : "多种走法匹配，等待人工核对");
    move = candidate;
    return true;
}

Game::Game()
{
    Board initial;
    initialBoard(initial);
    Game_reset(*this, initial);
}

bool Game_reset(Game &game, const Board &board, bool redToMove, bool humanRed)
{
    if (!validBoard(board, game.error)) return false;
    game.board = board;
    game.expected = board;
    game.redToMove = redToMove;
    game.humanRed = humanRed;
    game.result = Result::Playing;
    game.pending = false;
    game.pendingMove = {};
    fen(game.board, game.redToMove, 0, 1, game.baseFen);
    game.history.clear();
    game.moveCount = 0;
    game.positions = {game.baseFen};
    return true;
}

static bool canMove(Game &game)
{
    game.error.clear();
    if (game.result != Result::Playing) return reject(game.error, "棋局已结束或等待人工裁定");
    if (game.pending) return reject(game.error, "机械走法尚未确认，不能接受下一步");
    if (game.moveCount >= 1024) return reject(game.error, "达到历史长度上限，请人工处理");
    return true;
}

static void commit(Game &game, const Board &after, const Move &move)
{
    std::string moveText, position;

    game.board = after;
    uci(move, moveText);
    if (!game.history.empty()) game.history += ' ';
    game.history += moveText;
    game.redToMove = !game.redToMove;
    ++game.moveCount;
    fen(game.board, game.redToMove, 0, 1, position);
    game.positions.push_back(position);
    game.pending = false;
    game.error.clear();
}

bool Game_acceptHuman(Game &game, const Observation &after, const std::vector<std::string> &legal, Move &move)
{
    move = {};
    if (!canMove(game)) return false;
    if (game.redToMove != game.humanRed) return reject(game.error, "当前是机械臂一方的回合");
    if (!inferMove(game.board, after, game.redToMove, legal, move, game.error)) return false;
    commit(game, after.board, move);
    return true;
}

bool Game_prepareRobot(Game &game, const Observation &before, const std::string &choice,
                       const std::vector<std::string> &legal, Move &move)
{
    move = {};
    if (!canMove(game)) return false;
    if (game.redToMove == game.humanRed) return reject(game.error, "当前是人类一方的回合");
    if (!observed(before, game.error)) return false;
    if (before.board != game.board) return reject(game.error, "执行前盘面已变化，取消本次动作");
    if (std::find(legal.begin(), legal.end(), choice) == legal.end() || !parseMove(choice, move))
        return reject(game.error, "引擎选择不在当前合法走法表内");
    if (!applyKnownBoard(game.board, game.redToMove, move, game.expected, game.error)) return false;
    game.pendingMove = move;
    game.pending = true;
    return true;
}

bool Game_confirmRobot(Game &game, const Observation &after, bool armSucceeded)
{
    game.error.clear();
    if (!game.pending) return reject(game.error, "没有待确认的机械走法");
    // 失败时保留待确认状态，禁止不清楚棋子位置就重发搬运或切换回合。
    if (!armSucceeded) return reject(game.error, "机械执行未成功，棋局不提交，请停机核对");
    if (!observed(after, game.error)) return false;
    if (after.board != game.expected) return reject(game.error, "执行后盘面与计划不一致，棋局不提交");
    commit(game, after.board, game.pendingMove);
    return true;
}

bool Game_judge(Game &game, const std::vector<std::string> &legal)
{
    game.error.clear();
    if (game.pending) return reject(game.error, "必须先确认机械动作，再判断下一回合");
    if (game.result != Result::Playing) return reject(game.error, "棋局已结束或等待人工裁定");
    if (legal.empty())
        game.result = game.redToMove ? Result::BlackWon : Result::RedWon;
    else if (std::count(game.positions.begin(), game.positions.end(), game.positions.back()) >= 3 ||
             game.moveCount >= 1024)
    {
        game.result = Result::ReviewRequired;
        game.error = "重复局面或历史达到上限，暂停人工裁定；未实现完整长将追逐裁判";
    }
    return true;
}

bool Game_finish(Game &game, Result outcome)
{
    game.error.clear();
    if (game.pending || (game.result != Result::Playing && game.result != Result::ReviewRequired))
        return reject(game.error, "机械动作未确认或棋局已经结束");
    if (outcome != Result::RedWon && outcome != Result::BlackWon && outcome != Result::Draw)
        return reject(game.error, "需要明确的胜负或和棋结果");
    game.result = outcome;
    return true;
}

} // namespace chess
