#include "board.hpp"

#include <algorithm>
#include <charconv>
#include <sstream>

namespace chess {
namespace {
bool inside(int row, int col) { return row >= 0 && row < 10 && col >= 0 && col < 9; }
bool redPiece(char piece) { return piece >= 'A' && piece <= 'Z'; }
bool reject(std::string& error, const std::string& message) { error = message; return false; }

bool observed(const Observation& value, std::string& error)
{
    if (!value.complete || !value.stable || value.duplicate)
        return reject(error, "盘面不完整、不稳定或检测重复，等待重新识别");
    return validBoard(value.board, error);
}

bool legalList(const Board& board, bool redToMove,
               const std::vector<std::string>& legal, std::string& error)
{
    for (size_t i = 0; i < legal.size(); ++i) {
        Move move;
        Board next;
        if (!parseMove(legal[i], move) ||
            std::find(legal.begin(), legal.begin() + i, legal[i]) != legal.begin() + i)
            return reject(error, "合法走法列表格式错误或重复");
        if (!applyMove(board, redToMove, move, next, error)) return false;
    }
    return true;
}

bool number(const std::string& text, int minimum, int& value)
{
    if (text.empty()) return false;
    auto parsed = std::from_chars(text.data(), text.data() + text.size(), value);
    return parsed.ec == std::errc() && parsed.ptr == text.data() + text.size() && value >= minimum;
}
} // namespace

Board initialBoard()
{
    const char* rows[] = {"rnbakabnr", ".........", ".c.....c.", "p.p.p.p.p", ".........",
                          ".........", "P.P.P.P.P", ".C.....C.", ".........", "RNBAKABNR"};
    Board board;
    for (int row = 0; row < 10; ++row)
        for (int col = 0; col < 9; ++col) board[row][col] = rows[row][col];
    return board;
}

bool parseMove(const std::string& text, Move& move)
{
    move = {};
    if (text.size() != 4 || text[0] < 'a' || text[0] > 'i' ||
        text[2] < 'a' || text[2] > 'i' || text[1] < '0' || text[1] > '9' ||
        text[3] < '0' || text[3] > '9' || text.substr(0, 2) == text.substr(2, 2)) return false;
    move = {9 - (text[1] - '0'), text[0] - 'a', 9 - (text[3] - '0'), text[2] - 'a'};
    return true;
}

std::string uci(const Move& move)
{
    if (!inside(move.fromRow, move.fromCol) || !inside(move.toRow, move.toCol) ||
        (move.fromRow == move.toRow && move.fromCol == move.toCol)) return {};
    return {char('a' + move.fromCol), char('0' + 9 - move.fromRow),
            char('a' + move.toCol), char('0' + 9 - move.toRow)};
}

bool validBoard(const Board& board, std::string& error)
{
    error.clear();
    const std::string pieces = "KABNRCPkabnrcp";
    const int maximum[] = {1, 2, 2, 2, 2, 2, 5, 1, 2, 2, 2, 2, 2, 5};
    int counts[14] = {};
    for (int row = 0; row < 10; ++row) {
        for (int col = 0; col < 9; ++col) {
            const char piece = board[row][col];
            if (piece == '.') continue;
            size_t index = pieces.find(piece);
            if (index == std::string::npos) return reject(error, "棋盘存在未知或无效棋子，不能当作空位");
            if (++counts[index] > maximum[index]) return reject(error, "同类棋子数量超过初始数量");
            if ((piece == 'K' && (row < 7 || col < 3 || col > 5)) ||
                (piece == 'k' && (row > 2 || col < 3 || col > 5)))
                return reject(error, "将帅不在九宫内，请核对识别和棋盘方向");
        }
    }
    if (counts[0] != 1 || counts[7] != 1)
        return reject(error, "将帅缺失属于待核对盘面，不能据此宣布胜负");
    return true;
}

std::string fen(const Board& board, bool redToMove, int halfmoves, int fullmove)
{
    std::string error;
    if (!validBoard(board, error) || halfmoves < 0 || fullmove < 1) return {};
    std::string result;
    for (int row = 0; row < 10; ++row) {
        int empty = 0;
        for (int col = 0; col < 9; ++col) {
            if (board[row][col] == '.') { ++empty; continue; }
            if (empty) { result += char('0' + empty); empty = 0; }
            result += board[row][col];
        }
        if (empty) result += char('0' + empty);
        if (row != 9) result += '/';
    }
    return result + (redToMove ? " w - - " : " b - - ") +
           std::to_string(halfmoves) + " " + std::to_string(fullmove);
}

bool parseFen(const std::string& text, Board& board, bool& redToMove, std::string& error)
{
    std::istringstream input(text);
    std::string placement, side, castling, enPassant, half, full, extra;
    int halfmoves = 0, fullmove = 0;
    if (!(input >> placement >> side >> castling >> enPassant >> half >> full) || input >> extra ||
        (side != "w" && side != "b") || castling != "-" || enPassant != "-" ||
        !number(half, 0, halfmoves) || !number(full, 1, fullmove))
        return reject(error, "象棋FEN字段不合法");
    Board parsed;
    for (auto& row : parsed) row.fill('.');
    int row = 0, col = 0;
    for (char piece : placement) {
        if (piece == '/') {
            if (col != 9 || row == 9) return reject(error, "象棋FEN行列数不合法");
            ++row; col = 0;
        } else if (piece >= '1' && piece <= '9') {
            col += piece - '0';
            if (col > 9) return reject(error, "象棋FEN行列数不合法");
        } else {
            if (col >= 9) return reject(error, "象棋FEN行列数不合法");
            parsed[row][col++] = piece;
        }
    }
    if (row != 9 || col != 9) return reject(error, "象棋FEN必须是9列10行");
    if (!validBoard(parsed, error)) return false;
    board = parsed;
    redToMove = side == "w";
    return true;
}

bool applyMove(const Board& before, bool redToMove, const Move& move,
               Board& after, std::string& error)
{
    if (!validBoard(before, error)) return false;
    if (uci(move).empty()) return reject(error, "走法坐标不合法");
    const char piece = before[move.fromRow][move.fromCol];
    const char target = before[move.toRow][move.toCol];
    if (piece == '.' || redPiece(piece) != redToMove ||
        (target != '.' && redPiece(target) == redToMove))
        return reject(error, "走法起点、轮次或目标棋子不匹配");
    if (target == 'K' || target == 'k') return reject(error, "不能把拿走将帅当作合法终局");
    Board next = before;
    next[move.fromRow][move.fromCol] = '.';
    next[move.toRow][move.toCol] = piece;
    if (!validBoard(next, error)) return false;
    after = next;
    return true;
}

bool inferMove(const Board& before, const Observation& after, bool redToMove,
               const std::vector<std::string>& legal, Move& move, std::string& error)
{
    move = {};
    if (!observed(after, error) || !validBoard(before, error) ||
        !legalList(before, redToMove, legal, error)) return false;
    int matches = 0;
    Move candidate;
    for (const auto& text : legal) {
        Move current;
        Board predicted;
        if (!parseMove(text, current) || !applyMove(before, redToMove, current, predicted, error)) return false;
        if (predicted == after.board) { candidate = current; ++matches; }
    }
    if (matches != 1) return reject(error, matches == 0 ? "盘面未对应唯一合法走法，请重新观察" : "多种走法匹配，等待人工核对");
    move = candidate;
    return true;
}

Game::Game() { reset(initialBoard()); }
bool Game::fail(const std::string& message) { error = message; return false; }

bool Game::reset(const Board& board, bool redToMove, bool humanRed)
{
    if (!validBoard(board, error)) return false;
    board_ = expected_ = board;
    redToMove_ = redToMove; humanRed_ = humanRed;
    result_ = Result::Playing; pending_ = false; pendingMove_ = {};
    baseFen_ = fen(board_, redToMove_); history_.clear(); moveCount_ = 0;
    positions_ = {baseFen_};
    return true;
}

bool Game::canMove()
{
    error.clear();
    if (result_ != Result::Playing) return fail("棋局已结束或等待人工裁定");
    if (pending_) return fail("机械走法尚未确认，不能接受下一步");
    if (moveCount_ >= 1024) return fail("达到历史长度上限，请人工处理");
    return true;
}

void Game::commit(const Board& after, const Move& move)
{
    board_ = after;
    history_ += (history_.empty() ? "" : " ") + uci(move);
    redToMove_ = !redToMove_; ++moveCount_;
    positions_.push_back(fen(board_, redToMove_));
    pending_ = false;
    error.clear();
}

bool Game::acceptHuman(const Observation& after, const std::vector<std::string>& legal, Move& move)
{
    move = {};
    if (!canMove()) return false;
    if (redToMove_ != humanRed_) return fail("当前是机械臂一方的回合");
    if (!inferMove(board_, after, redToMove_, legal, move, error)) return false;
    commit(after.board, move);
    return true;
}

bool Game::prepareRobot(const Observation& before, const std::string& choice,
                        const std::vector<std::string>& legal, Move& move)
{
    move = {};
    if (!canMove()) return false;
    if (redToMove_ == humanRed_) return fail("当前是人类一方的回合");
    if (!observed(before, error)) return false;
    if (before.board != board_) return fail("执行前盘面已变化，取消本次动作");
    if (!legalList(board_, redToMove_, legal, error)) return false;
    if (std::find(legal.begin(), legal.end(), choice) == legal.end() || !parseMove(choice, move))
        return fail("引擎选择不在当前合法走法表内");
    if (!applyMove(board_, redToMove_, move, expected_, error)) return false;
    pendingMove_ = move;
    pending_ = true;
    return true;
}

bool Game::confirmRobot(const Observation& after, bool armSucceeded)
{
    error.clear();
    if (!pending_) return fail("没有待确认的机械走法");
    // 失败时保留待确认状态，禁止不清楚棋子位置就重发搬运或切换回合。
    if (!armSucceeded) return fail("机械执行未成功，棋局不提交，请停机核对");
    if (!observed(after, error)) return false;
    if (after.board != expected_) return fail("执行后盘面与计划不一致，棋局不提交");
    commit(after.board, pendingMove_);
    return true;
}

bool Game::judge(const std::vector<std::string>& legal)
{
    error.clear();
    if (pending_) return fail("必须先确认机械动作，再判断下一回合");
    if (result_ != Result::Playing) return fail("棋局已结束或等待人工裁定");
    if (!legalList(board_, redToMove_, legal, error)) return false;
    if (legal.empty()) result_ = redToMove_ ? Result::BlackWon : Result::RedWon;
    else if (std::count(positions_.begin(), positions_.end(), positions_.back()) >= 3 || moveCount_ >= 1024) {
        result_ = Result::ReviewRequired;
        error = "重复局面或历史达到上限，暂停人工裁定；未实现完整长将追逐裁判";
    }
    return true;
}

bool Game::agreeDraw()
{
    error.clear();
    if (pending_ || (result_ != Result::Playing && result_ != Result::ReviewRequired))
        return fail("机械动作未确认或棋局已经结束");
    result_ = Result::Draw;
    return true;
}

bool Game::resign(bool red)
{
    error.clear();
    if (pending_ || (result_ != Result::Playing && result_ != Result::ReviewRequired))
        return fail("机械动作未确认或棋局已经结束");
    result_ = red ? Result::BlackWon : Result::RedWon;
    return true;
}
} // namespace chess
