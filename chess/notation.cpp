// 走法文本和起始局面采用皮卡鱼的 UCI、FEN 格式。
#include "board.hpp"

namespace chess
{
//---------引擎和棋谱的格式转换区---------
//输入四字符UCI走法，输出黑方底线起算的数组行列
bool parseMove(std::string_view text, Move &move)
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
std::string uci(const Move &move)
{
    if (!validSquares(move)) return {};
    return {char('a' + move.fromCol), char('0' + Rows - 1 - move.fromRow),
            char('a' + move.toCol), char('0' + Rows - 1 - move.toRow)};
}

// 开局 FEN 只记录起始盘面和行棋方，随后由 UCI moves 交付棋谱。
std::string startFen(const Board &board, bool redToMove)
{
    std::string result;
    //FEN逐行输出，连续空格压缩为一个数字
    for (const auto &rank : board)
    {
        if (!result.empty()) result += '/';
        int empty = 0;
        for (char piece : rank)
        {
            if (piece == '.')
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
            result += piece;
        }
        // 写出这一行末尾连续的空格数。
        if (empty) result += char('0' + empty);
    }
    result += redToMove ? " w - - 0 1" : " b - - 0 1";
    return result;
}

} // namespace chess
