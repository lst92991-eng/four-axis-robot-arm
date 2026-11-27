// 走法文本和起始局面采用皮卡鱼的 UCI、FEN 格式。
#include "board.hpp"

namespace chess
{
//---------引擎和棋谱的格式转换区---------
// 输入四字符 UCI 走法，输出黑方底线起算的数组行列。
bool parseMove(std::string_view text, Move &move)
{
    if (text.size() == 4)
    {
        // UCI 从红方底线记作 0；棋局数组从黑方底线记作行 0。
        move.fromRow = Rows - 1 - (text[1] - '0');
        move.fromCol = text[0] - 'a';
        move.toRow = Rows - 1 - (text[3] - '0');
        move.toCol = text[2] - 'a';
        if (validSquares(move)) return true;
    }
    move = {};
    return false;
}

// 输入起点终点行列，展开成引擎棋谱的两对列、行字符。
std::string uci(const Move &move)
{
    if (!validSquares(move)) return {};
    std::string text;
    text += char('a' + move.fromCol);
    text += char('0' + Rows - 1 - move.fromRow);
    text += char('a' + move.toCol);
    text += char('0' + Rows - 1 - move.toRow);
    return text;
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
