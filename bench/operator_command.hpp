#pragma once
#include "app/game.hpp"

namespace scara
{
// 在台架和仿真入口把输入文本转换成命令及数字行列。
inline Command Command_parse(const std::string &text, chess::Move *move)
{
    if (move) *move = {};
    if (text.empty() || text == "ready") return Command::Ready;
    if (text == "resign") return Command::Resign;
    if (text == "draw") return Command::Draw;
    if (text == "quit") return Command::Quit;
    if (!move) return Command::Invalid;

    if (text.size() == 7 && text[1] == ',' && text[3] == ' ' && text[5] == ',')
    {
        chess::Move selected{text[0] - '0', text[2] - '0', text[4] - '0', text[6] - '0'};
        if (!chess::validSquares(selected)) return Command::Invalid;
        *move = selected;
        return Command::Move;
    }
    return chess::parseMove(text, *move) ? Command::Move : Command::Invalid;
}
}
