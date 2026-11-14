#pragma once
#include "game.hpp"

namespace scara
{
// 仅保存预览和棋局状态，机械执行不经过文件。
void Files_open(const std::string &directory, bool simulated);
std::string Runtime_path(const std::string &name);
bool Runtime_publish(const std::string &name, const std::string &bytes);
} // namespace scara
