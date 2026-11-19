#pragma once
#include "game.hpp"

namespace scara
{
//保存预览和棋局状态，供页面读取
void Files_open(const std::string &directory, bool simulated);
//显示回调只更新界面原因，不控制机械动作
void Files_event(const AppEvent &event);
bool Runtime_publish(const std::string &name, const std::string &bytes);
} // namespace scara
