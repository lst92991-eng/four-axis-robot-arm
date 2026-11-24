#pragma once
#include "game.hpp"
#include <string_view>

namespace scara
{
// 安装文件只在入口读取，运动和棋局使用解出的数值。
std::string Site_path();
void Site_loadFile(const std::string &path, GameConfig &config, Session &session);
void Site_parse(std::string_view text, GameConfig &config, Session &session);
void Site_loadArm(const std::string &path, ArmGeometry &arm);
}
