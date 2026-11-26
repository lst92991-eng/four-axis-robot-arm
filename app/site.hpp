#pragma once
#include "game.hpp"
#include <string_view>

namespace scara
{
// 安装文件只在入口读取，运动和棋局使用解出的数值。
// 对局读取本机表；巡点和离线验证可以指定文件。
void Site_loadFile(StaticParams &staticParams, const std::string &path = {});
void Site_parse(std::string_view text, StaticParams &staticParams);
ArmGeometry Site_loadArm(const std::string &path);
// 独立视觉预览只读安装表的裁剪起点。
cv::Point Site_loadCrop();
}
