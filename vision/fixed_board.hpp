#pragma once
#include "geometry.hpp"

namespace scara
{
// 棋盘和基座固定后，保存一次网格与90个基座目标；每帧只更新棋子和帧号。
struct FixedBoard
{
    Camera camera;
    Board grid;
    std::array<TargetXY, Cells> targets{};
    std::array<bool, Cells> mapped{}; // 对应格点是否在已验证的标定范围内。
};

bool FixedBoard_setup(const PlaneMap &plane, const Board &grid, FixedBoard &fixed);
void FixedBoard_assign(FixedBoard &fixed, const std::string &frameId, const std::vector<Detection> &pieces,
                       bool sceneClear, Observation &observation);
bool FixedBoard_target(const FixedBoard &fixed, int column, int row, const std::string &frameId, TargetXY &target);
} // namespace scara
