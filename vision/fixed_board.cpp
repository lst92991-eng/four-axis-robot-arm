#include "fixed_board.hpp"

namespace scara
{
//---------固定坐标初始化区---------
//棋盘四角和基座标定已经确认，逐格缓存毫米坐标
bool FixedBoard_setup(const PlaneMap &plane, const Board &grid, FixedBoard &fixed)
{
    FixedBoard prepared;
    int mappedCount = 0;
    prepared.grid = grid;
    for (int row = 0; row < Rows; ++row)
        for (int column = 0; column < Columns; ++column)
        {
            int index = row * Columns + column;
            prepared.mapped[index] = gridTarget(plane, grid, column, row, prepared.targets[index]);
            mappedCount += prepared.mapped[index];
        }
    //整体交付本次布局，未覆盖的格点保留不可用状态
    fixed = prepared;
    return mappedCount > 0;
}

//---------走棋目标读取区---------
//走法给出行列，读取初始化时保存的基座XY
bool FixedBoard_target(const FixedBoard &fixed, int column, int row, cv::Point2d &baseMm)
{
    baseMm = {};
    bool inside = column >= 0 && column < Columns && row >= 0 && row < Rows;
    if (!inside) return false;
    int index = row * Columns + column;
    if (!fixed.mapped[index]) return false;
    baseMm = fixed.targets[index];
    return true;
}
} // namespace scara
