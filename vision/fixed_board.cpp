#include "fixed_board.hpp"

namespace scara
{
bool FixedBoard_setup(const PlaneMap &plane, const Board &grid, FixedBoard &fixed)
{
    FixedBoard prepared;
    int mappedCount = 0;
    prepared.camera = plane.camera;
    prepared.grid = grid;
    for (int row = 0; row < Rows; ++row)
        for (int column = 0; column < Columns; ++column)
        {
            int index = row * Columns + column;
            prepared.mapped[index] = gridTarget(plane, grid, column, row, grid.frameId, prepared.targets[index]);
            mappedCount += prepared.mapped[index];
        }
    fixed = prepared;
    return mappedCount > 0;
}

void FixedBoard_assign(FixedBoard &fixed, const std::string &frameId, const std::vector<Detection> &pieces,
                       bool sceneClear, Observation &observation)
{
    fixed.grid.frameId = frameId;
    assignPieces(fixed.camera, fixed.grid, frameId, pieces, sceneClear, observation);
}

bool FixedBoard_target(const FixedBoard &fixed, int column, int row, const std::string &frameId, TargetXY &target)
{
    target = {};
    bool inside = column >= 0 && column < Columns && row >= 0 && row < Rows;
    if (!inside || frameId.empty() || frameId != fixed.grid.frameId) return false;
    int index = row * Columns + column;
    if (!fixed.mapped[index]) return false;
    target = fixed.targets[index];
    target.frameId = frameId;
    return true;
}
} // namespace scara
