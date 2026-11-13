#pragma once
#include "board.hpp"
#include "vision/geometry.hpp"

namespace scara
{
// Vision rank 0 is the red home rank; chess array row 0 is the black home rank.
// The caller supplies stability from distinct fresh frames, never from repeated reads.
void chessObservation(const Board &grid, const Observation &source, bool stable, chess::Observation &result);

} // namespace scara
