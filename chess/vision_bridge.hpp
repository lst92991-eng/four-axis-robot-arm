#pragma once
#include "board.hpp"
#include "vision/geometry.hpp"

namespace scara
{
// Vision rank 0 is the red home rank; chess array row 0 is the black home rank.
// The caller supplies stability from distinct fresh frames, never from repeated reads.
void chessObservation(const Board &grid, const Observation &source, bool stable, chess::Observation &result);

struct ChessMoveTargets
{
    TargetXY from, to;
};
// A pure coordinate conversion, not legal-move validation or a motor command.
// Check against the engine's legal list; finish geometry/IK checks before locking
// the turn with Game::prepareRobot and passing it to a separate motion executor.
bool chessMoveTargets(const PlaneMap &plane, const Board &grid, const chess::Move &move,
                      const std::string &currentFrameId, ChessMoveTargets &targets);
} // namespace scara
