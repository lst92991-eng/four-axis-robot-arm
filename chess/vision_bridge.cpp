#include "vision_bridge.hpp"

namespace scara {
chess::Observation chessObservation(const Board& grid, const Observation& source, bool stable)
{
    chess::Observation result;
    for (auto& row : result.board) row.fill('?');
    result.complete = grid.geometryValid && grid.orientationValid && source.valid &&
        source.issues.empty() && !grid.frameId.empty() && grid.frameId == source.frameId &&
        !grid.cameraId.empty() && grid.cameraId == source.cameraId;
    for (int rank = 0; rank < Rows; ++rank) {
        for (int col = 0; col < Columns; ++col) {
            const int piece = source.pieces[rank * Columns + col];
            char symbol = '?';
            if (piece == Empty) symbol = '.';
            else if (piece >= 0 && piece < int(sizeof(PieceSymbols) - 1)) symbol = PieceSymbols[piece];
            else result.complete = false;
            result.board[Rows - 1 - rank][col] = symbol;
        }
    }
    std::string error;
    if (result.complete && !chess::validBoard(result.board, error)) result.complete = false;
    result.stable = stable && result.complete;
    return result;
}

std::optional<ChessMoveTargets> chessMoveTargets(
    const PlaneMap& plane, const Board& grid, const chess::Move& move,
    const std::string& currentFrameId)
{
    if (chess::uci(move).empty()) return {};
    auto from = gridTarget(plane, grid, move.fromCol, Rows - 1 - move.fromRow, currentFrameId);
    auto to = gridTarget(plane, grid, move.toCol, Rows - 1 - move.toRow, currentFrameId);
    if (!from || !to) return {};
    return ChessMoveTargets{*from, *to};
}
}
