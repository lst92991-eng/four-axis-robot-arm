#include "vision_bridge.hpp"

namespace scara
{
void chessObservation(const Board &grid, const Observation &source, bool stable, chess::Observation &result)
{
    result = {};
    for (auto &row : result.board)
        row.fill('?');
    result.complete = grid.geometryValid && grid.orientationValid && source.valid && source.issues.empty() &&
                      !grid.frameId.empty() && grid.frameId == source.frameId;
    for (int rank = 0; rank < Rows; ++rank)
    {
        for (int col = 0; col < Columns; ++col)
        {
            const int piece = source.pieces[rank * Columns + col];
            char symbol = '?';
            if (piece == Empty)
                symbol = '.';
            else if (piece >= 0 && piece < int(sizeof(PieceSymbols) - 1))
                symbol = PieceSymbols[piece];
            else
                result.complete = false;
            result.board[Rows - 1 - rank][col] = symbol;
        }
    }
    std::string error;
    if (result.complete && !chess::validBoard(result.board, error)) result.complete = false;
    result.stable = stable && result.complete;
}

} // namespace scara
