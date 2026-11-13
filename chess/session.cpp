#include "session.hpp"
#include <utility>

namespace scara
{
void chessObservation(const Observation &source, chess::Observation &result)
{
    result = {};
    result.valid = source.valid && !source.frameId.empty();
    for (int rank = 0; rank < Rows; ++rank)
        for (int col = 0; col < Columns; ++col)
        {
            const int piece = source.pieces[rank * Columns + col];
            char symbol = '?';
            if (piece == Empty)
                symbol = '.';
            else if (piece >= 0 && piece < int(sizeof(PieceSymbols) - 1))
                symbol = PieceSymbols[piece];
            else
                result.valid = false;
            result.board[Rows - 1 - rank][col] = symbol;
        }
    std::string error;
    if (result.valid && !chess::validBoard(result.board, error)) result.valid = false;
}

void Session_observe(Session &session, const std::string &frameId, const std::vector<Detection> &pieces,
                     bool sceneClear)
{
    session.lastFrameId = frameId;
    Observation current, stable;
    assignPieces(session.layout.grid, frameId, pieces, sceneClear, current);
    session.stable.update(current, stable);
    chessObservation(stable, session.observed);
}

bool Session_plan(Session &session, const std::string &choice, const std::vector<std::string> &legal)
{
    chess::Move move;
    cv::Point2d source, destination;        // 本次起点和终点，基座毫米坐标。
    std::vector<PickStep> prepared, moving; // 先清走被吃棋子，再搬自己的棋子。
    bool ready = chess::parseMove(choice, move);
    if (ready)
    {
        bool sourceMapped = FixedBoard_target(session.layout, move.fromCol, Rows - 1 - move.fromRow, source);
        bool destinationMapped = FixedBoard_target(session.layout, move.toCol, Rows - 1 - move.toRow, destination);
        ready = sourceMapped && destinationMapped;
        bool capture = session.game.board[move.toRow][move.toCol] != '.';
        if (ready && capture)
            ready = session.capturePlaceTaught &&
                    Pick_plan(session.arm, session.heights, destination, session.capturePlaceMm, false, prepared);
        if (ready) ready = Pick_plan(session.arm, session.heights, source, destination, false, moving);
    }
    if (!ready)
    {
        session.game.error = "走法坐标、可达范围或吃子区尚未准备好";
        return false;
    }
    // 全部计算完成后才锁定走法；失败不会覆盖原有棋局、历史或待执行步骤。
    prepared.insert(prepared.end(), moving.begin(), moving.end());
    if (!chess::Game_prepareRobot(session.game, session.observed, choice, legal, move)) return false;
    session.steps = std::move(prepared);
    return true;
}

bool Session_confirm(Session &session, bool armSucceeded)
{
    bool confirmed = chess::Game_confirmRobot(session.game, session.observed, armSucceeded);
    if (confirmed) session.steps.clear();
    return confirmed;
}

bool Session_restore(Session &session)
{
    if (!session.game.pending || !session.observed.valid || session.observed.board != session.game.board)
    {
        session.game.error = "复位盘面还未与上一步已确认盘面一致";
        return false;
    }
    session.game.pending = false;
    session.game.pendingMove = {};
    session.game.expected = session.game.board;
    session.game.error.clear();
    session.steps.clear();
    session.stable.reset();
    return true;
}
} // namespace scara
