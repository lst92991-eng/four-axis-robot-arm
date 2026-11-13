#include "session.hpp"
#include "vision_bridge.hpp"

namespace scara
{
void Session_observe(Session &session, const std::string &frameId, const std::vector<Detection> &pieces,
                     bool sceneClear)
{
    Observation current, stable;
    FixedBoard_assign(session.layout, frameId, pieces, sceneClear, current);
    session.stable.update(session.layout.grid, current, stable);
    chessObservation(session.layout.grid, stable, stable.valid, session.observed);
}

bool Session_acceptHuman(Session &session, const std::vector<std::string> &legal, chess::Move &move)
{
    return chess::Game_acceptHuman(session.game, session.observed, legal, move);
}

bool Session_plan(Session &session, const std::string &choice, const std::vector<std::string> &legal)
{
    chess::Game preparedGame = session.game;
    chess::Move move;
    TargetXY source, destination;
    std::vector<PickStep> prepared, moving;
    bool ready = chess::Game_prepareRobot(preparedGame, session.observed, choice, legal, move);
    if (ready)
    {
        const std::string &frame = session.layout.grid.frameId;
        bool sourceMapped = FixedBoard_target(session.layout, move.fromCol, Rows - 1 - move.fromRow, frame, source);
        bool destinationMapped =
            FixedBoard_target(session.layout, move.toCol, Rows - 1 - move.toRow, frame, destination);
        ready = sourceMapped && destinationMapped;
        bool capture = session.game.board[move.toRow][move.toCol] != '.';
        if (ready && capture)
            ready = session.capturePlaceTaught && Pick_plan(session.arm, session.heights, destination.baseMm,
                                                            session.capturePlaceMm, false, prepared);
        if (ready) ready = Pick_plan(session.arm, session.heights, source.baseMm, destination.baseMm, false, moving);
    }
    if (ready)
    {
        prepared.insert(prepared.end(), moving.begin(), moving.end());
        session.steps = prepared;
        session.game = preparedGame;
    }
    else
        session.game.error = preparedGame.error.empty() ? "取放坐标、可达范围或吃子区尚未准备好" : preparedGame.error;
    return ready;
}

bool Session_confirm(Session &session, bool armSucceeded)
{
    bool confirmed = chess::Game_confirmRobot(session.game, session.observed, armSucceeded);
    if (confirmed) session.steps.clear();
    return confirmed;
}
} // namespace scara
