// 相机确认一张实盘。和已记谱盘面或预期落点不同，就指出格号让操作者处理。
#include "game.hpp"

namespace scara
{
// 把需要核对的格号写进提示，保留相机实际交付的盘面。
static void reportBoard(const Session &session, const chess::Board *expected, std::string message)
{
    if (expected)
        for (int row = 0; row < Rows; ++row)
            for (int col = 0; col < Columns; ++col)
                if ((*expected)[row][col] != session.observed[row][col])
                    message += " (" + std::to_string(row) + "," + std::to_string(col) + ")";
    Operator_show(session, message);
}

//---------观察与恢复区---------
bool Observe_board(Session &session, const chess::Board *expected)
{
    int same = 0;
    bool warned = false; // 同一处遮挡只提示一次，格子变化后再更新。
    session.observationStable = false;
    Operator_show(session);
    while (gameRunning && same < 3)
    {
        std::string frameId;
        std::vector<Detection> pieces;
        Vision_observe(session.layout, frameId, pieces);
        if (!gameRunning) return false;
        BoardObservation frame = Geometry_assign(session.layout, pieces, !frameId.empty());
        if (frame.valid && same > 0 && frameId == session.lastFrameId) continue;
        session.lastFrameId = frameId;
        if (frame.cells != session.observed)
        {
            same = 0;
            warned = false;
        }
        session.observed = frame.cells;
        if (frame.valid) ++same;
        else
        {
            same = 0;
            if (!warned) reportBoard(session, expected, "棋子偏离格点或受遮挡，请检查实盘。");
            warned = true;
        }
    }
    if (!gameRunning) return false;
    session.observationStable = true;
    Operator_show(session);
    return true;
}

bool Observe_placement(Session &session, const chess::Board &expected)
{
    if (!Observe_board(session, &expected)) return false;
    while (session.observed != expected)
    {
        reportBoard(session, &expected, "请核对落点");
        if (!Operator_waitReady("修正落点后回车。若摆回走棋前的局面，回车后重走。"))
            return false;
        if (!Observe_board(session, &expected)) return false;
        if (session.observed == session.game.board) return false;
    }
    return true;
}

// 实盘恢复到已记谱的棋局，才能继续操作。
bool Observe_original(Session &session)
{
    const chess::Board &expected = session.game.board;
    if (!Observe_board(session, &expected)) return false;
    if (session.observed == expected) return true;
    do
    {
        reportBoard(session, &expected, "请恢复原棋局");
        if (!Observe_board(session, &expected)) return false;
    } while (session.observed != expected);
    Operator_show(session, "原棋局已恢复。");
    return true;
}

} // namespace scara
