// 相机确认一张实盘。和已记谱盘面或预期落点不同，就指出格号让操作者处理。
#include "game.hpp"

namespace scara
{
// 把需要核对的格号写进提示，保留相机实际交付的盘面。
static void reportBoard(const DynamicParams &dynamicParams, const chess::Board *expected, std::string message)
{
    if (expected)
        for (int row = 0; row < Rows; ++row)
            for (int col = 0; col < Columns; ++col)
                if ((*expected)[row][col] != dynamicParams.observed[row][col])
                    message += " (" + std::to_string(row) + "," + std::to_string(col) + ")";
    Operator_show(dynamicParams, message);
}

//---------观察与恢复区---------
bool Observe_board(const Geometry &layout, DynamicParams &dynamicParams, const chess::Board *expected)
{
    int same = 0;
    bool warned = false; // 同一处遮挡只提示一次，格子变化后再更新。
    dynamicParams.observationStable = false;
    Operator_show(dynamicParams);
    while (gameRunning && same < 3)
    {
        std::string frameId;
        std::vector<Detection> pieces;
        Vision_observe(layout, frameId, pieces);
        if (!gameRunning) return false;
        BoardObservation frame = Geometry_assign(layout, pieces, !frameId.empty());
        if (frame.valid && same > 0 && frameId == dynamicParams.lastFrameId) continue;
        dynamicParams.lastFrameId = frameId;
        if (frame.cells != dynamicParams.observed)
        {
            same = 0;
            warned = false;
        }
        dynamicParams.observed = frame.cells;
        if (frame.valid) ++same;
        else
        {
            same = 0;
            if (!warned) reportBoard(dynamicParams, expected, "棋子偏离格点或受遮挡，请检查实盘。");
            warned = true;
        }
    }
    if (!gameRunning) return false;
    dynamicParams.observationStable = true;
    Operator_show(dynamicParams);
    return true;
}

bool Observe_placement(const Geometry &layout, DynamicParams &dynamicParams, const chess::Board &expected)
{
    if (!Observe_board(layout, dynamicParams, &expected)) return false;
    while (dynamicParams.observed != expected)
    {
        reportBoard(dynamicParams, &expected, "请核对落点");
        if (!Operator_waitReady("修正落点后回车。若摆回走棋前的局面，回车后重走。"))
            return false;
        if (!Observe_board(layout, dynamicParams, &expected)) return false;
        if (dynamicParams.observed == dynamicParams.game.board) return false;
    }
    return true;
}

// 实盘恢复到已记谱的棋局，才能继续操作。
bool Observe_original(const Geometry &layout, DynamicParams &dynamicParams)
{
    const chess::Board &expected = dynamicParams.game.board;
    if (!Observe_board(layout, dynamicParams, &expected)) return false;
    if (dynamicParams.observed == expected) return true;
    do
    {
        reportBoard(dynamicParams, &expected, "请恢复原棋局");
        if (!Observe_board(layout, dynamicParams, &expected)) return false;
    } while (dynamicParams.observed != expected);
    Operator_show(dynamicParams, "原棋局已恢复。");
    return true;
}

} // namespace scara
