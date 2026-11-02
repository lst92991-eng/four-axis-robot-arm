// Release构建也运行断言中的执行步骤。
#ifdef NDEBUG
#undef NDEBUG
#endif
#include "robot/simulator.hpp"
#include "app/config.hpp"
#include "algorithm/ik.hpp"
#include <cassert>
#include <cmath>
#include <limits>
#include <string>

namespace {

double duration(const robot::Simulator& simulator)
{
    double seconds = 0;
    for (const auto& step : simulator.plan().steps) seconds += step.seconds;
    return seconds;
}

void closePose(const scara::Pose& a, const scara::Pose& b)
{
    assert(std::hypot(std::hypot(a.x - b.x, a.y - b.y), a.z - b.z) < 1e-7);
    assert(std::abs(std::remainder(a.yaw - b.yaw, 2 * scara::Pi)) < 1e-8);
}

} // namespace

void testSimulator()
{
    const auto initial = chess::initialBoard();
    const chess::Move pawnMove = {6, 0, 5, 0};
    robot::Simulator arm;
    assert(!arm.needsReset() && !arm.busy() && !arm.succeeded());
    assert(!arm.vacuumOn());
    closePose(arm.tcp(), {0, 220, 70, 0});
    assert(arm.start(initial, pawnMove));
    assert(arm.board() == initial && arm.carried() == '.' && arm.busy() && !arm.vacuumOn());
    assert(!arm.plan().calibrationConfirmed);
    assert(arm.plan().steps.front().tool == scara::ToolAction::Keep);

    // 半段时间只能走到关节中点，不能瞬间跳到笛卡尔终点或修改目标棋盘。
    const auto start = arm.joints();
    const auto first = arm.plan().steps.front();
    arm.step(first.seconds / 2);
    assert(std::abs(arm.joints().shoulder - (start.shoulder + first.joints.shoulder) / 2) < 1e-9);
    assert(std::abs(arm.joints().elbow - (start.elbow + first.joints.elbow) / 2) < 1e-9);
    closePose(arm.tcp(), scara::forward(arm.joints()));
    assert(arm.board() == initial && !arm.succeeded());
    bool sawCarriedPawn = false, sawReleasedBeforeFinish = false;
    for (int frame = 0; frame < 30000 && arm.busy(); ++frame) {
        arm.step(0.01);
        if (arm.carried() == 'P') {
            sawCarriedPawn = true;
            assert(arm.vacuumOn());
            assert(arm.board()[6][0] == '.' && arm.board()[5][0] == '.');
        }
        if (arm.board()[5][0] == 'P' && arm.busy()) {
            sawReleasedBeforeFinish = true;
            assert(arm.carried() == '.' && !arm.succeeded() && !arm.vacuumOn());
        }
    }
    chess::Board expected;
    std::string error;
    assert(chess::applyMove(initial, true, pawnMove, expected, error));
    assert(sawCarriedPawn && sawReleasedBeforeFinish && arm.succeeded() && !arm.busy());
    assert(arm.board() == expected && arm.carried() == '.' && !arm.vacuumOn());
    closePose(arm.tcp(), arm.plan().steps.back().pose);

    // 吃子先搬对方，再搬己方；两个吸取阶段都应真实反映到虚拟世界。
    auto captureBoard = initial;
    captureBoard[6][0] = '.';
    const chess::Move capture = {9, 0, 3, 0};
    assert(arm.reset(captureBoard));
    assert(arm.start(captureBoard, capture));
    bool sawCapturedPawn = false, sawOwnRook = false;
    for (int frame = 0; frame < 30000 && arm.busy(); ++frame) {
        arm.step(0.01);
        if (arm.carried() == 'p') {
            sawCapturedPawn = true;
            assert(arm.board()[3][0] == '.' && arm.board()[9][0] == 'R');
        }
        if (arm.carried() == 'R') {
            sawOwnRook = true;
            assert(arm.captureSlots(false)[0] == 'p');
            assert(arm.board()[3][0] == '.' && arm.board()[9][0] == '.');
        }
    }
    assert(sawCapturedPawn && sawOwnRook && arm.succeeded());
    assert(arm.board()[3][0] == 'R' && arm.board()[9][0] == '.');
    assert(arm.captureSlots(false)[0] == 'p' && arm.captureSlots(true)[0] == '.');

    assert(arm.reset(initial));
    for (const auto& invalid : {chess::Move{-1, 0, 5, 0}, chess::Move{6, 0, 10, 0},
                               chess::Move{6, 0, 5, 9}, chess::Move{6, 0, 6, 0},
                               chess::Move{4, 0, 5, 0}, chess::Move{6, 0, 6, 2},
                               chess::Move{6, 0, 0, 4}}) {
        assert(!arm.start(initial, invalid));
        assert(!arm.busy() && !arm.succeeded() && arm.board() == initial);
    }
    auto unknown = initial;
    unknown[4][0] = '?';
    assert(!arm.start(unknown, pawnMove));

    assert(arm.start(initial, pawnMove));
    for (int frame = 0; frame < 30000 && arm.carried() == '.'; ++frame) arm.step(0.01);
    assert(arm.carried() == 'P' && arm.busy() && arm.vacuumOn());
    const auto stoppedBoard = arm.board();
    const auto stoppedPose = arm.tcp();
    arm.stop();
    assert(!arm.busy() && !arm.succeeded() && arm.needsReset());
    arm.step(1000);
    assert(arm.board() == stoppedBoard && arm.carried() == 'P' && arm.vacuumOn());
    closePose(arm.tcp(), stoppedPose);
    assert(!arm.start(initial, pawnMove));
    assert(!arm.observeHuman(stoppedBoard, {6, 2, 5, 2}));
    assert(arm.board() == stoppedBoard && arm.carried() == 'P' && arm.vacuumOn());
    auto badReset = initial;
    badReset[0][4] = '?';
    assert(!arm.reset(badReset) && arm.vacuumOn() && arm.carried() == 'P');
    assert(arm.reset(initial) && arm.carried() == '.' && !arm.vacuumOn());
    assert(arm.start(initial, pawnMove));
    arm.step(std::numeric_limits<double>::quiet_NaN());
    assert(arm.needsReset() && !arm.succeeded());

    // 数学上可达的棋位，也可能因给定关节限位而无法规划；不得修改虚拟棋盘。
    auto limited = project::simulationMotionConfig();
    scara::Joints park;
    assert(scara::inverse({0, 220, 70, 0}, true, park));
    limited.minimum.shoulder = park.shoulder - 0.001;
    limited.maximum.shoulder = park.shoulder + 0.001;
    robot::Simulator constrained(limited);
    assert(!constrained.needsReset());
    assert(!constrained.start(initial, pawnMove));
    assert(constrained.board() == initial && !constrained.busy());

    // 这里只测搬运和槽容量，不替代引擎的象棋规则测试。
    auto pieces = initial;
    for (auto& row : pieces) {
        for (char& piece : row) {
            if (piece >= 'A' && piece <= 'Z' && piece != 'K') piece = '.';
        }
    }
    pieces[9][0] = 'R';
    assert(arm.reset(pieces));
    int rookRow = 9, rookCol = 0, captured = 0;
    for (int row = 0; row < 10; ++row) {
        for (int col = 0; col < 9; ++col) {
            const char piece = pieces[row][col];
            if (piece < 'a' || piece > 'z' || piece == 'k') continue;
            assert(arm.start(arm.board(), {rookRow, rookCol, row, col}));
            arm.step(duration(arm) + 1);
            assert(arm.succeeded() && arm.captureSlots(false)[captured] == piece);
            ++captured;
            rookRow = row;
            rookCol = col;
        }
    }
    assert(captured == 15);
    auto fullSlots = arm.board();
    fullSlots[5][4] = 'p';
    assert(!arm.start(fullSlots, {rookRow, rookCol, 5, 4}));
    assert(!arm.busy() && !arm.succeeded());
    // 新的确认盘面放回一枚对方棋子，但满槽时人工吃子也必须原子地拒绝。
    assert(arm.start(fullSlots, {rookRow, rookCol, 4, 8}));
    arm.step(duration(arm) + 1);
    assert(arm.succeeded());
    const auto beforeFullHuman = arm.board();
    const auto fullCaptures = arm.captureSlots(false);
    assert(!arm.observeHuman(beforeFullHuman, {4, 8, 5, 4}));
    assert(arm.board() == beforeFullHuman && arm.captureSlots(false) == fullCaptures);

    auto redCapture = initial;
    redCapture[3][0] = '.';
    assert(arm.reset(redCapture));
    assert(arm.start(redCapture, {0, 0, 6, 0}));
    arm.step(duration(arm) + 1);
    assert(arm.succeeded() && arm.captureSlots(true)[0] == 'P');

    // 多轮人工→机械臂交替，不reset场景，不丢掉末端位置和以前的吃子记录。
    robot::Simulator turns;
    const auto parked = turns.tcp();
    assert(turns.observeHuman(initial, pawnMove));
    assert(turns.board() == expected && !turns.busy() && !turns.succeeded());
    assert(turns.plan().steps.empty() && turns.carried() == '.' && !turns.vacuumOn());
    closePose(turns.tcp(), parked);
    assert(turns.start(turns.board(), {3, 0, 4, 0}));
    const auto beforeBusyHuman = turns.board();
    const auto busyPose = turns.tcp();
    assert(!turns.observeHuman(beforeBusyHuman, {6, 2, 5, 2}));
    assert(turns.busy() && turns.board() == beforeBusyHuman);
    closePose(turns.tcp(), busyPose);
    turns.step(duration(turns) + 1);
    assert(turns.succeeded());

    const auto afterRobot = turns.tcp();
    assert(turns.observeHuman(turns.board(), {5, 0, 4, 0}));
    assert(turns.captureSlots(false)[0] == 'p' && turns.board()[4][0] == 'P');
    assert(!turns.succeeded() && turns.plan().steps.empty() && !turns.vacuumOn());
    closePose(turns.tcp(), afterRobot);
    assert(turns.start(turns.board(), {3, 2, 4, 2}));
    turns.step(duration(turns) + 1);
    assert(turns.succeeded() && turns.captureSlots(false)[0] == 'p');
    const auto secondRobot = turns.tcp();
    assert(turns.observeHuman(turns.board(), {6, 2, 5, 2}));
    closePose(turns.tcp(), secondRobot);
    assert(turns.start(turns.board(), {4, 2, 5, 2}));
    turns.step(duration(turns) + 1);
    assert(turns.succeeded() && turns.captureSlots(false)[0] == 'p' && turns.captureSlots(true)[0] == 'P');
    const auto thirdRobot = turns.tcp();
    assert(turns.observeHuman(turns.board(), {6, 4, 5, 4}));
    closePose(turns.tcp(), thirdRobot);
    const auto preservedWorld = turns.board();
    const auto preservedRed = turns.captureSlots(true);
    const auto preservedBlack = turns.captureSlots(false);
    assert(!turns.observeHuman(initial, pawnMove));
    for (const auto& invalid : {chess::Move{-1, 0, 5, 0}, chess::Move{5, 4, 5, 4},
                               chess::Move{4, 4, 4, 5}, chess::Move{6, 6, 6, 8}}) {
        assert(!turns.observeHuman(preservedWorld, invalid));
        assert(turns.board() == preservedWorld && turns.captureSlots(true) == preservedRed
               && turns.captureSlots(false) == preservedBlack && !turns.busy());
        closePose(turns.tcp(), thirdRobot);
    }
}
