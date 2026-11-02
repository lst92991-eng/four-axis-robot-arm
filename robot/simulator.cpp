#include "simulator.hpp"
#include "../algorithm/ik.hpp"
#include "../app/config.hpp"
#include <algorithm>
#include <cmath>
#include <utility>

namespace robot {

Simulator::Simulator() : Simulator(project::simulationMotionConfig()) {}

Simulator::Simulator(const scara::MotionConfig& config) : config_(config)
{
    reset(chess::initialBoard());
}

bool Simulator::reset(const chess::Board& board)
{
    busy_ = succeeded_ = false;
    needsReset_ = true;
    if (!chess::validBoard(board, error_)) return false;
    scara::Joints park;
    if (!scara::inverse({0, 220, 70, 0}, true, park)) {
        error_ = "虚拟停泊点不可达。";
        return false;
    }
    scara::MotionPlan check;
    if (!scara::planTrajectory(park, {}, config_, check, error_)) return false;
    board_ = expected_ = board;
    joints_ = segmentStart_ = park;
    for (auto& side : captures_) side.fill('.');
    carried_ = '.';
    vacuumOn_ = false;
    plan_ = {};
    nextStep_ = 0;
    toolEvents_ = 0;
    elapsed_ = 0;
    needsReset_ = false;
    error_.clear();
    return true;
}

bool Simulator::start(const chess::Board& confirmed, const chess::Move& move)
{
    succeeded_ = false;
    error_.clear();
    if (busy_ || needsReset_) {
        error_ = "虚拟机械臂忙碌或取消后尚未复位。";
        return false;
    }
    if (chess::uci(move).empty()) { error_ = "棋盘坐标越界或起终点相同。"; return false; }
    const char piece = confirmed[move.fromRow][move.fromCol];
    const char target = confirmed[move.toRow][move.toCol];
    chess::Board expected;
    if (!chess::applyMove(confirmed, piece >= 'A' && piece <= 'Z', move, expected, error_)) return false;

    const bool capture = target != '.';
    const bool red = target >= 'A' && target <= 'Z';
    int slot = -1;
    scara::MotionPlan complete, part;
    scara::Joints next = joints_;
    if (capture) {
        for (int i = 0; i < 15; ++i) {
            if (captures_[red ? 1 : 0][i] == '.') { slot = i; break; }
        }
        if (slot < 0) { error_ = "该方15个吃子槽已满。"; return false; }
        if (!scara::planPickPlace(next, project::squarePose(move.toRow, move.toCol),
                                  project::capturePose(red, slot), config_, part, error_)) return false;
        complete.steps = std::move(part.steps);
        next = complete.steps.back().joints;
    }
    if (!scara::planPickPlace(next, project::squarePose(move.fromRow, move.fromCol),
                              project::squarePose(move.toRow, move.toCol), config_, part, error_)) return false;
    complete.steps.insert(complete.steps.end(), part.steps.begin(), part.steps.end());
    complete.calibrationConfirmed = config_.calibrated;

    // 此处只接受确认的“动作前盘面”；期望盘面只用于结束核对，不能提前写入世界。
    board_ = confirmed;
    expected_ = expected;
    move_ = move;
    hasCapture_ = capture;
    capturedRed_ = red;
    captureSlot_ = slot;
    plan_ = std::move(complete);
    nextStep_ = 0;
    toolEvents_ = 0;
    elapsed_ = 0;
    segmentStart_ = joints_;
    busy_ = true;
    return true;
}

bool Simulator::observeHuman(const chess::Board& before, const chess::Move& move)
{
    error_.clear();
    if (busy_ || needsReset_ || board_ != before) {
        error_ = "机械臂忙碌、需要复位或人工走棋前盘面不一致。";
        return false;
    }
    if (chess::uci(move).empty()) { error_ = "人工走法坐标无效。"; return false; }
    const char piece = before[move.fromRow][move.fromCol];
    const char captured = before[move.toRow][move.toCol];
    chess::Board after;
    if (!chess::applyMove(before, piece >= 'A' && piece <= 'Z', move, after, error_)) return false;
    const bool capturedRed = captured >= 'A' && captured <= 'Z';
    int slot = -1;
    if (captured != '.') {
        for (int i = 0; i < 15; ++i) {
            if (captures_[capturedRed ? 1 : 0][i] == '.') { slot = i; break; }
        }
        if (slot < 0) { error_ = "人工吃子对应的15个槽位已满。"; return false; }
    }
    // 所有检查结束后才提交人手搬运结果；不移动关节、不清空已有槽位。
    if (slot >= 0) captures_[capturedRed ? 1 : 0][slot] = captured;
    board_ = after;
    expected_ = after;
    plan_ = {};
    succeeded_ = false;
    return true;
}

bool Simulator::applyTool(scara::ToolAction action)
{
    const bool taking = toolEvents_ % 2 == 0;
    const bool capturedPiece = hasCapture_ && toolEvents_ < 2;
    if ((action == scara::ToolAction::VacuumOn) != taking ||
        (taking && carried_ != '.') || (!taking && carried_ == '.')) {
        error_ = "吸放次序与虚拟吸盘状态不一致。";
        return false;
    }
    const int row = taking && !capturedPiece ? move_.fromRow : move_.toRow;
    const int col = taking && !capturedPiece ? move_.fromCol : move_.toCol;
    const scara::Pose destination = !taking && capturedPiece
        ? project::capturePose(capturedRed_, captureSlot_) : project::squarePose(row, col);
    const scara::Pose current = tcp();
    if (std::hypot(std::hypot(current.x - destination.x, current.y - destination.y),
                   current.z - destination.z) > 1e-5) {
        error_ = "工具尚未到达吸放位置。";
        return false;
    }
    if (taking) {
        if (board_[row][col] == '.') { error_ = "吸取位置没有棋子。"; return false; }
        carried_ = board_[row][col];
        board_[row][col] = '.';
        vacuumOn_ = true;
    } else {
        char& cell = capturedPiece ? captures_[capturedRed_ ? 1 : 0][captureSlot_]
                                  : board_[row][col];
        if (cell != '.') { error_ = "释放位置仍有棋子。"; return false; }
        cell = carried_;
        carried_ = '.';
        vacuumOn_ = false;
    }
    ++toolEvents_;
    return true;
}

void Simulator::step(double dt)
{
    if (!busy_ || dt == 0) return;
    if (!std::isfinite(dt) || dt < 0) {
        stop();
        error_ = "虚拟执行时间无效，需重新确认盘面。";
        return;
    }
    while (busy_ && dt > 0) {
        const auto& target = plan_.steps[nextStep_];
        if (elapsed_ == 0) {
            segmentStart_ = joints_;
            // 气路在事件开始时切换，随后按规划等待，不在开始任务时偷改盘面。
            if (target.tool != scara::ToolAction::Keep && !applyTool(target.tool)) {
                busy_ = succeeded_ = false;
                needsReset_ = true;
                return;
            }
        }
        const double advance = std::min(dt, target.seconds - elapsed_);
        elapsed_ += advance;
        dt -= advance;
        const double fraction = target.seconds > 0 ? elapsed_ / target.seconds : 1;
        joints_ = {
            segmentStart_.shoulder + (target.joints.shoulder - segmentStart_.shoulder) * fraction,
            segmentStart_.elbow + (target.joints.elbow - segmentStart_.elbow) * fraction,
            segmentStart_.z + (target.joints.z - segmentStart_.z) * fraction,
            segmentStart_.wrist + (target.joints.wrist - segmentStart_.wrist) * fraction
        };
        if (elapsed_ < target.seconds) return;
        joints_ = target.joints;
        elapsed_ = 0;
        ++nextStep_;
        if (nextStep_ == plan_.steps.size()) {
            busy_ = false;
            succeeded_ = carried_ == '.' && board_ == expected_ && toolEvents_ == (hasCapture_ ? 4 : 2);
            needsReset_ = !succeeded_;
            if (!succeeded_) error_ = "虚拟搬运结束，但世界盘面与目标不一致。";
        }
    }
}

void Simulator::stop()
{
    if (!busy_) return;
    busy_ = succeeded_ = false;
    needsReset_ = true;
    error_ = "虚拟搬运已取消；保留中间棋盘和吸盘状态，需复位或重新观测。";
}

} // namespace robot
