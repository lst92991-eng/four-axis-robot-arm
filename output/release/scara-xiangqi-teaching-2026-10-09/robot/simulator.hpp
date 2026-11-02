#pragma once
#include "../algorithm/motion_plan.hpp"
#include "../chess/board.hpp"
#include <array>

namespace robot {

// 只操作内存中的虚拟世界；位置由关节轨迹和FK求得，没有真实电机接口。
class Simulator {
public:
    Simulator();
    explicit Simulator(const scara::MotionConfig& config);
    // 明确重建虚拟场景：回停泊点、清空吸盘及吃子槽，不声称真实硬件已复位。
    bool reset(const chess::Board& board);
    // 调用者已用引擎确认合法走法；这里检查搬子数据并先完整规划。
    bool start(const chess::Board& confirmed, const chess::Move& move);
    // 同步已核实的人工动作，保留关节和槽位；不产生机械臂轨迹或成功信号。
    bool observeHuman(const chess::Board& before, const chess::Move& move);
    void step(double dt);
    void stop();

    const chess::Board& board() const { return board_; }
    const scara::Joints& joints() const { return joints_; }
    scara::Pose tcp() const { return scara::forward(joints_); }
    char carried() const { return carried_; }
    bool vacuumOn() const { return vacuumOn_; }
    bool busy() const { return busy_; }
    bool succeeded() const { return succeeded_; }
    bool needsReset() const { return needsReset_; }
    const std::string& error() const { return error_; }
    const scara::MotionPlan& plan() const { return plan_; }
    const std::array<char, 15>& captureSlots(bool red) const { return captures_[red ? 1 : 0]; }

private:
    bool applyTool(scara::ToolAction action);
    scara::MotionConfig config_;
    scara::MotionPlan plan_;
    chess::Board board_{}, expected_{};
    chess::Move move_;
    scara::Joints joints_{}, segmentStart_{};
    std::array<std::array<char, 15>, 2> captures_{};
    std::size_t nextStep_ = 0;
    int toolEvents_ = 0, captureSlot_ = -1;
    double elapsed_ = 0;
    char carried_ = '.';
    bool vacuumOn_ = false;  // 吸盘命令状态，与当前是否携带棋子分别记录。
    bool busy_ = false, succeeded_ = false, needsReset_ = true;
    bool hasCapture_ = false, capturedRed_ = false;
    std::string error_;
};

} // namespace robot
