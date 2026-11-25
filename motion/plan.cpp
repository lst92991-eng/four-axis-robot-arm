// 按取子、落子和回观察位的顺序排出一手棋的动作。
#include "plan.hpp"
#include <stdexcept>

namespace scara
{
//---------取放动作区---------
// 把一个目标姿态追加到 steps。三个位置依次是 J1、J2 和 J3 驱动角。
void Plan_move(JointTarget joints, double zDeg, std::vector<PickStep> &steps)
{
    steps.push_back({PickAction::Move, {zDeg, joints.j2Deg, joints.j3MotorDeg}});
}

// 取子先到棋子上方，再竖直下降吸合，最后抬到允许横移的高度。
static void Plan_pick(const WorkHeights &z, const JointTarget &source, std::vector<PickStep> &steps)
{
    Plan_move(source, z.travelDeg, steps);
    Plan_move(source, z.pickupDeg, steps);
    // 吸头停在取子高度通电，gripDwellMs 留出棋子贴合的时间。
    PickStep grip;
    grip.action = PickAction::Grip;
    grip.durationMs = z.gripDwellMs;
    steps.push_back(grip);
    Plan_move(source, z.clearanceDeg, steps);
}

// 落子先到目标格上方，下降到释放高度，关断磁铁后抬起。
static void Plan_place(const WorkHeights &z, const JointTarget &target, std::vector<PickStep> &steps)
{
    Plan_move(target, z.travelDeg, steps);
    Plan_move(target, z.releaseDeg, steps);
    steps.push_back({PickAction::Release});
    Plan_move(target, z.clearanceDeg, steps);
}

void Plan_turn(const ArmGeometry &arm, const WorkHeights &z, cv::Point2d sourceMm, cv::Point2d targetMm,
               const cv::Point2d *discardMm, const JointTarget &park, std::vector<PickStep> &steps)
{
    steps.clear();
    if (z.pickupDeg == z.travelDeg) throw std::runtime_error("吸取和搬运高度尚未示教");
    JointTarget source, target;
    if (!Arm_inverse(arm, sourceMm, source)) throw std::runtime_error("取子位置不可达");
    if (!Arm_inverse(arm, targetMm, target)) throw std::runtime_error("落子位置不可达");

    // 吃子先移走目标格上的对方棋子，在弃子区上方松开。
    if (discardMm)
    {
        JointTarget bin;
        if (!Arm_inverse(arm, *discardMm, bin)) throw std::runtime_error("弃子位置不可达");
        Plan_pick(z, target, steps);
        Plan_move(bin, z.travelDeg, steps);
        steps.push_back({PickAction::ReleaseOffBoard});
    }
    // 把己方棋子搬到目标格，再回观察位。
    Plan_pick(z, source, steps);
    Plan_place(z, target, steps);
    Plan_move(park, z.travelDeg, steps);
}
} // namespace scara
