// 运动规划的数据约定和接口，按机械参数、关节目标、动作表与轨迹组织。
#pragma once
#include <opencv2/core.hpp>
#include <array>
#include <vector>

namespace scara
{
//---------机构与示教参数区---------
// 直接使用实机示教保存的模型；J2反馈角、J3电机角的换算分别保留。
struct ArmGeometry
{
    // 从肩轴到肘轴、从肘轴到吸头中心的两段有效臂长，单位毫米。
    double upperMm = 0;
    double forearmMm = 0;
    // 同一示教姿态下的 J2 反馈角与 J3 电机角，单位度。
    double j2ReferenceDeg = 0;
    double j3ReferenceMotorDeg = 0;
    // 每一度驱动读数对应的关节角度。负号表示转向相反。
    double j2JointPerDrive = 1;
    double j3JointPerDrive = 1;
    // 同一示教姿态下，大臂相对基座、小臂相对大臂的实际角度，单位弧度。
    double shoulderReferenceRad = 0;
    double elbowReferenceRad = 0;
};
// 水平目标用驱动可接收的两个角表示，J3 保留电机端读数。
struct JointTarget
{
    double j2Deg = 0;
    double j3MotorDeg = 0;
};
// Z 的工作层保存 J1 丝杆电机的示教角，单位度。
struct WorkHeights
{
    //J1吸取层的已示教角度
    double pickupDeg = 0;
    // 开始横移前要达到的 J1 净空角，当前安装表在这里使用搬运高度。
    double clearanceDeg = 0;
    //搬运层与释放层的已示教角度
    double travelDeg = 0;
    double releaseDeg = 0;
    // 正向吸合后看齐并贴住棋子的时间，单位毫秒。
    int gripDwellMs = 420;
};
//---------三轴轨迹数据区---------
//三个元素依次为J1电机角、J2反馈角、J3电机角；J4保持原姿态
using AxisValues = std::array<double, 3>;
// 轴选择用位掩码表示，最低位选择 J1，三个低位全置一选择 J1 至 J3。
constexpr unsigned ZAxis = 1, AllAxes = 7;
struct MotionLimits
{
    //每轴的速度和加速度上限，单位为度/秒、度/秒²
    AxisValues velocity{}, acceleration{};
};
struct JointTrajectory
{
    //同一段三轴的起点、终点和公共时长，秒
    AxisValues start{}, target{};
    double seconds = 0;
};
// 一次采样产生同一时刻的角度和角速度，分别用度、度/秒。
struct JointReference
{
    //同一时刻同时交给三个驱动的参考
    AxisValues position{}, velocity{};
};
//---------取放动作数据区---------
// Grip 使磁铁吸持，Release 和 ReleaseOffBoard 分别标记盘上与盘外的关断。
enum class PickAction
{
    Grip,
    Move,
    Release,
    ReleaseOffBoard
};
// steps 的每一项保存一种动作，执行器按追加顺序读取。
struct PickStep
{
    PickAction action = PickAction::Grip;
    AxisValues target{};
    //位0/1/2选择J1/J2/J3，其余轴使用本段起点位置
    unsigned axes = AllAxes;
    // 吸合时的原地停留时间，单位毫秒。
    int durationMs = 0;
};

//---------取放动作接口区---------
// 向本手的动作表追加一个关节目标。
void Plan_move(JointTarget xy, double zDeg, std::vector<PickStep> &steps);
// 先定位和下降，再吸合，最后抬到净空层。目标无效时撤销整张动作表。
void Plan_pick(const ArmGeometry &arm, const WorkHeights &z, cv::Point2d sourceMm, std::vector<PickStep> &steps);
// 在目标格下降并关断磁铁，再抬离棋子。
void Plan_place(const ArmGeometry &arm, const WorkHeights &z, cv::Point2d targetMm, std::vector<PickStep> &steps);
// 在盘外搬运高度释放被吃子。
void Plan_discard(const ArmGeometry &arm, const WorkHeights &z, cv::Point2d binMm, std::vector<PickStep> &steps);

//---------坐标与轨迹接口区---------
// 用驱动角算吸头的基座 XY 毫米，供示教采点核对。
void Arm_forward(const ArmGeometry &arm, const JointTarget &joints, cv::Point2d &baseMm);
// 用基座 XY 毫米算一组关节角，不可达时返回 false。
bool Arm_inverse(const ArmGeometry &arm, cv::Point2d baseMm, JointTarget &joints);
// 按最慢轴的速度和加速度约束确定三轴共用时长。
bool Trajectory_plan(const AxisValues &start, const AxisValues &target, const MotionLimits &limits,
                     JointTrajectory &trajectory);
// 按本段已用秒数写出一组三轴位置和速度参考。
void Trajectory_sample(const JointTrajectory &trajectory, double elapsedSeconds, JointReference &reference);
} // namespace scara
