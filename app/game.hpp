// StaticParams 在开局前准备，DynamicParams 随对局更新。
#pragma once
#include "chess/board.hpp"
#include "vision/geometry.hpp"
#include "motion/plan.hpp"
#include <csignal>
#include <cstdint>

namespace scara
{
//---------静态参数区---------
struct StaticParams
{
    // 固定棋盘的 90 个格点坐标，用于视觉归格和查找取放位置。
    Geometry layout;
    // 平面关节参考和 J1 的取放高度，交给运动规划。
    ArmGeometry arm;
    WorkHeights heights;
    JointTarget park;           // 观察位的 J2 反馈角和 J3 电机角，单位度。
    // 被吃棋子的盘外放置点，使用机械臂基座 XY，单位毫米。
    cv::Point2d capturePlaceMm;
    // 模型输入裁剪框在原图中的左上角，单位像素。
    cv::Point visionCropOrigin;
    //Pikafish程序及配套NNUE所在目录
    std::string engineDir = "engine/pikafish-2023-03-05";
    //J1/J2/J3共用时间规划，各轴使用自己的速度和加速度上限
    MotionLimits motion;
    // humanRed 决定玩家执哪一方，redToMove 指定开局时轮到哪一方。
    bool humanRed = true, redToMove = true;
    int skillLevel = 20; // Pikafish 棋力档，0–20。
    //每步思考时间，毫秒
    int thinkMs = 1000;
};
//---------动态参数区---------
struct DynamicParams
{
    // 已确认盘面、轮次、对局结果和棋谱。
    chess::Game game;
    // 最近一次归格的盘面。连续三张不同的有效帧相同，盘面才算确认。
    chess::Board observed{};
    bool stable = false;
    uint64_t lastFrameId = 0; // 最近接收的帧号。同一局的帧号递增。
    //供画面表示这一手尚未确认，控制顺序由当前回合函数负责
    bool pending = false;
};
//---------操作员命令区---------
enum class Command
{
    // 回车用于确认当前步骤。玩家亲手搬棋后，回车会触发视觉核盘。
    Ready,
    Resign,
    Draw,
    Quit,
    // 面板选择起点和终点后，由机械臂代走。
    Move,
    Invalid // 终端输入尚不能组成一条命令。
};
extern volatile std::sig_atomic_t gameRunning;

// 从确认开局运行到终局或退出。返回值同时反映设备收尾是否成功。
int Match_run(const StaticParams &site, DynamicParams &match);
// 对局或入口准备失败时，统一停车、回收设备并发布最终结果。
int Match_finish(const DynamicParams &match, std::string failure);

//---------设备与交互接口区---------
// 结束本次运行，回报停车及资源回收是否成功。
bool Runtime_close();
// 交付本帧的检测框与帧号。显示由需要画面的设备入口处理。
void Vision_observe(uint64_t &frameId, std::vector<Detection> &pieces);
// 用起始 FEN 和已确认历史查询合法表，电脑行棋时还要选出应手。
void Engine_query(const chess::Game &game, chess::MoveList &legal, chess::Move &choice);
// 确认可进入观察位，再核对相机读到的开局盘面。
bool Operator_prepare();
bool Operator_confirmOpening(const chess::Board &initial);
// 读取输入。传入 move 时，也可接收上位机选中的走法。
Command Operator_read(const std::string &prompt, chess::Move *move = nullptr);
// 执行排好的动作表，返回本次执行结果。落点由调用方另行核盘。
bool Robot_execute(const std::string &purpose, const chess::Move &move, const std::vector<PickStep> &steps);
// 显示盘面和操作提示。只在请玩家选棋时提供可选走法。
void Operator_show(const DynamicParams &dynamicParams, const std::string &message = {}, const chess::MoveList *choices = nullptr);
} // namespace scara
