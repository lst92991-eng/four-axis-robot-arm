// 对局主程序共用的数据和接口。Session 保存这一局，GameConfig 提供启动参数。
#pragma once
#include "chess/board.hpp"
#include "vision/geometry.hpp"
#include "motion/plan.hpp"
#include <csignal>

namespace scara
{
//---------对局会话数据区---------
//整局共用的机械参数、观测结果和已确认棋局
struct Session
{
    // 正式棋局和固定棋盘的坐标表，分别用于判棋与查找毫米目标。
    chess::Game game;
    Geometry layout;
    // 平面关节参考和 J1 的取放高度，交给运动规划。
    ArmGeometry arm;
    WorkHeights heights;
    JointTarget park;           // 观察位的 J2 反馈角和 J3 电机角，单位度。
    // 相机读回的盘面，与 game 中已经记谱的盘面分开保存。
    chess::Observation observed;
    chess::MoveList legal;           // 本回合引擎给出的合法走法。
    // 观察环节比较帧号，统计连续相同的新画面。
    std::string lastFrameId;
    // 被吃棋子的盘外放置点，使用机械臂基座 XY，单位毫米。
    cv::Point2d capturePlaceMm;
    //供画面表示这一手尚未确认，控制顺序由当前回合函数负责
    bool pending = false;
};
//---------启动参数区---------
struct GameConfig
{
    //相机RTSP地址
    std::string cameraUrl;
    // 原图上的推理区域，依次为左上角 x、y 和宽、高，单位像素。
    cv::Rect visionRoi{300, 70, 650, 600};
    //Pikafish程序及配套NNUE所在目录
    std::string engineDir = "engine/pikafish-2023-03-05";
    //J1/J2/J3共用时间规划，各轴使用自己的速度和加速度上限
    MotionLimits motion;
    // humanRed 决定玩家执哪一方，redToMove 指定开局时轮到哪一方。
    bool humanRed = true, redToMove = true;
    //每步思考时间，毫秒
    int thinkMs = 1000;
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
    Move
};
extern volatile std::sig_atomic_t gameRunning;

// 从确认开局运行到终局或退出。返回值同时反映设备收尾是否成功。
int Match_run(const GameConfig &config, Session &session, const std::string &modelPath);

//---------设备与交互接口区---------
//同一主程序在链接时选择实机IO或仿真IO
void Runtime_open(const GameConfig &config, const std::string &modelPath, Session &session);
// 结束本次运行，回报停车及资源回收是否成功。
bool Runtime_close();
// 交付本帧的检测框与帧号。显示由需要画面的设备入口处理。
void Vision_observe(const Geometry &layout, std::string &frameId, std::vector<Detection> &pieces);
// 用起始 FEN 和已确认历史查询合法表，电脑行棋时还要选出应手。
void Engine_query(const chess::Game &game, chess::MoveList &legal, chess::Move &choice);
// 读取命令成功时写入 command。传入 move 可接收选中的走法。
bool Command_parse(const std::string &text, Command &command, chess::Move *move = nullptr);
// 普通输入写入 command。quit 或输入结束时返回 Quit，由主程序统一收尾。
void Operator_read(const std::string &phase, const std::string &prompt, Command &command, chess::Move *move = nullptr);
// 执行排好的动作表，返回本次执行结果。落点由调用方另行核盘。
bool Robot_execute(const std::string &purpose, const chess::Move &move, const std::vector<PickStep> &steps);
// 显示端直接读取本次会话，message 只在操作员需要处理问题时填写。
void Operator_show(const char *phase, const Session &session, const std::string &message = {});
} // namespace scara
