#pragma once
#include "chess/board.hpp"
#include "vision/geometry.hpp"
#include "motion/plan.hpp"
#include "common/events.hpp"
#include <csignal>

namespace scara
{
//整局共用的机械参数、观测结果和已确认棋局
struct Session
{
    chess::Game game;
    Geometry layout;
    ArmGeometry arm;
    WorkHeights heights;
    chess::Observation observed;
    std::string lastFrameId;
    cv::Point2d capturePlaceMm;
    //供画面表示这一手尚未确认，控制顺序由当前回合函数负责
    bool pending = false;
};
struct GameConfig
{
    //相机RTSP地址
    std::string cameraUrl;
    //中文标签图片
    std::string labelPath;
    //Pikafish程序及配套NNUE所在目录
    std::string engineDir;
    //预览和棋局状态的保存目录
    std::string workDir;
    //仅仿真使用的场景脚本
    std::string scenarioPath;
    //已示教、能看清整张棋盘的J2反馈角与J3电机角，单位度
    JointTarget park;
    //J1/J2/J3共用时间规划，各轴使用自己的速度和加速度上限
    MotionLimits motion;
    bool humanRed = true, redToMove = true;
    //每步思考时间，毫秒
    int thinkMs = 1000;
};
enum class Command
{
    Ready,
    Resign,
    Draw,
    Quit,
    Move
};
extern volatile std::sig_atomic_t gameRunning;

void Match_load(const std::string &path, GameConfig &config, Session &session);
void Match_start(const GameConfig &config, Session &session);
void Match_human(const GameConfig &config, Session &session, const std::vector<std::string> &legal);
void Match_executeMove(const GameConfig &config, Session &session, const std::vector<std::string> &legal,
                       const std::string &choice);

//同一主程序在链接时选择实机IO或仿真IO
void Runtime_open(const GameConfig &config, const std::string &modelPath);
bool Runtime_close();
void Vision_observe(const Geometry &layout, std::string &frameId, std::vector<Detection> &pieces);
void Engine_query(const chess::Game &game, std::vector<std::string> &legal, std::string &choice);
bool Command_parse(const std::string &text, Command &command, std::string *move = nullptr);
//正常输入写入command；退出或EOF设置停止标志，转入主程序统一收尾
void Operator_read(const std::string &phase, const std::string &prompt, Command &command, std::string *move = nullptr);
void Robot_execute(const std::string &purpose, const std::string &move, const std::vector<PickStep> &steps,
                   bool &success);
//更新页面读取的棋局状态与错误信息
void Match_publishStatus(const std::string &phase, const Session &session, const std::vector<std::string> *legal = nullptr);
} // namespace scara
