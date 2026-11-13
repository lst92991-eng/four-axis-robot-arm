#pragma once
#include "chess/session.hpp"
#include <csignal>

namespace scara
{
struct GameConfig
{
    std::string cameraUrl;    // 相机RTSP地址。
    std::string labelPath;    // 中文标签图片。
    std::string engineDir;    // Pikafish程序及配套NNUE所在目录。
    std::string workDir;      // 预览、动作请求、回执和状态的保存目录。
    std::string scenarioPath; // 仅仿真使用的场景脚本。
    JointTarget park;         // 已示教、能看清整张棋盘的J2反馈角与J3电机角，单位度。
    bool humanRed = true, redToMove = true;
    int thinkMs = 1000; // 每步思考时间，毫秒。
};
enum class Command
{
    Ready,
    Retry,
    Accept,
    Restored,
    Resign,
    Draw,
    Quit
};
extern volatile std::sig_atomic_t gameRunning;

void Match_load(const std::string &path, GameConfig &config, Session &session);
void Match_start(const GameConfig &config, Session &session);
void Match_human(Session &session, const std::vector<std::string> &legal);
void Match_robot(const GameConfig &config, Session &session, const std::vector<std::string> &legal,
                 const std::string &choice);

// 相同主程序在链接时选择实机IO或仿真IO，没有两套对局逻辑。
void Runtime_open(const GameConfig &config, const std::string &modelPath);
void Runtime_stop(); // 信号回调只设置停止状态，不等待、不写文件。
bool Runtime_close();
void Vision_observe(const FixedBoard &layout, std::string &frameId, std::vector<Detection> &pieces);
void Engine_query(const chess::Game &game, std::vector<std::string> &legal, std::string &choice);
bool Command_parse(const std::string &text, Command &command);
void Operator_read(const std::string &phase, const std::string &prompt, Command &command);
void Robot_execute(const std::string &purpose, const std::string &move, const std::vector<PickStep> &steps,
                   bool &success);
void Match_report(const std::string &phase, const Session &session);
} // namespace scara
