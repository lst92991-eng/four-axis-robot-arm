#pragma once
#include "chess/session.hpp"
#include <csignal>

namespace scara
{
struct GameConfig
{
    std::string cameraUrl, labelPath, engineDir, workDir, scenarioPath;
    JointTarget park; // 已示教、能看清整张棋盘的J2反馈角与J3电机角，单位度。
    bool humanRed = true, redToMove = true;
    int thinkMs = 1000;
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
void IO_open(const GameConfig &config, const std::string &modelPath);
void IO_stop(); // 信号回调只设置停止状态，不等待、不写文件。
bool IO_close();
void IO_observe(const FixedBoard &layout, std::string &frameId, std::vector<Detection> &pieces);
void IO_search(const chess::Game &game, std::vector<std::string> &legal, std::string &choice);
bool Command_parse(const std::string &text, Command &command);
void IO_command(const std::string &phase, const std::string &prompt, Command &command);
void IO_execute(const std::string &purpose, const std::string &move, const std::vector<PickStep> &steps, bool &success);
void IO_report(const std::string &phase, const Session &session);
} // namespace scara
