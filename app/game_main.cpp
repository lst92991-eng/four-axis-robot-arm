#include "game.hpp"
#include <iostream>
#include <filesystem>
#include <stdexcept>

volatile std::sig_atomic_t scara::gameRunning = 1;

static void requestStop(int)
{
    scara::gameRunning = 0;
    scara::IO_stop();
}

int runGame(int argc, char **argv)
{
    using namespace scara;
    if (argc < 2 || argc > 3) return 2;
    const char *configPath = argv[1];                 // 布局、机械参考、示教高度与对局设置。
    const char *modelPath = argc == 3 ? argv[2] : ""; // 实机需要RKNN模型；仿真不加载模型。
    GameConfig config;
    Session session;
    int result = 0;
    std::signal(SIGINT, requestStop);
    std::signal(SIGTERM, requestStop);
    try
    {
        Match_load(configPath, config, session);
        IO_open(config, modelPath);
        Match_start(config, session);
        while (gameRunning)
        {
            std::vector<std::string> legal; // 当前已确认盘面的合法走法。
            std::string choice;             // 机器人回合的最佳应手。

            IO_report("thinking", session);
            IO_search(session.game, legal, choice);
            if (!chess::Game_judge(session.game, legal)) throw std::runtime_error(session.game.error);
            if (session.game.result != chess::Result::Playing)
                gameRunning = 0;
            else if (session.game.redToMove == session.game.humanRed)
                Match_human(session, legal);
            else
                Match_robot(config, session, legal, choice);
            IO_report("turn_complete", session);
            if (session.game.result != chess::Result::Playing) gameRunning = 0;
        }
    }
    catch (const std::exception &error)
    {
        result = gameRunning ? 1 : 0;
        if (gameRunning) session.game.error = error.what();
        std::cerr << error.what() << '\n';
    }
    if (!IO_close())
    {
        result = 1;
        session.game.error += "；停止请求或资源回收未确认";
    }
    std::string phase = "finished";
    if (session.game.result == chess::Result::Playing) phase = "stopped";
    if (session.game.result == chess::Result::ReviewRequired) phase = "review_required";
    if (result) phase = "error";
    IO_report(phase, session);
    return result;
}

#ifdef _WIN32
int wmain(int argc, wchar_t **argv)
{
    std::vector<std::string> text;
    for (int i = 0; i < argc; ++i)
        text.push_back(std::filesystem::path(argv[i]).u8string());
    std::vector<char *> args;
    for (auto &value : text)
        args.push_back(value.data());
    return runGame(argc, args.data());
}
#else
int main(int argc, char **argv)
{
    return runGame(argc, argv);
}
#endif
