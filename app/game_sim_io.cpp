#include "game_files.hpp"
#include "vision/io.hpp"
#include <algorithm>
#include <iostream>
#include <iomanip>

namespace scara
{
static cv::FileStorage script;
static chess::Board world;
static size_t commandIndex = 0, executionIndex = 0;
static unsigned frameNumber = 0;

static void readBoard(const cv::FileNode &rows)
{
    if (rows.size() != 10) throw std::runtime_error("仿真盘面必须有10行");
    for (int row = 0; row < 10; ++row)
    {
        std::string line = (std::string)rows[row];
        if (line.size() != 9) throw std::runtime_error("仿真盘面必须有9列");
        std::copy(line.begin(), line.end(), world[row].begin());
    }
}

void Runtime_open(const GameConfig &config, const std::string &)
{
    Files_open(config.workDir, true);
    readStorage(config.scenarioPath, script);
    readBoard(script["initial_board"]);
    std::cout << "SIMULATION：不会加载相机、NPU、引擎或控制硬件。\n";
}

void Runtime_stop()
{
}
bool Runtime_close()
{
    return true; // 仿真没有硬件动作需要停止。
}

void Vision_observe(const FixedBoard &layout, std::string &frameId, std::vector<Detection> &pieces)
{
    std::string symbols = PieceSymbols;
    cv::Point2d center; // 场景中的棋子中心，原图像素。
    Detection piece;

    ++frameNumber;
    frameId = std::to_string(frameNumber);
    pieces.clear();
    for (int row = 0; row < 10; ++row)
        for (int col = 0; col < 9; ++col)
            if (world[row][col] != '.')
            {
                center = layout.grid.intersections[(9 - row) * Columns + col];
                piece.classId = int(symbols.find(world[row][col]));
                piece.confidence = .9;
                piece.bbox.x = center.x - 10;
                piece.bbox.y = center.y - 10;
                piece.bbox.width = 20;
                piece.bbox.height = 20;
                pieces.push_back(piece);
            }
}

void Engine_query(const chess::Game &game, std::vector<std::string> &legal, std::string &choice)
{
    for (const cv::FileNode &query : script["queries"])
        if ((std::string)query["history"] == game.history)
        {
            if (int(query["fail"]) == 1) throw std::runtime_error("仿真引擎查询失败");
            if (!query["base_fen"].empty() && (std::string)query["base_fen"] != game.baseFen)
                throw std::runtime_error("仿真起始FEN不匹配");
            query["legal"] >> legal;
            choice = (std::string)query["choice"];
            return;
        }
    throw std::runtime_error("仿真没有当前完整历史的引擎回复：" + game.history);
}

void Operator_read(const std::string &phase, const std::string &, Command &result)
{
    cv::FileNode commands = script["commands"];
    cv::FileNode command;
    std::string name;

    result = Command::Quit;
    if (commandIndex >= commands.size()) return;
    command = commands[int(commandIndex)];
    ++commandIndex;
    if ((std::string)command["phase"] != phase) throw std::runtime_error("仿真进入了意外的交互阶段：" + phase);
    if (!command["board"].empty()) readBoard(command["board"]);
    command["command"] >> name;
    if (!Command_parse(name, result)) throw std::runtime_error("未知仿真命令");
}

void Robot_execute(const std::string &purpose, const std::string &move, const std::vector<PickStep> &steps,
                   bool &success)
{
    cv::FileNode events = script["executions"];
    cv::FileNode event;
    int result;

    success = false;
    if (!gameRunning) throw std::runtime_error("对局已停止");
    if (executionIndex >= events.size()) throw std::runtime_error("仿真出现了额外的机械执行请求");
    event = events[int(executionIndex)];
    ++executionIndex;
    if ((std::string)event["purpose"] != purpose || (std::string)event["move"] != move)
        throw std::runtime_error("仿真动作与预定场景不符");
    // 单向打印调用参数供离线核对，不生成请求文件，也不从日志接收命令。
    std::cout << "SIM_CALL " << purpose << ' ' << (move.empty() ? "-" : move) << '\n' << std::setprecision(17);
    for (const PickStep &step : steps)
        std::cout << "SIM_STEP " << int(step.action) << ' ' << step.joints.j2Deg << ' ' << step.joints.j3MotorDeg << ' '
                  << step.zDeg << ' ' << step.speedDegS << ' ' << step.durationMs << '\n';
    if (!event["board"].empty()) readBoard(event["board"]); // 独立场景决定落子结果，不直接复制计划盘面。
    result = int(event["success"]);
    if (result != 0 && result != 1) throw std::runtime_error("仿真执行结果必须是0或1");
    success = result == 1;
    std::cout << "SIM_RETURN " << result << '\n';
}
} // namespace scara
