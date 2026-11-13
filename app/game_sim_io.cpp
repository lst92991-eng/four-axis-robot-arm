#include "game_files.hpp"
#include "vision/io.hpp"
#include <algorithm>
#include <iostream>

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

void IO_open(const GameConfig &config, const std::string &)
{
    Files_open(config.workDir, true);
    script = readStorage(config.scenarioPath);
    readBoard(script["initial_board"]);
    std::cout << "SIMULATION：不会加载相机、NPU、引擎或控制硬件。\n";
}

void IO_stop()
{
}
bool IO_close()
{
    return Motion_cancel();
}

void IO_observe(const FixedBoard &layout, std::string &frameId, std::vector<Detection> &pieces)
{
    frameId = std::to_string(++frameNumber);
    pieces.clear();
    std::string symbols = PieceSymbols;
    for (int row = 0; row < 10; ++row)
        for (int col = 0; col < 9; ++col)
            if (world[row][col] != '.')
            {
                auto center = layout.grid.intersections[(9 - row) * Columns + col];
                pieces.push_back({int(symbols.find(world[row][col])), .9, {center.x - 10, center.y - 10, 20, 20}});
            }
}

void IO_search(const chess::Game &game, std::vector<std::string> &legal, std::string &choice)
{
    for (const auto &query : script["queries"])
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

Command IO_command(const std::string &phase, const std::string &)
{
    auto commands = script["commands"];
    if (commandIndex >= commands.size()) return Command::Quit;
    auto command = commands[int(commandIndex++)];
    if ((std::string)command["phase"] != phase) throw std::runtime_error("仿真进入了意外的交互阶段：" + phase);
    if (!command["board"].empty()) readBoard(command["board"]);
    std::string name = (std::string)command["command"];
    if (name == "ready") return Command::Ready;
    if (name == "retry") return Command::Retry;
    if (name == "accept") return Command::Accept;
    if (name == "restored") return Command::Restored;
    if (name == "resign") return Command::Resign;
    if (name == "draw") return Command::Draw;
    if (name == "quit") return Command::Quit;
    throw std::runtime_error("未知仿真命令");
}

void IO_execute(const std::string &purpose, const std::string &move, const std::vector<PickStep> &steps, bool &success)
{
    auto events = script["executions"];
    if (executionIndex >= events.size()) throw std::runtime_error("仿真出现了额外的机械执行请求");
    auto event = events[int(executionIndex++)];
    if ((std::string)event["purpose"] != purpose || (std::string)event["move"] != move)
        throw std::runtime_error("仿真动作与预定场景不符");
    std::string ticket;
    Motion_request(purpose, move, steps, ticket);
    writeBytes(Runtime_path("request-" + std::to_string(executionIndex) + ".json"),
               readBytes(Runtime_path("motion-request.json")));
    if (!event["board"].empty()) readBoard(event["board"]); // 独立场景决定落子结果，不直接复制计划盘面。
    auto reply = memoryStorage(true);
    reply << "ticket" << ticket << "settled" << 1 << "success" << int(event["success"]);
    writeBytes(Runtime_path("motion-reply.json"), reply.releaseAndGetString());
    Motion_wait(ticket, success);
}
} // namespace scara
