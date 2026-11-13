#include "game_files.hpp"
#include "vision/io.hpp"
#include <chrono>
#include <filesystem>
#include <iostream>
#include <thread>

namespace scara
{
static std::string directory, runId, activeTicket;
static unsigned requestNumber = 0;
static bool simulated = false;

std::string Runtime_path(const std::string &name)
{
    return (std::filesystem::u8path(directory) / name).u8string();
}

bool Runtime_publish(const std::string &name, const std::string &text)
{
    std::string target = Runtime_path(name), temporary = target + ".tmp";
    std::ofstream file(std::filesystem::u8path(temporary), std::ios::binary);
    if (!file.write(text.data(), static_cast<std::streamsize>(text.size()))) return false;
    file.close();
    if (!file) return false;
    std::error_code error;
    std::filesystem::rename(std::filesystem::u8path(temporary), std::filesystem::u8path(target), error);
    return !error;
}

void Files_open(const std::string &path, bool simulation)
{
    directory = path;
    simulated = simulation;
    runId = std::to_string(std::chrono::system_clock::now().time_since_epoch().count());
    activeTicket.clear();
    requestNumber = 0;
    std::filesystem::create_directories(std::filesystem::u8path(directory));
}

void Motion_request(const std::string &purpose, const std::string &move, const std::vector<PickStep> &steps,
                    std::string &ticket)
{
    if (!gameRunning) throw std::runtime_error("对局已停止");
    ticket = runId + "-" + std::to_string(++requestNumber);
    auto file = memoryStorage(true);
    file << "ticket" << ticket << "simulated" << int(simulated) << "purpose" << purpose << "move" << move << "steps"
         << "[";
    for (const auto &step : steps)
    {
        file << "{";
        if (step.action == PickAction::Hold) file << "action" << "hold";
        if (step.action == PickAction::MoveXY)
            file << "action" << "xy" << "j2_deg" << step.joints.j2Deg << "j3_motor_deg" << step.joints.j3MotorDeg;
        if (step.action == PickAction::MoveZ)
            file << "action" << "z" << "j1_deg" << step.zDeg << "speed_deg_s" << step.speedDegS;
        if (step.action == PickAction::ReleasePulse) file << "action" << "release" << "duration_ms" << step.durationMs;
        file << "}";
    }
    file << "]";
    activeTicket = ticket;
    if (!Runtime_publish("motion-request.json", file.releaseAndGetString()))
        throw std::runtime_error("无法发布动作请求");
}

void Motion_wait(const std::string &ticket, bool &success)
{
    success = false;
    while (gameRunning)
    {
        if (std::filesystem::exists(std::filesystem::u8path(Runtime_path("motion-reply.json"))))
        {
            auto reply = readStorage(Runtime_path("motion-reply.json"));
            if ((std::string)reply["ticket"] == ticket && int(reply["settled"]) == 1)
            {
                if (!reply["success"].isInt() || (int(reply["success"]) != 0 && int(reply["success"]) != 1))
                    throw std::runtime_error("执行回执缺少明确结果");
                success = int(reply["success"]) == 1;
                if (!success) std::cerr << "执行器报告失败：" << (std::string)reply["error"] << '\n';
                activeTicket.clear();
                return;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    bool requested = Motion_cancel();
    throw std::runtime_error(requested ? "已发停止请求，停稳需执行器确认" : "停止请求未能写出");
}

bool Motion_cancel()
{
    if (activeTicket.empty()) return true;
    auto file = memoryStorage(true);
    file << "ticket" << activeTicket << "action" << "stop";
    bool sent = Runtime_publish("motion-cancel.json", file.releaseAndGetString());
    if (sent) activeTicket.clear();
    return sent;
}

static void writeBoard(cv::FileStorage &file, const char *key, const chess::Board &board)
{
    file << key << "[";
    for (const auto &row : board)
        file << std::string(row.begin(), row.end());
    file << "]";
}

void Match_report(const std::string &phase, const Session &session)
{
    if (directory.empty()) return;
    const auto &game = session.game;
    auto file = memoryStorage(true);
    double nowMs =
        std::chrono::duration<double, std::milli>(std::chrono::system_clock::now().time_since_epoch()).count();
    file << "run_id" << runId << "updated_at_ms" << nowMs << "frame_id" << session.lastFrameId << "simulated"
         << int(simulated) << "phase" << phase << "base_fen" << game.baseFen << "history" << game.history
         << "red_to_move" << int(game.redToMove) << "human_red" << int(game.humanRed) << "pending" << int(game.pending)
         << "result" << int(game.result) << "error" << game.error << "observation_valid" << int(session.observed.valid);
    writeBoard(file, "board", game.board);
    writeBoard(file, "observed", session.observed.board);
    writeBoard(file, "expected", game.expected);
    if (!Runtime_publish("status.json", file.releaseAndGetString())) std::cerr << "无法保存对局状态\n";
    std::cout << (simulated ? "[SIMULATION] " : "") << phase << " | " << game.history << " | " << game.error << '\n';
    if (phase == "confirm_start")
        for (const auto &row : session.observed.board)
            std::cout << std::string(row.begin(), row.end()) << '\n';
    std::cout.flush();
}
} // namespace scara
