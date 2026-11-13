#include "game_files.hpp"
#include "vision/io.hpp"
#include <chrono>
#include <filesystem>
#include <iostream>
#include <thread>

namespace scara
{
static std::string directory;    // 对局文件所在目录。
static std::string runId;        // 本次程序运行编号，区分以前留下的文件。
static std::string activeTicket; // 当前等待执行器完成的动作编号。
static unsigned requestNumber = 0;
static bool simulated = false;

std::string Runtime_path(const std::string &name)
{
    std::filesystem::path path = std::filesystem::u8path(directory);
    path /= name;
    return path.u8string();
}

bool Runtime_publish(const std::string &name, const std::string &text)
{
    std::filesystem::path target = std::filesystem::u8path(Runtime_path(name));
    std::filesystem::path temporary = target;
    std::ofstream file;
    std::error_code error;

    temporary += ".tmp";
    file.open(temporary, std::ios::binary);
    file.write(text.data(), std::streamsize(text.size()));
    if (!file) return false;
    file.close();
    if (!file) return false;
    std::filesystem::rename(temporary, target, error);
    return !error;
}

void Files_open(const std::string &path, bool simulation)
{
    std::chrono::system_clock::time_point started;

    directory = path;
    simulated = simulation;
    started = std::chrono::system_clock::now();
    runId = std::to_string(started.time_since_epoch().count());
    activeTicket.clear();
    requestNumber = 0;
    std::filesystem::create_directories(std::filesystem::u8path(directory));
}

void Motion_request(const std::string &purpose, const std::string &move, const std::vector<PickStep> &steps,
                    std::string &ticket)
{
    cv::FileStorage file; // 在内存里逐字段写出JSON。
    std::string message;  // 完整请求写好后才发布给执行器。

    if (!gameRunning) throw std::runtime_error("对局已停止");
    ++requestNumber;
    ticket = runId + "-" + std::to_string(requestNumber);
    memoryStorage(true, file);
    file << "ticket" << ticket;
    file << "simulated" << int(simulated);
    file << "purpose" << purpose;
    file << "move" << move;
    file << "steps" << "[";
    for (const PickStep &step : steps)
    {
        file << "{";
        if (step.action == PickAction::Hold) file << "action" << "hold";
        if (step.action == PickAction::MoveXY)
        {
            file << "action" << "xy";
            file << "j2_deg" << step.joints.j2Deg;
            file << "j3_motor_deg" << step.joints.j3MotorDeg;
        }
        if (step.action == PickAction::MoveZ)
        {
            file << "action" << "z";
            file << "j1_deg" << step.zDeg;
            file << "speed_deg_s" << step.speedDegS;
        }
        if (step.action == PickAction::ReleasePulse)
        {
            file << "action" << "release";
            file << "duration_ms" << step.durationMs;
        }
        file << "}";
    }
    file << "]";
    message = file.releaseAndGetString();
    activeTicket = ticket;
    if (!Runtime_publish("motion-request.json", message)) throw std::runtime_error("无法发布动作请求");
}

void Motion_wait(const std::string &ticket, bool &success)
{
    std::string path = Runtime_path("motion-reply.json");
    std::string replyTicket;
    cv::FileStorage reply;
    cv::FileNode result;
    bool requested;

    success = false;
    while (gameRunning)
    {
        if (std::filesystem::exists(std::filesystem::u8path(path)))
        {
            readStorage(path, reply);
            reply["ticket"] >> replyTicket;
            if (replyTicket == ticket && int(reply["settled"]) == 1)
            {
                result = reply["success"];
                if (!result.isInt() || (int(result) != 0 && int(result) != 1))
                    throw std::runtime_error("执行回执缺少明确结果");
                success = int(result) == 1;
                if (!success) std::cerr << "执行器报告失败：" << (std::string)reply["error"] << '\n';
                activeTicket.clear();
                return;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    requested = Motion_cancel();
    throw std::runtime_error(requested ? "已发停止请求，停稳需执行器确认" : "停止请求未能写出");
}

bool Motion_cancel()
{
    cv::FileStorage file;
    std::string message;
    bool sent;

    if (activeTicket.empty()) return true;
    memoryStorage(true, file);
    file << "ticket" << activeTicket;
    file << "action" << "stop";
    message = file.releaseAndGetString();
    sent = Runtime_publish("motion-cancel.json", message);
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
    const chess::Game &game = session.game;
    cv::FileStorage file;
    std::chrono::system_clock::time_point now;
    std::chrono::duration<double, std::milli> elapsed;
    std::string message;

    if (directory.empty()) return;
    memoryStorage(true, file);
    now = std::chrono::system_clock::now();
    elapsed = now.time_since_epoch();
    file << "run_id" << runId;
    file << "updated_at_ms" << elapsed.count();
    file << "frame_id" << session.lastFrameId;
    file << "simulated" << int(simulated);
    file << "phase" << phase;
    file << "base_fen" << game.baseFen;
    file << "history" << game.history;
    file << "red_to_move" << int(game.redToMove);
    file << "human_red" << int(game.humanRed);
    file << "pending" << int(game.pending);
    file << "result" << int(game.result);
    file << "error" << game.error;
    file << "observation_valid" << int(session.observed.valid);
    writeBoard(file, "board", game.board);
    writeBoard(file, "observed", session.observed.board);
    writeBoard(file, "expected", game.expected);
    message = file.releaseAndGetString();
    if (!Runtime_publish("status.json", message)) std::cerr << "无法保存对局状态\n";
    std::cout << (simulated ? "[SIMULATION] " : "") << phase << " | " << game.history << " | " << game.error << '\n';
    if (phase == "confirm_start")
        for (const auto &row : session.observed.board)
            std::cout << std::string(row.begin(), row.end()) << '\n';
    std::cout.flush();
}
} // namespace scara
