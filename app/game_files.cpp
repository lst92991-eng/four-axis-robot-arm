#include "game_files.hpp"
#include "vision/io.hpp"
#include <chrono>
#include <filesystem>
#include <iostream>

namespace scara
{
static std::string directory; // 对局文件所在目录。
static std::string runId;     // 本次程序运行编号，区分以前留下的文件。
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
    std::filesystem::create_directories(std::filesystem::u8path(directory));
}

static void writeBoard(cv::FileStorage &file, const char *key, const chess::Board &board)
{
    file << key << "[";
    for (const auto &row : board)
        file << std::string(row.begin(), row.end());
    file << "]";
}

//将当前盘面、走棋历史和错误原因交给预览页面
void Match_publishStatus(const std::string &phase, const Session &session)
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
}
} // namespace scara
