#include "game_files.hpp"
#include "vision/io.hpp"
#include <chrono>
#include <filesystem>
#include <iostream>

namespace scara
{
//对局文件所在目录
static std::filesystem::path directory;
static bool simulated = false;
//显示层保留事件原因，收到恢复事件后清除
static std::string eventMessage;

static std::string describe(const chess::Board &expected, const chess::Board &actual);

//事件回调只更新显示文字；棋盘指针仅在本次回调中读取
void Files_event(const AppEvent &event)
{
    std::string text;
    if (event.expected && event.observed) text = describe(*event.expected, *event.observed);
    text += event.message;
    if (event.type == EventType::Restored) text.clear();
    if (text == eventMessage) return;
    eventMessage = text;
    if (!text.empty()) std::cout << text << '\n';
}

//---------输出目录初始化区---------
void Files_open(const std::string &path, bool simulation)
{
    directory = std::filesystem::u8path(path);
    simulated = simulation;
    std::filesystem::create_directories(directory);
}

//---------文件发布区---------
//写入临时文件并关闭后再替换，让页面始终读到完整的一份结果
bool Runtime_publish(const std::string &name, const std::string &text)
{
    std::filesystem::path target = directory / name;
    std::filesystem::path temporary = target;
    std::ofstream file;
    std::error_code error;

    temporary += ".tmp";
    file.open(temporary, std::ios::binary);
    file.write(text.data(), std::streamsize(text.size()));
    file.close();
    if (!file) return false;
    std::filesystem::rename(temporary, target, error);
    return !error;
}

//按十行字符串保存棋盘
static void writeBoard(cv::FileStorage &file, const char *key, const chess::Board &board)
{
    file << key << "[";
    for (const auto &row : board)
        file << std::string(row.begin(), row.end());
    file << "]";
}

//---------状态发布区---------
//将当前盘面、走棋历史和错误原因交给预览页面
void Match_publishStatus(const std::string &phase, const Session &session, const std::vector<std::string> *legal)
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
    file << "updated_at_ms" << elapsed.count();
    file << "frame_id" << session.lastFrameId;
    file << "simulated" << int(simulated);
    file << "phase" << phase;
    file << "history" << game.history;
    file << "pending" << int(session.pending);
    file << "result" << int(game.result);
    file << "error" << eventMessage;
    file << "observation_valid" << int(session.observed.valid);
    if (legal) file << "legal" << *legal;
    writeBoard(file, "board", game.board);
    writeBoard(file, "observed", session.observed.board);
    message = file.releaseAndGetString();
    if (!Runtime_publish("status.json", message)) std::cerr << "无法保存对局状态\n";
}
//---------棋盘差异显示区---------
static std::string describe(const chess::Board &expected, const chess::Board &actual)
{
    static const char *names[] = {"黑将","黑士","黑象","黑马","黑车","黑炮","黑卒","红帅","红仕","红相","红马","红车","红炮","红兵"};
    const std::string symbols = PieceSymbols;
    std::vector<int> changed;
    for (int i = 0; i < Cells; ++i)
        if (expected[i / Columns][i % Columns] != actual[i / Columns][i % Columns]) changed.push_back(i);
    std::string text = "棋盘发生变化：";
    if (changed.size() == 1 && actual[changed[0]/Columns][changed[0]%Columns] == '.')
        text = "检测到棋子缺失：";
    if (changed.size() == 2)
    {
        int a = changed[0], b = changed[1];
        if (expected[a/Columns][a%Columns] == actual[b/Columns][b%Columns] &&
            expected[b/Columns][b%Columns] == actual[a/Columns][a%Columns] &&
            expected[a/Columns][a%Columns] != '.' && expected[b/Columns][b%Columns] != '.') text = "检测到棋子位置互换：";
    }
    for (int index : changed)
    {
        text += char('a' + index % Columns);
        text += char('0' + Rows - 1 - index / Columns);
        text += " ";
        for (char piece : {expected[index/Columns][index%Columns], actual[index/Columns][index%Columns]})
        {
            size_t id = symbols.find(piece);
            text += id < 14 ? names[id] : (piece == '.' ? "空位" : "未识别");
            text += " → ";
        }
        text.resize(text.size() - std::string(" → ").size());
        text += "；";
    }
    return text;
}

} // namespace scara
