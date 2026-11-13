#include "game_files.hpp"
#include "chess/pikafish.hpp"
#include "vision/capture.hpp"
#include "vision/io.hpp"
#include <opencv2/imgcodecs.hpp>
#include <iostream>
#include <poll.h>
#include <unistd.h>

namespace scara
{
static GameConfig settings;
static std::string model;
static cv::Mat labels;
static Pikafish engine;
static bool visionOpen = false;
static unsigned frameNumber = 0;

static void closeVision()
{
    Camera_close();
    Yolo_close();
    visionOpen = false;
}

void IO_open(const GameConfig &config, const std::string &modelPath)
{
    settings = config;
    model = modelPath;
    if (model.empty() || settings.cameraUrl.empty() || settings.labelPath.empty() || settings.engineDir.empty())
        throw std::runtime_error("实机模式需要模型、相机、字图及引擎路径");
    Files_open(settings.workDir, false);
    auto bytes = readBytes(settings.labelPath);
    labels = cv::imdecode(std::vector<unsigned char>(bytes.begin(), bytes.end()), cv::IMREAD_GRAYSCALE);
    Vision_setup();
    engine.cancelled = [] { return !gameRunning; };
}

void IO_stop()
{
    Camera_stop();
}

bool IO_close()
{
    bool stopped = Motion_cancel();
    bool reaped = engine.close();
    closeVision();
    return stopped && reaped;
}

void IO_observe(const FixedBoard &layout, std::string &frameId, std::vector<Detection> &pieces)
{
    if (!gameRunning) throw std::runtime_error("对局已停止");
    if (!visionOpen)
    {
        if (!Yolo_open(model)) throw std::runtime_error("无法打开NPU模型");
        Camera_open(settings.cameraUrl);
        visionOpen = true;
    }
    cv::Mat source; // 新收到的1280x720 BGR原图。
    cv::Mat input;  // 原有固定ROI生成的640x640 RGB输入。
    cv::Mat detected, grid, preview;
    YoloHead heads[YoloHeads];
    Camera_read(source);
    Vision_prepare(source, input);
    Yolo_run(input, heads);
    Vision_decode(heads, pieces);
    Vision_draw(source, pieces, labels, detected);
    drawBoard(source, layout.grid, grid);
    cv::hconcat(detected, grid, preview); // 固定双画面尺寸，和识别ROI分别处理。
    auto path = Runtime_path("preview.jpg");
    std::vector<unsigned char> jpeg;
    if (!cv::imencode(".jpg", preview, jpeg)) throw std::runtime_error("无法生成预览");
    writeBytes(path + ".tmp", {jpeg.begin(), jpeg.end()});
    std::filesystem::rename(std::filesystem::u8path(path + ".tmp"), std::filesystem::u8path(path));
    frameId = std::to_string(++frameNumber);
}

void IO_search(const chess::Game &game, std::vector<std::string> &legal, std::string &choice)
{
    closeVision(); // 释放相机线程和NPU后才创建引擎进程。
    choice.clear();
    bool ready = engine.start(settings.engineDir.c_str());
    if (ready) ready = engine.legalMoves(game, legal);
    if (ready && !legal.empty() && game.redToMove != game.humanRed)
        ready = engine.think(game, settings.thinkMs, choice);
    bool reaped = engine.close();
    if (!ready || !reaped) throw std::runtime_error("引擎查询失败：" + engine.error);
    // 下次观察才重开相机/NPU；失败时不会拿空合法列表宣布胜负。
}

Command IO_command(const std::string &, const std::string &prompt)
{
    std::cout << prompt << '\n' << std::flush;
    std::string line;
    while (gameRunning)
    {
        pollfd input{STDIN_FILENO, POLLIN, 0};
        if (poll(&input, 1, 100) <= 0) continue;
        char character;
        if (read(STDIN_FILENO, &character, 1) <= 0) return Command::Quit;
        if (character == '\r') continue;
        if (character != '\n')
        {
            line += character;
            continue;
        }
        if (line.empty() || line == "ready") return Command::Ready;
        if (line == "retry") return Command::Retry;
        if (line == "accept") return Command::Accept;
        if (line == "restored") return Command::Restored;
        if (line == "resign") return Command::Resign;
        if (line == "draw") return Command::Draw;
        if (line == "quit") return Command::Quit;
        std::cout << "请输入提示中的命令。\n" << std::flush;
        line.clear();
    }
    return Command::Quit;
}

void IO_execute(const std::string &purpose, const std::string &move, const std::vector<PickStep> &steps, bool &success)
{
    closeVision();
    std::string ticket;
    Motion_request(purpose, move, steps, ticket);
    std::cout << "等待执行器回执：" << ticket << '\n' << std::flush;
    Motion_wait(ticket, success);
}
} // namespace scara
