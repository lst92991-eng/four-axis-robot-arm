#include "game_files.hpp"
#include "motion/execute.hpp"
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

static void closeVision();

//---------初始化与收尾区---------
void Runtime_open(const GameConfig &config, const std::string &modelPath)
{
    settings = config;
    model = modelPath;
    if (model.empty() || settings.cameraUrl.empty() || settings.labelPath.empty() || settings.engineDir.empty())
        throw std::runtime_error("实机模式需要模型、相机、字图及引擎路径");
    Files_open(settings.workDir, false);
    //Linux直接读取固定字图，供识别结果绘制使用
    labels = cv::imread(settings.labelPath, cv::IMREAD_GRAYSCALE);
    if (labels.empty()) throw std::runtime_error("无法读取中文字图");
    Vision_setup();
    engine.cancelled = [] { return !gameRunning; };
}

bool Runtime_close()
{
    bool stopped = Motion_stop();
    bool reaped = engine.close();
    closeVision();
    return stopped && reaped;
}

//---------取流识别区---------
void Vision_observe(const Geometry &layout, std::string &frameId, std::vector<Detection> &pieces)
{
    //新收到的1280x720 BGR原图
    cv::Mat source;
    //原有固定ROI生成的640x640 RGB输入
    cv::Mat input;
    //左侧：棋子检测框、类别和分数
    cv::Mat detected;
    //右侧：固定90个交点
    cv::Mat grid;
    //左右拼接后的预览
    cv::Mat preview;
    //NPU三路输出的地址和格式
    YoloHead heads[YoloHeads];
    //压缩后的预览图片
    std::vector<unsigned char> jpeg;
    //待写入文件的图片字节
    std::string bytes;

    if (!gameRunning) throw std::runtime_error("对局已停止");
    if (!visionOpen)
    {
        if (!Yolo_open(model)) throw std::runtime_error("无法打开NPU模型");
        Camera_open(settings.cameraUrl);
        visionOpen = true;
    }
    try
    {
        Camera_read(source);
        Vision_prepare(source, input);
        Yolo_run(input, heads);
        Vision_decode(heads, pieces);
        Vision_draw(source, pieces, labels, detected);
        Geometry_draw(source, layout, grid);
    }
    catch (...)
    {
        closeVision();
        throw;
    }
    //固定双画面尺寸，和识别ROI分别处理
    cv::hconcat(detected, grid, preview);
    if (!cv::imencode(".jpg", preview, jpeg)) throw std::runtime_error("无法生成预览");
    bytes.assign(jpeg.begin(), jpeg.end());
    if (!Runtime_publish("preview.jpg", bytes)) throw std::runtime_error("无法写入预览");
    ++frameNumber;
    frameId = std::to_string(frameNumber);
}

//---------引擎查询区---------
void Engine_query(const chess::Game &game, std::vector<std::string> &legal, std::string &choice)
{
    bool ready, reaped;

    //释放相机线程和NPU后才创建引擎进程
    closeVision();
    choice.clear();
    ready = engine.start(settings.engineDir.c_str());
    if (ready) ready = engine.legalMoves(game, legal);
    if (ready && !legal.empty() && game.redToMove != game.humanRed)
        ready = engine.think(game, settings.thinkMs, choice);
    reaped = engine.close();
    if (!ready || !reaped) throw std::runtime_error("引擎查询失败：" + engine.error);
    //下次观察重新打开相机/NPU，引擎查询失败交给主入口处理
}

//---------终端输入区---------
void Operator_read(const std::string &, const std::string &prompt, Command &command)
{
    //按回车之前输入的命令文本
    std::string line;
    char character;
    pollfd input;

    input.fd = STDIN_FILENO;
    input.events = POLLIN;
    input.revents = 0;
    //EOF或停止时，向调用方报告退出
    command = Command::Quit;
    std::cout << prompt << '\n' << std::flush;
    while (gameRunning)
    {
        if (poll(&input, 1, 100) <= 0) continue;
        if (read(STDIN_FILENO, &character, 1) <= 0) break;
        if (character == '\r') continue;
        if (character != '\n')
        {
            line += character;
            continue;
        }
        if (Command_parse(line, command)) break;
        std::cout << "请输入提示中的命令。\n" << std::flush;
        line.clear();
    }
    //退出、EOF或停止信号在输入入口结束当前流程
    if (command == Command::Quit)
    {
        gameRunning = 0;
        throw std::runtime_error("对局已停止");
    }
}

//---------机械执行区---------
void Robot_execute(const std::string &, const std::string &, const std::vector<PickStep> &steps, bool &success)
{
    closeVision();
    Motion_execute(steps, settings.motion, gameRunning, success);
    if (!gameRunning) throw std::runtime_error("对局已停止");
}
//---------视觉资源收尾区---------
static void closeVision()
{
    Camera_close();
    Yolo_close();
    visionOpen = false;
}

//---------驱动收发区---------
//实机接入点：三轴反馈、整组参考、释放脉冲、减速保持
//当前硬件尚未连接，返回false使执行流程停在未就绪状态
bool Drive_read(DriveFeedback &feedback)
{
    //读取SPI-CAN状态后填写实际位置、速度和本周期时间
    feedback = {};
    Event_emit(EventType::Motion, "真实电机驱动接口尚未接入");
    return false;
}
bool Drive_write(const JointReference &)
{
    //同一周期向J1/J2/J3交付位置与速度参考
    return false;
}
bool Drive_release(int)
{
    //H桥执行定时释放，0毫秒结束脉冲并恢复吸持状态
    return false;
}
bool Drive_stop()
{
    //发送整组减速保持，并通过实际反馈确认三轴停稳
    Event_emit(EventType::Stop, "真实驱动尚未接入，不能确认机械臂停稳");
    return false;
}
} // namespace scara
