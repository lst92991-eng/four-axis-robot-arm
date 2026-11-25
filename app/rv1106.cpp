// RV1106 上使用的相机、NPU、棋力引擎、终端和机械臂接口。
#include "app/game.hpp"
#include "motion/execute.hpp"
#include "chess/pikafish.hpp"
#include "vision/capture.hpp"
#include <cerrno>
#include <iostream>
#include <poll.h>
#include <unistd.h>

namespace scara
{
//---------板端运行数据区---------
static GameConfig settings;
static std::string model;
// 首次观察时打开相机和模型。每次完成识别递增帧号。
static bool visionOpen = false;
static unsigned frameNumber = 0;

static void closeVision();

//---------初始化与收尾区---------
void Runtime_open(const GameConfig &config, const std::string &modelPath, Session &)
{
    // 保存本机参数和模型路径，供对局使用。
    settings = config;
    model = modelPath;
}

bool Runtime_close()
{
    // 先请求机械停稳，再回收棋力引擎和视觉资源。
    bool stopped = Motion_stop();
    bool reaped = Pikafish_close();
    closeVision();
    return stopped && reaped;
}

//---------取流识别区---------
void Vision_observe(const Geometry &, std::string &frameId, std::vector<Detection> &pieces)
{
    if (!gameRunning) return;
    // 开局第一次观察或上次关闭视觉后，在此重新建立取流和推理。
    if (!visionOpen)
    {
        if (!Yolo_open(model)) throw std::runtime_error("无法打开NPU模型");
        Camera_open(settings.cameraUrl);
        visionOpen = true;
    }
    try
    {
        cv::Mat source = Camera_read(); // 1280×720 BGR 原图
        if (!gameRunning) return;
        if (source.empty()) throw std::runtime_error("相机未读到画面");
        cv::Mat input = Vision_prepare(source, settings.visionCropOrigin);
        pieces = Yolo_run(input, settings.visionCropOrigin);
    }
    catch (...)
    {
        closeVision();
        throw;
    }
    ++frameNumber;
    frameId = std::to_string(frameNumber);
}

//---------引擎查询区---------
void Engine_query(const chess::Game &game, chess::MoveList &legal, chess::Move &choice)
{
    // 搜索前释放相机和 NPU 缓冲，为 RV1106 上的引擎和权重腾出内存。
    closeVision();
    if (!Pikafish_query(settings.engineDir.c_str(), game, settings.thinkMs, gameRunning, legal, choice) && gameRunning)
        throw std::runtime_error("皮卡鱼查询失败，请检查引擎和配套权重。");
}

//---------终端显示区---------
static void printBoard(const chess::Board &board)
{
    for (const auto &row : board) std::cout << std::string(row.begin(), row.end()) << '\n';
}

void Operator_show(const Session &session, const std::string &message, const chess::MoveList *choices)
{
    if (!message.empty()) std::cout << message << '\n';
    if (choices || session.game.result != chess::Result::Playing) printBoard(session.game.board);
}

//---------终端输入区---------
bool Operator_prepare()
{
    Command command;
    do
    {
        Operator_read("确认空吸头已抬离棋盘、手已移开，回车进入观察位。输入 quit 退出。", command);
    } while (gameRunning && command != Command::Ready);
    return gameRunning;
}

bool Operator_confirmOpening(const chess::Board &initial)
{
    printBoard(initial);
    Command command;
    Operator_read("核对开局盘面，回车确认。输入 quit 退出。", command);
    return command == Command::Ready;
}

void Operator_read(const std::string &prompt, Command &command, chess::Move *move)
{
    //按回车之前输入的命令文本
    std::string line;
    char character;
    pollfd input{STDIN_FILENO, POLLIN, 0};
    input.revents = 0;
    //EOF或停止时，向调用方报告退出
    command = Command::Quit;
    std::cout << prompt << '\n' << std::flush;
    // 以 100 毫秒为一轮等键盘，使等候期间也能响应停止标志。
    while (gameRunning)
    {
        int ready = poll(&input, 1, 100);
        if (ready == 0 || (ready < 0 && errno == EINTR)) continue;
        if (ready < 0) break;
        auto count = read(STDIN_FILENO, &character, 1);
        if (count < 0 && errno == EINTR) continue;
        if (count <= 0) break;
        // 接收字符直到换行。回车符用于兼容 CRLF，解析只在整行到齐时发生。
        if (character == '\n')
        {
            if (Command_parse(line, command, move)) break;
            std::cout << "请输入提示中的命令，走棋用 起点行,列 终点行,列。\n" << std::flush;
            line.clear();
        }
        else if (character != '\r') line += character;
    }
    if (command == Command::Quit) gameRunning = 0;
}

//---------机械执行区---------
bool Robot_execute(const std::string &, const chess::Move &, const std::vector<PickStep> &steps)
{
    // 关闭视觉后按 steps 执行三轴与磁铁动作，执行期间继续检查停止标志。
    closeVision();
    return Motion_execute(steps, settings.motion, gameRunning);
}
//---------视觉资源收尾区---------
static void closeVision()
{
    Camera_close();
    Yolo_close();
    visionOpen = false;
}

//---------驱动收发区---------
// 这里预留原生 SPI-CAN 与 H 桥接口。Windows 台架在 usb_io.cpp 接入实际设备。
// 四个接口目前都返回 false，上层据此知道设备操作尚未完成。
bool Drive_read(DriveFeedback &feedback)
{
    // 清空反馈，待接入驱动后填写同一周期的三轴位置、速度和时间。
    feedback = {};
    return false;
}
bool Drive_write(const JointReference &)
{
    // 待接入的操作是一次下发 J1、J2、J3 的参考角和参考角速度。
    return false;
}
bool Drive_magnet(bool)
{
    // 此处预留 H 桥正向吸合和关断两种输出。
    return false;
}
bool Drive_stop()
{
    // 此处预留整组停车及停稳回读。
    return false;
}
} // namespace scara
