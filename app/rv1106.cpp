// RV1106 上的相机、NPU、棋力引擎和扩展板接口。
#include "app/game.hpp"
#include "app/rv1106.hpp"
#include "motion/execute.hpp"
#include "chess/pikafish.hpp"
#include "vision/capture.hpp"
#include <iostream>
#include <stdexcept>

namespace scara
{
//---------板端运行数据区---------
static std::string model;
static cv::Point visionCrop;
static std::string engineDirectory;
static int engineSkill, engineThinkMs;
static MotionLimits motionLimits;
// 首次观察时打开相机和模型。每次完成识别递增帧号。
static bool visionOpen = false;
static uint64_t frameNumber = 0;

static void closeVision()
{
    Camera_close();
    Yolo_close();
    visionOpen = false;
}

//---------初始化与收尾区---------
void RV1106_configure(const std::string &modelPath, cv::Point cropOrigin,
                      const std::string &engineDir, int skillLevel, int thinkMs,
                      const MotionLimits &limits)
{
    model = modelPath;
    visionCrop = cropOrigin;
    engineDirectory = engineDir;
    engineSkill = skillLevel;
    engineThinkMs = thinkMs;
    motionLimits = limits;
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
void Vision_observe(uint64_t &frameId, std::vector<Detection> &pieces)
{
    if (!gameRunning) return;
    // 开局第一次观察或上次关闭视觉后，在此重新建立取流和推理。
    if (!visionOpen)
    {
        if (!Yolo_open(model)) throw std::runtime_error("无法打开NPU模型");
        Camera_open();
        visionOpen = true;
    }
    cv::Mat source = Camera_read();
    if (!gameRunning) return;
    if (source.size() != cv::Size(ImageWidth, ImageHeight))
        throw std::runtime_error("相机未返回 1280×720 原图");
    cv::Mat input = Vision_prepare(source, visionCrop);
    if (!Yolo_run(input, visionCrop, pieces))
        throw std::runtime_error("NPU 推理失败");
    frameId = ++frameNumber;
}

//---------引擎查询区---------
void Engine_query(const chess::Game &game, chess::MoveList &legal, chess::Move &choice)
{
    // 搜索前释放相机和 NPU 缓冲，为 RV1106 上的引擎和权重腾出内存。
    closeVision();
    if (!Pikafish_query(engineDirectory.c_str(), game, engineSkill, engineThinkMs,
                        gameRunning, legal, choice) && gameRunning)
        throw std::runtime_error("皮卡鱼查询失败，请检查引擎和配套权重。");
}

//---------对局日志区---------
void Operator_show(const DynamicParams &, const std::string &message, const chess::MoveList *)
{
    if (!message.empty()) std::cerr << message << '\n';
}

//---------扩展板按钮接入区---------
bool Operator_prepare()
{
    // 开局确认由扩展板按钮提供。
    throw std::runtime_error("扩展板按钮尚未接入，无法确认开局");
}

bool Operator_confirmOpening(const chess::Board &)
{
    return false;
}

Command Operator_read(const std::string &, chess::Move *)
{
    gameRunning = 0;
    return Command::Quit;
}

//---------机械执行区---------
bool Robot_execute(const std::string &, const chess::Move &, const std::vector<PickStep> &steps)
{
    // 关闭视觉后按 steps 执行三轴与磁铁动作，执行期间继续检查停止标志。
    closeVision();
    return Motion_execute(steps, motionLimits, gameRunning);
}

//---------驱动收发区---------
// 这里预留原生 SPI-CAN 与 H 桥接口。Windows 台架在 usb_io.cpp 接入实际设备。
// 四个接口目前都返回 false，上层据此知道设备操作尚未完成。
bool Drive_read(DriveFeedback &feedback)
{
    // SPI-CAN 接入后读取状态帧（目标、到位、静止）；确认属于这次命令后才填写 reached。
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
