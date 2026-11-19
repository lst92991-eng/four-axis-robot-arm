#include "game_files.hpp"
#include "vision/io.hpp"
#include "motion/execute.hpp"
#include <algorithm>
#include <iostream>
#include <iomanip>

namespace scara
{
static cv::FileStorage script;
static chess::Board world;
static size_t commandIndex = 0, executionIndex = 0;
static unsigned frameNumber = 0;
static EventType visibleEvent = EventType::Restored;
static size_t restorationIndex = 0;
static unsigned eventSerial = 0, consumedEvent = 0, restorationFrames = 0;
static std::vector<bool> queryUsed;
static std::string offGrid;
//仿真驱动使用虚拟通信时钟，仍运行正式Motion_execute的联动调度
static MotionLimits motionLimits;
static JointReference driveReference;
static uint64_t driveMs = 0, referenceCount = 0;

static void readBoard(const cv::FileNode &rows);
static void screenEvent(const AppEvent &event);

//---------仿真初始化与收尾区---------
void Runtime_open(const GameConfig &config, const std::string &)
{
    Files_open(config.workDir, true);
    for (size_t i = 0; i < size_t(EventType::Count); ++i) Event_on(static_cast<EventType>(i), screenEvent);
    readStorage(config.scenarioPath, script);
    readBoard(script["initial_board"]);
    queryUsed.assign(script["queries"].size(), false);
    motionLimits = config.motion;
    cv::FileNode initial = script["initial_joints_deg"];
    if (initial.size() != 3) throw std::runtime_error("仿真需要三个初始驱动角度");
    for (int axis = 0; axis < 3; ++axis) driveReference.position[axis] = double(initial[axis]);
    driveReference.velocity = {};
    driveMs = referenceCount = 0;
    std::cout << "SIMULATION：不会加载相机、NPU、引擎或控制硬件。\n";
}

//回调仅记录模拟屏幕显示；场景中的人类恢复在下一次相机输入中发生
static void screenEvent(const AppEvent &event)
{
    Files_event(event);
    visibleEvent = event.type;
    ++eventSerial;
    std::cout << "SIM_EVENT " << int(event.type) << ' ' << event.message << '\n';
}

//仿真没有相机，主入口停止回调仍使用相同的Camera_stop接口
void Camera_stop()
{
}
bool Runtime_close()
{
    return Motion_stop();
}

//---------仿真驱动区---------
//每次读取推进10ms，虚拟执行器直接跟随上个周期的整组参考
bool Drive_read(DriveFeedback &feedback)
{
    driveMs += 10;
    feedback.position = driveReference.position;
    feedback.velocity = driveReference.velocity;
    feedback.timeMs = driveMs;
    return true;
}
bool Drive_write(const JointReference &reference)
{
    driveReference = reference;
    ++referenceCount;
    return true;
}
bool Drive_release(int)
{
    return true;
}
bool Drive_stop()
{
    driveReference.velocity = {};
    return true;
}

//---------场景观察区---------
void Vision_observe(const Geometry &layout, std::string &frameId, std::vector<Detection> &pieces)
{
    if (!script["cancel_after_frames"].empty() && frameNumber >= unsigned(int(script["cancel_after_frames"])))
    {
        gameRunning = 0;
        throw std::runtime_error("仿真取消");
    }
    //模拟人看到事件提示后恢复棋盘，应用仍需三张新图才能确认
    cv::FileNode restorations = script["restorations"];
    if (restorationIndex < restorations.size() && eventSerial != consumedEvent)
    {
        cv::FileNode item = restorations[int(restorationIndex)];
        if (int(item["event"]) == int(visibleEvent) && ++restorationFrames >= unsigned(int(item["after_frames"])))
        {
            readBoard(item["board"]);
            ++restorationIndex;
            consumedEvent = eventSerial;
            restorationFrames = 0;
        }
    }
    std::string symbols = PieceSymbols;
    //场景中的棋子中心，原图像素
    cv::Point2d center;
    Detection piece;

    ++frameNumber;
    frameId = std::to_string(frameNumber);
    pieces.clear();
    for (int row = 0; row < 10; ++row)
        for (int col = 0; col < 9; ++col)
            if (world[row][col] != '.')
            {
                center = layout.pixels[(9 - row) * Columns + col];
                if (offGrid.size() == 2 && col == offGrid[0] - 'a' && 9 - row == offGrid[1] - '0')
                    center += .53 * (layout.pixels[Columns + col] - layout.pixels[col]);
                piece.classId = int(symbols.find(world[row][col]));
                piece.confidence = .9;
                piece.bbox.x = center.x - 10;
                piece.bbox.y = center.y - 10;
                piece.bbox.width = 20;
                piece.bbox.height = 20;
                pieces.push_back(piece);
            }
}

//---------预置引擎回复区---------
void Engine_query(const chess::Game &game, std::vector<std::string> &legal, std::string &choice)
{
    int index = 0;
    for (const cv::FileNode &query : script["queries"])
    {
        if ((std::string)query["history"] == game.history)
        {
            if (int(query["fail"]) == 1) throw std::runtime_error("仿真引擎查询失败");
            if (int(query["fail_once"]) == 1 && !queryUsed[index])
            {
                queryUsed[index] = true;
                throw std::runtime_error("仿真引擎暂时不可用");
            }
            if (!query["base_fen"].empty() && (std::string)query["base_fen"] != game.baseFen)
                throw std::runtime_error("仿真起始FEN不匹配");
            query["legal"] >> legal;
            choice = (std::string)query["choice"];
            if (!queryUsed[index] && !query["board_after_think"].empty()) readBoard(query["board_after_think"]);
            queryUsed[index] = true;
            return;
        }
        ++index;
    }
    throw std::runtime_error("仿真没有当前完整历史的引擎回复：" + game.history);
}

//---------预置交互区---------
void Operator_read(const std::string &phase, const std::string &, Command &result, std::string *move)
{
    cv::FileNode commands = script["commands"];
    cv::FileNode command;
    std::string name;

    result = Command::Quit;
    if (commandIndex < commands.size())
    {
        command = commands[int(commandIndex)];
        ++commandIndex;
        if ((std::string)command["phase"] != phase) throw std::runtime_error("仿真进入了意外的交互阶段：" + phase);
        if (!command["board"].empty()) readBoard(command["board"]);
        command["command"] >> name;
        if (!Command_parse(name, result, move)) throw std::runtime_error("未知仿真命令");
        //离线检查用交互记录确认实际进入的等待与恢复分支
        std::cout << "SIM_COMMAND " << phase << ' ' << name << '\n';
    }
    if (result == Command::Quit)
    {
        gameRunning = 0;
        throw std::runtime_error("对局已停止");
    }
}

//---------仿真机械执行区---------
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
    //单向打印调用参数，供离线核对动作顺序和数值
    std::cout << "SIM_CALL " << purpose << ' ' << (move.empty() ? "-" : move) << '\n' << std::setprecision(17);
    for (const PickStep &step : steps)
        std::cout << "SIM_STEP " << int(step.action) << ' ' << step.axes << ' ' << step.target[0] << ' '
                  << step.target[1] << ' ' << step.target[2] << ' ' << step.durationMs << '\n';
    bool executed = false;
    Motion_execute(steps, motionLimits, gameRunning, executed);
    std::cout << "SIM_POSE " << driveReference.position[0] << ' ' << driveReference.position[1] << ' '
              << driveReference.position[2] << ' ' << referenceCount << '\n';
    //独立场景决定落子结果，不直接复制计划盘面
    if (!event["board"].empty()) readBoard(event["board"]);
    offGrid = (std::string)event["off_grid"];
    result = int(event["success"]);
    if (result != 0 && result != 1) throw std::runtime_error("仿真执行结果必须是0或1");
    success = executed && result == 1;
    if (!success) Event_emit(EventType::Motion, "执行结果未确认，请处理设备并恢复原棋局。");
    std::cout << "SIM_RETURN " << int(success) << '\n';
}
//---------场景盘面读取区---------
static void readBoard(const cv::FileNode &rows)
{
    offGrid.clear();
    if (rows.size() != 10) throw std::runtime_error("仿真盘面必须有10行");
    for (int row = 0; row < 10; ++row)
    {
        std::string line = (std::string)rows[row];
        if (line.size() != 9) throw std::runtime_error("仿真盘面必须有9列");
        std::copy(line.begin(), line.end(), world[row].begin());
    }
}

} // namespace scara
