// 离线对局后端。场景提供相机、引擎和驱动反馈，运行与实机共用的对局主流程。
#include "app/game.hpp"
#include "app/site.hpp"
#include "bench/legacy_site.hpp"
#include <filesystem>
#include "motion/execute.hpp"
#include <algorithm>
#include <iostream>
#include <iomanip>

namespace scara
{
//---------场景、盘面与虚拟驱动状态区---------
static cv::FileStorage script;
// world 是场景中相机能看到的棋盘。session.game.board 由正式对局在确认后更新。
static chess::Board world;
static size_t commandIndex = 0, executionIndex = 0;
static unsigned frameNumber = 0;
static std::string visiblePhase;
// 记录上一条可见提示和恢复所经过的帧数，决定何时更换场景中的实盘。
static size_t restorationIndex = 0;
static unsigned eventSerial = 0, consumedEvent = 0, restorationFrames = 0;
static std::vector<bool> queryUsed;
static std::string offGrid;
//仿真驱动使用虚拟通信时钟，仍运行正式Motion_execute的联动调度
static MotionLimits motionLimits;
static JointReference driveReference;
static uint64_t driveMs = 0, referenceCount = 0;

static void readBoard(const cv::FileNode &rows);

//---------仿真初始化与收尾区---------
void Runtime_open(const GameConfig &config, const std::string &, Session &session)
{
    readBoard(script["initial_board"]);
    // 续局先在内存棋局中重放既有走法，随后由主流程核对 world 中的实盘。
    if (!script["resume_moves"].empty())
    {
        chess::Game &game = session.game;
        chess::Game_reset(game, game.board, config.redToMove, config.humanRed);
        for (const auto &item : script["resume_moves"])
        {
            if (!chess::Game_replay(game, std::string(item))) throw std::runtime_error("无效的仿真续局走法");
        }
    }
    queryUsed.assign(script["queries"].size(), false);
    motionLimits = config.motion;
    // 按 J1、J2、J3 装入虚拟驱动的初始角度，再从零速度、零时刻开始。
    cv::FileNode initial = script["initial_joints_deg"];
    if (initial.size() != 3) throw std::runtime_error("仿真需要三个初始驱动角度");
    for (int axis = 0; axis < 3; ++axis) driveReference.position[axis] = double(initial[axis]);
    driveReference.velocity = {};
    driveMs = referenceCount = 0;
    std::cout << "SIMULATION：不会加载相机、NPU、引擎或控制硬件。\n";
}

// 测试端观察同一份内存盘面。只在进程退出时把最终结果送到测试管道。
void Operator_show(const char *phase, const Session &s, const std::string &message)
{
    std::string name = phase;
    if (!message.empty() || name == "restored")
    {
        visiblePhase = name;
        ++eventSerial;
        std::cout << "SIM_NOTICE " << phase << ' ' << message << '\n';
    }
    if (name != "finished" && name != "stopped" && name != "error" && name != "review_required") return;
    std::cout << "SIM_RESULT\t" << name << '\t' << int(s.game.result) << '\t' << s.pending
              << '\t' << s.observed.valid << '\t' << s.lastFrameId << '\t' << s.game.history << '\t';
    // 先输出记谱盘面，再输出最后一次观察，回归程序分别核对这两个结果。
    for (const auto &row : s.game.board) std::cout.write(row.data(), row.size());
    std::cout << '\t';
    for (const auto &row : s.observed.board) std::cout.write(row.data(), row.size());
    std::cout << '\t' << message << '\n';
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
// 保存本周期参考供下一次读取，referenceCount 用来对照实际下发次数。
bool Drive_write(const JointReference &reference)
{
    driveReference = reference;
    ++referenceCount;
    return true;
}
// 仿真接收磁铁开关命令，棋子落在哪里由场景指定。
bool Drive_magnet(bool)
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
        return;
    }
    //模拟人看到事件提示后恢复棋盘，应用仍需三张新图才能确认
    cv::FileNode restorations = script["restorations"];
    if (restorationIndex < restorations.size() && eventSerial != consumedEvent)
    {
        cv::FileNode item = restorations[int(restorationIndex)];
        if (std::string(item["phase"]) == visiblePhase && ++restorationFrames >= unsigned(int(item["after_frames"])))
        {
            readBoard(item["board"]);
            ++restorationIndex;
            consumedEvent = eventSerial;
            restorationFrames = 0;
        }
    }
    const std::string &symbols = chess::PieceSymbols;
    //场景中的棋子中心，原图像素
    cv::Point2d center;
    Detection piece;

    // 通常每帧使用递增编号。场景也能提供重复或空帧号，检查三帧确认流程。
    ++frameNumber;
    auto ids = script["frame_ids"];
    frameId = frameNumber <= ids.size() ? std::string(ids[int(frameNumber-1)]) : std::to_string(frameNumber);
    pieces.clear();
    for (int row = 0; row < 10; ++row)
        for (int col = 0; col < 9; ++col)
            if (world[row][col] != '.')
            {
                // 从格点像素生成模拟检测框，指定偏位时沿行方向移开 0.53 格。
                center = layout.cell[row][col].pixel;
                if (offGrid.size() == 2 && col == offGrid[0] - 'a' && 9 - row == offGrid[1] - '0')
                    center -= .53 * layout.rowStep.pixel;
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
void Engine_query(const chess::Game &game, chess::MoveList &legal, chess::Move &choice)
{
    int index = 0;
    // 用已完成的完整棋谱选取对应的引擎回合，输出数字合法表和应手。
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
            legal.clear();
            for (const auto &item : query["legal"])
            {
                chess::Move move;
                if (!chess::parseMove(std::string(item), move)) throw std::runtime_error("仿真合法走法格式错误");
                legal.push_back(move);
            }
            choice = {};
            std::string reply = query["choice"];
            if (!reply.empty() && !chess::parseMove(reply, choice)) throw std::runtime_error("仿真应手格式错误");
            // 场景可在思考结束时改动实盘，从而检验机械臂搬子前是否重新核盘。
            if (!queryUsed[index] && !query["board_after_think"].empty()) readBoard(query["board_after_think"]);
            queryUsed[index] = true;
            return;
        }
        ++index;
    }
    throw std::runtime_error("仿真没有当前完整历史的引擎回复：" + game.history);
}

//---------预置交互区---------
void Operator_read(const std::string &phase, const std::string &, Command &result, chess::Move *move)
{
    cv::FileNode commands = script["commands"];
    cv::FileNode command;
    std::string name;

    result = Command::Quit;
    if (commandIndex < commands.size())
    {
        // 依次消费预置输入，核对当前等待阶段，再解析命令或应用人工摆盘。
        command = commands[int(commandIndex)];
        ++commandIndex;
        if ((std::string)command["phase"] != phase) throw std::runtime_error("仿真进入了意外的交互阶段：" + phase);
        if (!command["board"].empty()) readBoard(command["board"]);
        command["command"] >> name;
        if (!Command_parse(name, result, move)) throw std::runtime_error("未知仿真命令");
        //离线检查用交互记录确认实际进入的等待与恢复分支
        std::cout << "SIM_COMMAND " << phase << ' ' << name << '\n';
    }
    if (result == Command::Quit) gameRunning = 0;
}

//---------仿真机械执行区---------
bool Robot_execute(const std::string &purpose, const chess::Move &move, const std::vector<PickStep> &steps)
{
    if (!gameRunning) return false;
    cv::FileNode events = script["executions"];
    cv::FileNode event;
    int result;

    if (!gameRunning) throw std::runtime_error("对局已停止");
    if (executionIndex >= events.size()) throw std::runtime_error("仿真出现了额外的机械执行请求");
    event = events[int(executionIndex)];
    ++executionIndex;
    std::string moveText;
    chess::uci(move, moveText);
    // 先核对动作所属阶段与走法，再让三轴参考按真实执行流程采样。
    if ((std::string)event["purpose"] != purpose || (std::string)event["move"] != moveText)
        throw std::runtime_error("仿真动作与预定场景不符");
    //单向打印调用参数，供离线核对动作顺序和数值
    std::cout << "SIM_CALL " << purpose << ' ' << (moveText.empty() ? "-" : moveText) << '\n' << std::setprecision(17);
    for (const PickStep &step : steps)
        std::cout << "SIM_STEP " << int(step.action) << ' ' << step.axes << ' ' << step.target[0] << ' '
                  << step.target[1] << ' ' << step.target[2] << ' ' << step.durationMs << '\n';
    bool executed = Motion_execute(steps, motionLimits, gameRunning);
    std::cout << "SIM_POSE " << driveReference.position[0] << ' ' << driveReference.position[1] << ' '
              << driveReference.position[2] << ' ' << referenceCount << '\n';
    //独立场景决定落子结果，不直接复制计划盘面
    if (!event["board"].empty()) readBoard(event["board"]);
    offGrid = (std::string)event["off_grid"];
    result = int(event["success"]);
    if (result != 0 && result != 1) throw std::runtime_error("仿真执行结果必须是0或1");
    // 轨迹执行成功且场景回报成功，这次动作才向对局报告完成。
    bool success = executed && result == 1;
    std::cout << "SIM_RETURN " << int(success) << '\n';
    return success;
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

//---------离线场景入口区---------
// 载入几何和场景脚本后，把同一份 Session 交给正式对局入口。
int main(int argc, char **argv)
{
    if (argc != 2 && argc != 3) return 2;
    try
    {
        scara::GameConfig config;
        scara::Session session;
        std::string scenario;
        if (argc == 3)
        {
            scara::Site_loadFile(argv[1], config, session);
            scenario = argv[2];
        }
        else
        {
            scara::Bench_load(argv[1], config, session);
            cv::FileStorage input(argv[1], cv::FileStorage::READ);
            auto dir = std::filesystem::u8path(argv[1]).parent_path();
            scenario = (dir / std::filesystem::u8path(std::string(input["simulation_file"]))).u8string();
        }
        if (!scara::script.open(scenario, cv::FileStorage::READ)) return 2;
        return scara::Match_run(config, session, "");
    }
    catch (const std::exception &error)
    {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
