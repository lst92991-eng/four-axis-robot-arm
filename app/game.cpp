// 从实盘建立棋局，逐手选棋、搬子、核对落点，再把已完成的走法记入棋谱。
#include "game.hpp"
#include "motion/execute.hpp"
#include <algorithm>
#include <stdexcept>

namespace scara
{
// 对局入口先开局，再逐手处理。
static bool Match_start(const StaticParams &staticParams, DynamicParams &dynamicParams);
static void Match_turn(const StaticParams &staticParams, DynamicParams &dynamicParams);
static bool parkForObservation(const StaticParams &staticParams);
static bool playHumanMove(const StaticParams &staticParams, DynamicParams &dynamicParams, const chess::MoveList &legal, chess::Move &move);
static bool carryMove(const StaticParams &staticParams, DynamicParams &dynamicParams, const chess::Move &move);

static bool restoreAfterDriveFailure(const StaticParams &staticParams, DynamicParams &dynamicParams);
// 主程序、取流和执行器共用这个停止标志。
volatile std::sig_atomic_t gameRunning = 1;

//---------停止请求区---------
static void requestStop(int)
{
    // 信号回调先让各处循环退出，停车和释放资源由 Match_run 收尾。
    gameRunning = 0;
}

//---------完整对局入口---------
int Match_run(const StaticParams &staticParams, DynamicParams &dynamicParams, const std::string &modelPath)
{
    std::string failure;
    // 第一帧到来前，90 个交点都记为未知。
    for (auto &row : dynamicParams.observed) row.fill('?');
    dynamicParams.observationStable = false;
    gameRunning = 1;
    auto oldInterrupt = std::signal(SIGINT, requestStop);
    auto oldTerminate = std::signal(SIGTERM, requestStop);
    try
    {
        // 打开本次运行资源后，再确认起始盘面。
        Runtime_open(staticParams, modelPath, dynamicParams);
        if (Match_start(staticParams, dynamicParams))
        {
            Operator_show(dynamicParams);
            while (gameRunning && dynamicParams.game.result == chess::Result::Playing)
                Match_turn(staticParams, dynamicParams);
        }
    }
    catch (const std::exception &error)
    {
        if (gameRunning) failure = std::string(error.what()) + "。请排除故障后重新开局。";
    }
    // 正常终局、人工退出和异常退出都要确认设备收尾。
    if (!Runtime_close())
    {
        if (!failure.empty()) failure += "。";
        failure += "机械停止或资源回收未确认";
    }
    if (oldInterrupt != SIG_ERR) std::signal(SIGINT, oldInterrupt);
    if (oldTerminate != SIG_ERR) std::signal(SIGTERM, oldTerminate);
    Operator_show(dynamicParams, failure);
    return failure.empty() ? 0 : 1;
}

bool Operator_waitReady(const char *prompt)
{
    while (gameRunning)
    {
        if (Operator_read(prompt) == Command::Ready) return gameRunning;
    }
    return false;
}

//---------开局区---------
static bool Match_start(const StaticParams &staticParams, DynamicParams &dynamicParams)
{
    if (!Operator_prepare()) return false;
    // 空吸头先升到搬运高度，再回观察位，让相机看全棋盘。
    while (gameRunning && !parkForObservation(staticParams))
        if (!Operator_waitReady("请恢复驱动、清空吸头并确认可运动，回车继续。")) return false;
    while (gameRunning && Observe_board(staticParams.layout, dynamicParams))
    {
        chess::Board initial = dynamicParams.observed;
        if (!Operator_confirmOpening(initial)) continue;
        if (!gameRunning) return false;
        // 回车后重新看盘，防止确认时棋子已经移动。
        if (!Observe_board(staticParams.layout, dynamicParams)) return false;
        if (dynamicParams.observed != initial) continue;
        // 续局保留原来的轮次和棋谱。再次看到同一盘面，才接着走。
        if (!dynamicParams.game.history.empty())
        {
            if (initial == dynamicParams.game.board) return true;
            if (chess::validBoard(initial))
                throw std::runtime_error("实拍盘面与已走历史不符");
        }
        // 新局以眼前这张盘面为起点，行棋方和玩家执哪一方取自配置。
        else if (chess::Game_reset(dynamicParams.game, initial, staticParams.redToMove, staticParams.humanRed)) return true;
        Operator_show(dynamicParams, "开局盘面有误，请核对棋子和朝向。");
    }
    return false;
}

//---------走一手棋---------
static void Match_turn(const StaticParams &staticParams, DynamicParams &dynamicParams)
{
    chess::Game &game = dynamicParams.game;
    chess::MoveList legal;
    chess::Move move;

    Engine_query(game, legal, move);
    if (!gameRunning) return;
    for (const chess::Move &candidate : legal)
        if (!chess::validTransfer(game.board, game.redToMove, candidate))
            throw std::runtime_error("引擎走法与当前盘面不符");
    chess::Game_judge(game, legal);
    if (game.result != chess::Result::Playing) return;

    // 电脑直接用引擎应手。只有轮到玩家时才读操作台或等待人工搬棋。
    if (game.redToMove == game.humanRed)
    {
        if (!playHumanMove(staticParams, dynamicParams, legal, move)) return;
    }
    else
    {
        if (std::count(legal.begin(), legal.end(), move) != 1)
            throw std::runtime_error("走法未通过本局合法表核对");
        if (!carryMove(staticParams, dynamicParams, move)) return;
    }

    // 人工走棋已核对合法。机械臂代走须确认实盘已到目标局面。
    if (!gameRunning) return;
    chess::Game_commit(game, move);
    dynamicParams.pending = false;
    Operator_show(dynamicParams);
}

//---------选棋区---------
// 面板选棋就让机械臂代走；玩家亲手搬棋就用相机核对。
static bool playHumanMove(const StaticParams &staticParams, DynamicParams &dynamicParams, const chess::MoveList &legal, chess::Move &move)
{
    chess::Game &game = dynamicParams.game;
    while (gameRunning)
    {
        Operator_show(dynamicParams, {}, &legal);
        Command command = Operator_read("在上位机选棋，或手动走棋后回车。resign 认输，draw 和棋，quit 退出。", &move);
        if (!gameRunning) return false;
        if (command == Command::Draw)
        {
            game.result = chess::Result::Draw;
            return false;
        }
        if (command == Command::Resign)
        {
            game.result = game.humanRed ? chess::Result::BlackWon : chess::Result::RedWon;
            return false;
        }
        if (command == Command::Move)
        {
            if (std::count(legal.begin(), legal.end(), move) == 1) return carryMove(staticParams, dynamicParams, move);
            Operator_show(dynamicParams, "这步不合法，请重新选棋。");
            continue;
        }

        // 回车后从新盘面找出唯一合法的走法。原盘没变化时继续等待。
        if (!Observe_board(staticParams.layout, dynamicParams)) return false;
        if (dynamicParams.observed == game.board) continue;
        if (chess::inferMove(game.board, dynamicParams.observed, legal, move)) return true;
        Operator_show(dynamicParams, "请恢复原棋局后重新走棋。");
        if (!Observe_original(staticParams.layout, dynamicParams)) return false;
    }
    return false;
}

//---------执行并核实一手棋---------
static bool carryMove(const StaticParams &staticParams, DynamicParams &dynamicParams, const chess::Move &move)
{
    if (!Observe_original(staticParams.layout, dynamicParams)) return false;
    chess::Board expected = dynamicParams.game.board;
    chess::movePiece(expected, move);

    // 从标定表取基座 XY（毫米）。吃子还要核对弃子点是否离开棋盘。
    cv::Point2d source = staticParams.layout.cell[move.fromRow][move.fromCol].xy;
    cv::Point2d target = staticParams.layout.cell[move.toRow][move.toCol].xy;
    const cv::Point2d *discard = nullptr;
    if (dynamicParams.game.board[move.toRow][move.toCol] != '.')
    {
        const Geometry &board = staticParams.layout;
        cv::Point2d cell = Geometry_gridPoint(staticParams.capturePlaceMm - board.origin.xy, board.rowStep.xy, board.columnStep.xy);
        // 从最外侧交点起至少半格才算盘外。
        bool outside = cell.x < -0.5 || cell.x > Columns - 0.5 || cell.y < -0.5 || cell.y > Rows - 0.5;
        if (!outside) throw std::runtime_error("弃子点必须在棋盘外");
        discard = &staticParams.capturePlaceMm;
    }
    // 取、放、弃子和回观察位都排好以后，才下发这一手。
    std::vector<PickStep> steps;
    Plan_turn(staticParams.arm, staticParams.heights, source, target, discard, staticParams.park, steps);

    while (gameRunning)
    {
        dynamicParams.pending = true;
        Operator_show(dynamicParams);
        if (!gameRunning) return false;
        bool moved = false;
        try { moved = Robot_execute("move", move, steps); }
        catch (const std::exception &error) { Operator_show(dynamicParams, error.what()); }

        if (!gameRunning) return false;
        if (moved)
        {
            if (Observe_placement(staticParams.layout, dynamicParams, expected)) return true;
        }
        else if (!restoreAfterDriveFailure(staticParams, dynamicParams)) return false;
        if (!gameRunning) return false;
        dynamicParams.pending = false;
        Operator_show(dynamicParams, "原棋局已恢复，重新执行这一手。");
    }
    return false;
}

//---------驱动故障后恢复原盘区---------
static bool restoreAfterDriveFailure(const StaticParams &staticParams, DynamicParams &dynamicParams)
{
    // 驱动中断后先处理吸头和故障。停车得到确认，才回观察位核对原棋局。
    do
    {
        if (!Operator_waitReady("处理报错、清空吸头并恢复驱动后回车。程序将回观察位并核对原棋局。"))
            return false;
    } while (!Motion_stop() || !parkForObservation(staticParams));

    return Observe_original(staticParams.layout, dynamicParams);
}

//---------动作准备区---------
static bool parkForObservation(const StaticParams &staticParams)
{
    // 准备动作按顺序关断磁铁、单独抬 J1、再让 J2 和 J3 到观察位。
    std::vector<PickStep> steps = {{PickAction::Release}, {PickAction::LiftZ, {staticParams.heights.travelDeg}}};
    Plan_move(staticParams.park, staticParams.heights.travelDeg, steps);
    return Robot_execute("prepare", {}, steps);
}

//---------终端命令区---------
// 把面板或终端的一行输入解释为命令，选棋时另写出数字起点和终点。
Command Command_parse(const std::string &text, chess::Move *move)
{
    if (move) *move = {};
    if (text.empty() || text == "ready") return Command::Ready;
    if (text == "resign") return Command::Resign;
    if (text == "draw") return Command::Draw;
    if (text == "quit") return Command::Quit;
    if (!move) return Command::Invalid;

    // 面板送来“6,0 5,0”，旧棋谱写作“a3a4”。
    if (text.size() == 7 && text[1] == ',' && text[3] == ' ' && text[5] == ',')
    {
        chess::Move selected{text[0]-'0', text[2]-'0', text[4]-'0', text[6]-'0'};
        if (!chess::validSquares(selected)) return Command::Invalid;
        *move = selected;
        return Command::Move;
    }
    return chess::parseMove(text, *move) ? Command::Move : Command::Invalid;
}
} // namespace scara
