// 从实盘建立棋局，逐手选棋、搬子、核对落点，再把已完成的走法记入棋谱。
#include "game.hpp"
#include "motion/execute.hpp"
#include "vision/capture.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace scara
{
// 对局入口先开局，再逐手处理。
static void Match_start(const GameConfig &config, Session &session);
static void Match_turn(Session &session);
static bool parkForObservation(Session &session);
static void readEngine(Session &session, chess::Move &move);
static bool selectMove(Session &session, chess::Move &move, bool &moveByRobot);
static void carry(Session &session, const chess::Move &move);

// 观察实盘、核对整盘和确认落点。
static void observe(Session &session, const chess::Board *expected = nullptr);
static void waitBoard(Session &session, const chess::Board &expected);
static bool confirmPlacement(Session &session, const chess::Board &expected);
// 主程序、取流和执行器共用这个停止标志。
volatile std::sig_atomic_t gameRunning = 1;

//---------停止请求区---------
static void requestStop(int)
{
    // 信号回调先让各处循环退出，停车和释放资源由 Match_run 收尾。
    gameRunning = 0;
    Camera_stop();
}

//---------完整对局入口---------
int Match_run(const GameConfig &config, Session &session, const std::string &modelPath)
{
    std::string failure;
    // 第一帧到来前，90 个交点都记为未知。
    for (auto &row : session.observed.board) row.fill('?');
    gameRunning = 1;
    auto oldInterrupt = std::signal(SIGINT, requestStop);
    auto oldTerminate = std::signal(SIGTERM, requestStop);
    try
    {
        // 安装参数已在 session 中。打开本次运行资源后，再确认起始盘面。
        Runtime_open(config, modelPath, session);
        Match_start(config, session);
        // 每次 Match_turn 最多提交一手，棋局结果决定是否进入下一轮。
        while (session.game.result == chess::Result::Playing)
            Match_turn(session);
    }
    catch (const std::exception &error)
    {
        if (gameRunning) failure = error.what();
    }
    // 正常终局、人工退出和异常退出都要确认设备收尾。
    if (!Runtime_close())
    {
        if (!failure.empty()) failure += "。";
        failure += "机械停止或资源回收未确认";
    }
    if (oldInterrupt != SIG_ERR) std::signal(SIGINT, oldInterrupt);
    if (oldTerminate != SIG_ERR) std::signal(SIGTERM, oldTerminate);
    // 把棋局结果和本次运行的失败原因一起交给显示端。
    const char *phase = "finished";
    if (!failure.empty()) phase = "error";
    else if (session.game.result == chess::Result::Playing) phase = "stopped";
    else if (session.game.result == chess::Result::ReviewRequired) phase = "review_required";
    Operator_show(phase, session, failure);
    return failure.empty() ? 0 : 1;
}

// 这些提示只等回车。quit 和断开输入都由 Operator_read 统一结束对局。
static void waitReady(const char *phase, const char *prompt)
{
    Command command;
    do
    {
        Operator_read(phase, prompt, command);
    } while (command != Command::Ready);
}

//---------开局区---------
static void Match_start(const GameConfig &config, Session &session)
{
    waitReady("prepare", "确认空吸头已抬离棋盘、手已移开，回车进入观察位；quit退出。");
    // 空吸头先升到搬运高度，再回观察位，让相机看全棋盘。
    while (!parkForObservation(session))
        waitReady("hardware_recovery", "请恢复驱动、清空吸头并确认可运动，回车继续。");
    while (gameRunning)
    {
        // 保存操作者即将核对的盘面。按回车后再取一组新帧，两次相同才采纳。
        observe(session);
        chess::Board initial = session.observed.board;
        Operator_show("confirm_start", session);
        Command command;
        Operator_read("confirm_start", "核对开局盘面，回车确认；quit退出。", command);
        if (command != Command::Ready) continue;
        observe(session);
        if (session.observed.board != initial) continue;
        // 续局保留原来的轮次和棋谱。再次看到同一盘面，才接着走。
        if (!session.game.history.empty())
        {
            if (initial == session.game.board) return;
            if (chess::validBoard(initial))
                throw std::runtime_error("实拍盘面与已走历史不符");
        }
        // 新局以眼前这张盘面为起点，行棋方和玩家执哪一方取自配置。
        else if (chess::Game_reset(session.game, initial, config.redToMove, config.humanRed)) return;
        Operator_show("confirm_start", session, "开局盘面有误，请核对棋子和朝向。");
    }
}

//---------走一手棋---------
static void Match_turn(Session &session)
{
    chess::Game &game = session.game;
    // 同一个 move 保存选中的起点和终点，最后用它记谱。
    chess::Move move;

    // 第一步，查询合法走法，再根据棋局结果决定是否继续。
    readEngine(session, move);
    chess::Game_judge(game, session.legal);
    if (game.result != chess::Result::Playing) return;

    // 第二步，确定这一手，并记录是否需要机械臂代走。
    bool moveByRobot;
    if (!selectMove(session, move, moveByRobot)) return;

    // 第三步，按起点和终点搬棋，核对落点后返回。
    if (moveByRobot) carry(session, move);

    // 第四步，实盘已确认，把走法写入棋谱并换边。
    chess::Game_commit(game, move);
    session.pending = false;
    Operator_show("confirmed", session);
}

//---------选棋区---------
// 写出合法的走法和搬棋方式。认输或和棋时记录整局结果。
static bool selectMove(Session &session, chess::Move &move, bool &moveByRobot)
{
    chess::Game &game = session.game;
    const chess::MoveList &legal = session.legal;
    bool human = game.redToMove == game.humanRed;
    while (gameRunning)
    {
        // 电脑使用引擎应手。玩家可以在面板选棋，也可以亲手搬棋后回车。
        Command command = Command::Move;
        if (human)
        {
            Operator_show("wait_human", session);
            Operator_read("human", "在上位机选棋，或手动走棋后回车；resign认输，draw和棋，quit退出。", command, &move);
        }
        if (command == Command::Draw || command == Command::Resign)
        {
            game.result = chess::Result::Draw;
            if (command == Command::Resign)
                game.result = game.humanRed ? chess::Result::BlackWon : chess::Result::RedWon;
            return false;
        }

        moveByRobot = command == Command::Move;
        if (moveByRobot)
        {
            if (std::count(legal.begin(), legal.end(), move) == 1) return true;
            if (!human) throw std::runtime_error("走法未通过本局合法表核对");
            Operator_show("illegal_move", session, "这步不合法，请重新选棋。");
            continue;
        }

        // 回车后从新盘面找出唯一合法的走法。原盘没变化时继续等待。
        observe(session);
        if (session.observed.board == game.board) continue;
        if (chess::inferMove(game.board, session.observed, legal, move)) return true;
        Operator_show("illegal_move", session, "请恢复原棋局后重新走棋。");
        waitBoard(session, game.board);
    }
    throw std::runtime_error("对局已停止");
}

//---------弃子点核对区---------
static bool discardIsOffBoard(const Session &session)
{
    const Geometry &board = session.layout;
    if (std::abs(board.columnStep.xy.cross(board.rowStep.xy)) < 1e-6) return false;
    double row, column;
    // 把弃子点的基座毫米换成棋盘行列，才能按半格宽度判断是否已到盘外。
    Geometry_rowCol(session.capturePlaceMm - board.origin.xy, board.rowStep.xy, board.columnStep.xy, row, column);
    //在外侧格线外至少留半格，避免从搬运高度把棋子放到棋盘边。
    return column < -.5 || column > Columns - .5 || row < -.5 || row > Rows - .5;
}

//---------机械搬棋区---------
static void carry(Session &session, const chess::Move &move)
{
    // 动手前重新看盘，确认思考或选棋期间棋子仍在原位。
    const chess::Board &before = session.game.board;
    waitBoard(session, before);
    // 在盘面副本上先走一步，取放后按这张盘面确认落点。
    chess::Board expected = before;
    chess::movePiece(expected, move);

    const ArmGeometry &arm = session.arm;
    const WorkHeights &z = session.heights;
    // 从标定表取起点和终点的基座 XY，单位为毫米。steps 按执行先后存动作。
    cv::Point2d source = session.layout.cell[move.fromRow][move.fromCol].xy;
    cv::Point2d target = session.layout.cell[move.toRow][move.toCol].xy;
    std::vector<PickStep> steps;

    // 吃子时先腾出目标格。普通走子从后面的取起点开始。
    if (before[move.toRow][move.toCol] != '.')
    {
        if (!discardIsOffBoard(session)) throw std::runtime_error("弃子点必须在棋盘外");
        Plan_pick(arm, z, target, steps);
        Plan_discard(arm, z, session.capturePlaceMm, steps);
    }
    // 目标格腾空后，把进攻子从起点搬到终点，再回观察位核盘。
    Plan_pick(arm, z, source, steps);
    Plan_place(arm, z, target, steps);
    Plan_move(session.park, z.travelDeg, steps);

    while (gameRunning)
    {
        // 从下发到落点确认期间，pending 告诉上位机这手棋仍待核实。
        session.pending = true;
        Operator_show("executing", session);
        bool completed = false;
        try
        {
            completed = Robot_execute("move", move, steps);
        }
        catch (const std::exception &error)
        {
            if (!gameRunning) throw;
            Operator_show("hardware_recovery", session, error.what());
        }
        // 驱动报完成后还要用相机核对这手棋。
        if (completed && confirmPlacement(session, expected)) return;
        if (!completed)
        {
            // 设备中断后先等人工处理，再停车、回观察位，并确认已恢复走棋前的盘面。
            do
            {
                Operator_show("hardware_recovery", session);
                waitReady("hardware_recovery", "处理报错、清空吸头并恢复驱动后回车；程序将回观察位并核对原棋局。");
            } while (!Motion_stop() || !parkForObservation(session));
            waitBoard(session, before);
        }
        session.pending = false;
        Operator_show("restored", session, "原棋局已恢复，重新执行这一手。");
    }
    throw std::runtime_error("对局已停止");
}

//---------引擎与恢复区---------
static void readEngine(Session &session, chess::Move &move)
{
    while (gameRunning)
    {
        Operator_show("thinking", session);
        try
        {
            // 引擎写回合法走法和本轮应手，先核对候选的坐标、起点棋子和目标格。
            Engine_query(session.game, session.legal, move);
            for (const chess::Move &candidate : session.legal)
                if (!chess::validTransfer(session.game.board, session.game.redToMove, candidate))
                    throw std::runtime_error("引擎走法与当前盘面不符");
            return;
        }
        catch (const std::exception &error)
        {
            if (!gameRunning) throw;
            Operator_show("engine_recovery", session, error.what());
            waitReady("engine_recovery", "处理引擎问题后回车继续当前棋局；quit退出。");
        }
    }
    throw std::runtime_error("对局已停止");
}

// 把有差别的格号写进提示。未看清的空位显示为问号。
static void reportBoard(Session &session, const chess::Board *expected, const char *phase, std::string message)
{
    if (expected)
        for (int row = 0; row < Rows; ++row)
            for (int col = 0; col < Columns; ++col)
            {
                char &seen = session.observed.board[row][col];
                if ((*expected)[row][col] == seen) continue;
                if (!session.observed.valid && seen == '.') seen = '?';
                message += " (" + std::to_string(row) + "," + std::to_string(col) + ")";
            }
    Operator_show(phase, session, message);
}

//---------观察与恢复区---------
static void observe(Session &session, const chess::Board *expected)
{
    // previous 保存上一张有效新帧的盘面，previousFrame 保存它的帧号。
    chess::Board previous{};
    std::string previousFrame;
    // same 计连续相同的新帧，frames 计取图次数，用于控制未看清时的提示频率。
    int same = 0, frames = 0;
    session.observed.valid = false;
    Operator_show("observing", session);
    while (gameRunning && same < 3)
    {
        std::vector<Detection> pieces;
        try
        {
            // 先取得检测框、帧号与预览，再用检测框中心给交点填子。
            Vision_observe(session.layout, session.lastFrameId, pieces, session.preview);
        }
        catch (const std::exception &error)
        {
            if (!gameRunning) throw;
            Operator_show("vision_recovery", session, error.what());
            waitReady("vision_recovery", "恢复相机画面后回车继续；quit退出。");
            same = 0;
            previousFrame.clear();
            continue;
        }
        Geometry_assign(session.layout, pieces, session.observed, !session.lastFrameId.empty());
        // 归格只看一帧。在这里用帧号排除重复，再确认三张新图的盘面相同。
        if (!session.observed.valid) same = 0;
        else if (session.lastFrameId != previousFrame)
        {
            same = session.observed.board == previous ? same + 1 : 1;
            previous = session.observed.board;
        }
        // 进入棋局前，把单帧归格的 valid 更新为三帧确认的结果。
        previousFrame = session.observed.valid ? session.lastFrameId : "";
        session.observed.valid = same == 3;
        if (++frames % 3 == 0 && !session.observed.valid)
            reportBoard(session, expected, "unclear", "棋子偏离格点或受遮挡，请检查实盘。");
    }
    if (!gameRunning) throw std::runtime_error("对局已停止");
    Operator_show("observing", session);
}

// 返回 true 表示棋子已落对。人工确认后若棋盘恢复到走棋前，返回 false 让机械臂重试。
static bool confirmPlacement(Session &session, const chess::Board &expected)
{
    observe(session, &expected);
    while (session.observed.board != expected)
    {
        reportBoard(session, &expected, "placement_recovery", "请核对落点");
        waitReady("placement_recovery", "修正落点后回车。若摆回走棋前的局面，回车后重走。");
        observe(session, &expected);
        if (session.observed.board == session.game.board) return false;
    }
    return true;
}

// 只有相机看到的整张盘面与 expected 相同，后续动作才继续。
static void waitBoard(Session &session, const chess::Board &expected)
{
    observe(session, &expected);
    if (session.observed.board == expected) return;
    do
    {
        reportBoard(session, &expected, "restore_board", "请恢复原棋局");
        observe(session, &expected);
    } while (session.observed.board != expected);
    Operator_show("restored", session, "原棋局已恢复。");
}

//---------动作准备区---------
static bool parkForObservation(Session &session)
{
    // 准备动作按顺序关断磁铁、单独抬 J1、再让 J2 和 J3 到观察位。
    PickStep lift;
    lift.action = PickAction::Move;
    lift.axes = ZAxis;
    lift.target[0] = session.heights.travelDeg;
    std::vector<PickStep> steps = {{PickAction::Release}, lift};
    Plan_move(session.park, session.heights.travelDeg, steps);
    return Robot_execute("prepare", {}, steps);
}

//---------终端命令区---------
// 把面板或终端的一行输入解释为命令，选棋时另写出数字起点和终点。
bool Command_parse(const std::string &text, Command &command, chess::Move *move)
{
    command = Command::Quit;
    if (move) *move = {};
    if (text.empty() || text == "ready") command = Command::Ready;
    else if (text == "resign") command = Command::Resign;
    else if (text == "draw") command = Command::Draw;
    else if (text != "quit")
    {
        if (!move) return false;
        chess::Move selected;
        // 上位机传“6,0 5,0”。同一入口也可读旧棋谱的“a3a4”。
        if (text.size() == 7 && text[1] == ',' && text[3] == ' ' && text[5] == ',')
        {
            selected.fromRow = text[0] - '0';
            selected.fromCol = text[2] - '0';
            selected.toRow = text[4] - '0';
            selected.toCol = text[6] - '0';
            if (!chess::validSquares(selected)) return false;
        }
        else if (!chess::parseMove(text, selected)) return false;
        command = Command::Move;
        *move = selected;
    }
    return true;
}
} // namespace scara
