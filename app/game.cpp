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
static bool readEngine(Session &session, chess::Move &move);
static bool selectMove(Session &session, chess::Move &move, bool &moveByRobot);
static bool discardIsOffBoard(const Session &session);
enum class Execution { Completed, Failed, Exited };
static Execution executeMove(Session &session, const chess::Move &move, const std::vector<PickStep> &steps);

// 观察结果决定记谱、按原计划重走，或结束对局。
enum class Placement { Confirmed, OriginalRestored, Exited };
static bool observe(Session &session, const chess::Board *expected = nullptr);
static bool waitBoard(Session &session, const chess::Board &expected);
static Placement confirmPlacement(Session &session, const chess::Board &before, const chess::Board &expected);
static Placement restoreAfterDriveFailure(Session &session, const chess::Board &before);
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
    session.observed.valid = false;
    gameRunning = 1;
    auto oldInterrupt = std::signal(SIGINT, requestStop);
    auto oldTerminate = std::signal(SIGTERM, requestStop);
    try
    {
        // 安装参数已在 session 中。打开本次运行资源后，再确认起始盘面。
        Runtime_open(config, modelPath, session);
        Match_start(config, session);
        // 每次 Match_turn 最多提交一手，棋局结果决定是否进入下一轮。
        while (gameRunning && session.game.result == chess::Result::Playing)
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

static bool waitReady(const char *phase, const char *prompt)
{
    while (gameRunning)
    {
        Command command;
        Operator_read(phase, prompt, command);
        if (command == Command::Ready) return gameRunning;
    }
    return false;
}

//---------开局区---------
static void Match_start(const GameConfig &config, Session &session)
{
    if (!waitReady("prepare", "确认空吸头已抬离棋盘、手已移开，回车进入观察位。输入 quit 退出。")) return;
    // 空吸头先升到搬运高度，再回观察位，让相机看全棋盘。
    while (gameRunning && !parkForObservation(session))
        if (!waitReady("hardware_recovery", "请恢复驱动、清空吸头并确认可运动，回车继续。")) return;
    while (observe(session))
    {
        chess::Board initial = session.observed.board;
        Operator_show("confirm_start", session);
        Command command;
        Operator_read("confirm_start", "核对开局盘面，回车确认。输入 quit 退出。", command);
        if (!gameRunning) return;
        if (command != Command::Ready) continue;
        // 回车后重新看盘，防止确认时棋子已经移动。
        if (!observe(session)) return;
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
    chess::Move move;

    if (!readEngine(session, move)) return;
    chess::Game_judge(game, session.legal);
    if (game.result != chess::Result::Playing) return;

    bool moveByRobot;
    if (!selectMove(session, move, moveByRobot)) return;

    if (moveByRobot)
    {
        const chess::Board &before = game.board;
        if (!waitBoard(session, before)) return;
        chess::Board expected = before;
        chess::movePiece(expected, move);

        // 从标定表取基座 XY（毫米）。吃子还要核对弃子点是否离开棋盘。
        cv::Point2d source = session.layout.cell[move.fromRow][move.fromCol].xy;
        cv::Point2d target = session.layout.cell[move.toRow][move.toCol].xy;
        const cv::Point2d *discard = nullptr;
        if (before[move.toRow][move.toCol] != '.')
        {
            if (!discardIsOffBoard(session)) throw std::runtime_error("弃子点必须在棋盘外");
            discard = &session.capturePlaceMm;
        }
        // 取、放、弃子和回观察位都排好以后，才下发这一手。
        std::vector<PickStep> steps;
        Plan_turn(session.arm, session.heights, source, target, discard, session.park, steps);

        while (gameRunning)
        {
            Execution execution = executeMove(session, move, steps);
            if (execution == Execution::Exited) return;

            Placement result;
            if (execution == Execution::Completed) result = confirmPlacement(session, before, expected);
            else result = restoreAfterDriveFailure(session, before);
            if (result == Placement::Exited) return;
            if (result == Placement::Confirmed) break;

            session.pending = false;
            Operator_show("restored", session, "原棋局已恢复，重新执行这一手。");
        }
    }

    // 人工走棋已核对合法。机械臂代走须确认实盘已到目标局面。
    if (!gameRunning) return;
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
            Operator_read("human", "在上位机选棋，或手动走棋后回车。resign 认输，draw 和棋，quit 退出。", command, &move);
            if (!gameRunning) return false;
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
        if (!observe(session)) return false;
        chess::InferredMove inferred = chess::inferMove(game.board, session.observed, legal, move);
        if (inferred == chess::InferredMove::Unchanged) continue;
        if (inferred == chess::InferredMove::Legal) return true;
        Operator_show("illegal_move", session, "请恢复原棋局后重新走棋。");
        if (!waitBoard(session, game.board)) return false;
    }
    return false;
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

//---------机械执行区---------
static Execution executeMove(Session &session, const chess::Move &move, const std::vector<PickStep> &steps)
{
    // 从下发到落点确认期间，pending 告诉上位机这手棋仍待核实。
    session.pending = true;
    Operator_show("executing", session);
    if (!gameRunning) return Execution::Exited;

    bool completed = false;
    try
    {
        completed = Robot_execute("move", move, steps);
    }
    catch (const std::exception &error)
    {
        if (gameRunning) Operator_show("hardware_recovery", session, error.what());
    }
    if (!gameRunning) return Execution::Exited;
    return completed ? Execution::Completed : Execution::Failed;
}

//---------引擎与恢复区---------
static bool readEngine(Session &session, chess::Move &move)
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
            return gameRunning;
        }
        catch (const std::exception &error)
        {
            if (!gameRunning) return false;
            Operator_show("engine_recovery", session, error.what());
            if (!waitReady("engine_recovery", "处理引擎问题后回车继续当前棋局。输入 quit 退出。")) return false;
        }
    }
    return false;
}

// 把需要核对的格号写进提示，保留相机实际交付的盘面。
static void reportBoard(const Session &session, const chess::Board *expected, const char *phase, std::string message)
{
    if (expected)
        for (int row = 0; row < Rows; ++row)
            for (int col = 0; col < Columns; ++col)
                if ((*expected)[row][col] != session.observed.board[row][col])
                    message += " (" + std::to_string(row) + "," + std::to_string(col) + ")";
    Operator_show(phase, session, message);
}

//---------观察与恢复区---------
static bool observe(Session &session, const chess::Board *expected)
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
            Vision_observe(session.layout, session.lastFrameId, pieces);
        }
        catch (const std::exception &error)
        {
            if (!gameRunning) return false;
            Operator_show("vision_recovery", session, error.what());
            if (!waitReady("vision_recovery", "恢复相机画面后回车继续。输入 quit 退出。")) return false;
            same = 0;
            previousFrame.clear();
            continue;
        }
        if (!gameRunning) return false;
        chess::Observation frame;
        Geometry_assign(session.layout, pieces, frame, !session.lastFrameId.empty());
        // 归格只看一帧。在这里用帧号排除重复，再确认三张新图的盘面相同。
        if (frame.valid && session.lastFrameId != previousFrame)
        {
            same = frame.board == previous ? same + 1 : 1;
            previous = frame.board;
        }
        if (!frame.valid) same = 0;
        previousFrame = frame.valid ? session.lastFrameId : "";
        session.observed.board = frame.board;
        session.observed.valid = same == 3;
        if (++frames % 3 == 0 && !session.observed.valid)
            reportBoard(session, expected, "unclear", "棋子偏离格点或受遮挡，请检查实盘。");
    }
    if (!gameRunning) return false;
    Operator_show("observing", session);
    return true;
}

static Placement confirmPlacement(Session &session, const chess::Board &before, const chess::Board &expected)
{
    if (!observe(session, &expected)) return Placement::Exited;
    while (session.observed.board != expected)
    {
        reportBoard(session, &expected, "placement_recovery", "请核对落点");
        if (!waitReady("placement_recovery", "修正落点后回车。若摆回走棋前的局面，回车后重走。"))
            return Placement::Exited;
        if (!observe(session, &expected)) return Placement::Exited;
        if (session.observed.board == before) return Placement::OriginalRestored;
    }
    return Placement::Confirmed;
}

static Placement restoreAfterDriveFailure(Session &session, const chess::Board &before)
{
    // 驱动中断后先处理吸头和故障。停车得到确认，才回观察位核对原棋局。
    do
    {
        Operator_show("hardware_recovery", session);
        if (!waitReady("hardware_recovery", "处理报错、清空吸头并恢复驱动后回车。程序将回观察位并核对原棋局。"))
            return Placement::Exited;
    } while (!Motion_stop() || !parkForObservation(session));

    if (!waitBoard(session, before)) return Placement::Exited;
    return Placement::OriginalRestored;
}

// 只有相机看到的整张盘面与 expected 相同，后续动作才继续。
static bool waitBoard(Session &session, const chess::Board &expected)
{
    if (!observe(session, &expected)) return false;
    if (session.observed.board == expected) return true;
    do
    {
        reportBoard(session, &expected, "restore_board", "请恢复原棋局");
        if (!observe(session, &expected)) return false;
    } while (session.observed.board != expected);
    Operator_show("restored", session, "原棋局已恢复。");
    return true;
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
