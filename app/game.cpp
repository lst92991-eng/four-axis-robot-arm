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
static bool readEngine(Session &session, chess::MoveList &legal, chess::Move &move);
static bool selectHumanMove(Session &session, const chess::MoveList &legal, chess::Move &move, bool &moveByRobot);
static bool discardIsOffBoard(const Session &session);
static bool executeMove(Session &session, const chess::Move &move, const std::vector<PickStep> &steps);

// 观察结果决定记谱、按原计划重走，或结束对局。
enum class Placement { Confirmed, OriginalRestored, Exited };
static bool observe(Session &session, const chess::Board *expected = nullptr);
static bool waitBoard(Session &session, const chess::Board &expected);
static Placement confirmPlacement(Session &session, const chess::Board &before, const chess::Board &expected);
static bool restoreAfterDriveFailure(Session &session, const chess::Board &before);
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
    for (auto &row : session.observed) row.fill('?');
    session.observationStable = false;
    gameRunning = 1;
    auto oldInterrupt = std::signal(SIGINT, requestStop);
    auto oldTerminate = std::signal(SIGTERM, requestStop);
    try
    {
        // 安装参数已在 session 中。打开本次运行资源后，再确认起始盘面。
        Runtime_open(config, modelPath, session);
        Match_start(config, session);
        if (gameRunning) Operator_show(session);
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
    Operator_show(session, failure);
    return failure.empty() ? 0 : 1;
}

static bool waitReady(const char *prompt)
{
    while (gameRunning)
    {
        Command command;
        Operator_read(prompt, command);
        if (command == Command::Ready) return gameRunning;
    }
    return false;
}

//---------开局区---------
static void Match_start(const GameConfig &config, Session &session)
{
    if (!Operator_prepare()) return;
    // 空吸头先升到搬运高度，再回观察位，让相机看全棋盘。
    while (gameRunning && !parkForObservation(session))
        if (!waitReady("请恢复驱动、清空吸头并确认可运动，回车继续。")) return;
    while (observe(session))
    {
        chess::Board initial = session.observed;
        bool approved = Operator_confirmOpening(initial);
        if (!gameRunning) return;
        if (!approved) continue;
        // 回车后重新看盘，防止确认时棋子已经移动。
        if (!observe(session)) return;
        if (session.observed != initial) continue;
        // 续局保留原来的轮次和棋谱。再次看到同一盘面，才接着走。
        if (!session.game.history.empty())
        {
            if (initial == session.game.board) return;
            if (chess::validBoard(initial))
                throw std::runtime_error("实拍盘面与已走历史不符");
        }
        // 新局以眼前这张盘面为起点，行棋方和玩家执哪一方取自配置。
        else if (chess::Game_reset(session.game, initial, config.redToMove, config.humanRed)) return;
        Operator_show(session, "开局盘面有误，请核对棋子和朝向。");
    }
}

//---------走一手棋---------
static void Match_turn(Session &session)
{
    chess::Game &game = session.game;
    chess::MoveList legal;
    chess::Move move;

    if (!readEngine(session, legal, move)) return;
    chess::Game_judge(game, legal);
    if (game.result != chess::Result::Playing) return;

    // 电脑直接用引擎应手。只有轮到玩家时才读操作台或等待人工搬棋。
    bool moveByRobot = true;
    if (game.redToMove == game.humanRed)
    {
        if (!selectHumanMove(session, legal, move, moveByRobot)) return;
    }
    else if (std::count(legal.begin(), legal.end(), move) != 1)
        throw std::runtime_error("走法未通过本局合法表核对");

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
            bool completed = executeMove(session, move, steps);
            if (!gameRunning) return;

            if (completed)
            {
                Placement result = confirmPlacement(session, before, expected);
                if (result == Placement::Exited) return;
                if (result == Placement::Confirmed) break;
            }
            else if (!restoreAfterDriveFailure(session, before)) return;

            session.pending = false;
            Operator_show(session, "原棋局已恢复，重新执行这一手。");
        }
    }

    // 人工走棋已核对合法。机械臂代走须确认实盘已到目标局面。
    if (!gameRunning) return;
    chess::Game_commit(game, move);
    session.pending = false;
    Operator_show(session);
}

//---------选棋区---------
// 写出合法的走法和搬棋方式。认输或和棋时记录整局结果。
static bool selectHumanMove(Session &session, const chess::MoveList &legal, chess::Move &move, bool &moveByRobot)
{
    chess::Game &game = session.game;
    while (gameRunning)
    {
        Command command;
        Operator_show(session, {}, &legal);
        Operator_read("在上位机选棋，或手动走棋后回车。resign 认输，draw 和棋，quit 退出。", command, &move);
        if (!gameRunning) return false;
        if (command == Command::Draw || command == Command::Resign)
        {
            game.result = chess::Result::Draw;
            if (command == Command::Resign)
                game.result = game.humanRed ? chess::Result::BlackWon : chess::Result::RedWon;
            return false;
        }

        if (command == Command::Move)
        {
            if (std::count(legal.begin(), legal.end(), move) == 1) return true;
            Operator_show(session, "这步不合法，请重新选棋。");
            continue;
        }

        // 回车后从新盘面找出唯一合法的走法。原盘没变化时继续等待。
        if (!observe(session)) return false;
        chess::InferredMove inferred = chess::inferMove(game.board, session.observed, legal, move);
        if (inferred == chess::InferredMove::Unchanged) continue;
        if (inferred == chess::InferredMove::Legal)
        {
            moveByRobot = false;
            return true;
        }
        Operator_show(session, "请恢复原棋局后重新走棋。");
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
static bool executeMove(Session &session, const chess::Move &move, const std::vector<PickStep> &steps)
{
    // 从下发到落点确认期间，pending 告诉上位机这手棋仍待核实。
    session.pending = true;
    Operator_show(session);
    if (!gameRunning) return false;

    try
    {
        return Robot_execute("move", move, steps);
    }
    catch (const std::exception &error)
    {
        if (gameRunning) Operator_show(session, error.what());
        return false;
    }
}

//---------引擎与恢复区---------
static bool readEngine(Session &session, chess::MoveList &legal, chess::Move &move)
{
    while (gameRunning)
    {
        try
        {
            // 引擎写回合法走法和本轮应手，先核对候选的坐标、起点棋子和目标格。
            Engine_query(session.game, legal, move);
            for (const chess::Move &candidate : legal)
                if (!chess::validTransfer(session.game.board, session.game.redToMove, candidate))
                    throw std::runtime_error("引擎走法与当前盘面不符");
            return gameRunning;
        }
        catch (const std::exception &error)
        {
            if (!gameRunning) return false;
            Operator_show(session, error.what());
            if (!waitReady("处理引擎问题后回车继续当前棋局。输入 quit 退出。")) return false;
        }
    }
    return false;
}

// 把需要核对的格号写进提示，保留相机实际交付的盘面。
static void reportBoard(const Session &session, const chess::Board *expected, std::string message)
{
    if (expected)
        for (int row = 0; row < Rows; ++row)
            for (int col = 0; col < Columns; ++col)
                if ((*expected)[row][col] != session.observed[row][col])
                    message += " (" + std::to_string(row) + "," + std::to_string(col) + ")";
    Operator_show(session, message);
}

//---------观察与恢复区---------
static bool observe(Session &session, const chess::Board *expected)
{
    // same 计连续相同的新帧，frames 计取图次数，用于控制未看清时的提示频率。
    int same = 0, frames = 0;
    session.observationStable = false;
    Operator_show(session);
    while (gameRunning && same < 3)
    {
        std::string frameId;
        std::vector<Detection> pieces;
        try
        {
            Vision_observe(session.layout, frameId, pieces);
        }
        catch (const std::exception &error)
        {
            if (!gameRunning) return false;
            Operator_show(session, error.what());
            if (!waitReady("恢复相机画面后回车继续。输入 quit 退出。")) return false;
            same = 0;
            continue;
        }
        if (!gameRunning) return false;
        chess::Board frame;
        bool frameValid = Geometry_assign(session.layout, pieces, frame, !frameId.empty());
        // 复用已经记录的帧号和盘面。同一有效帧不会重复计数或覆盖盘面。
        if (!frameValid || same == 0 || frameId != session.lastFrameId)
        {
            if (!frameValid) same = 0;
            else same = frame == session.observed ? same + 1 : 1;
            session.observed = frame;
        }
        session.lastFrameId = frameId;
        session.observationStable = same == 3;
        if (++frames % 3 == 0 && !session.observationStable)
            reportBoard(session, expected, "棋子偏离格点或受遮挡，请检查实盘。");
    }
    if (!gameRunning) return false;
    Operator_show(session);
    return true;
}

static Placement confirmPlacement(Session &session, const chess::Board &before, const chess::Board &expected)
{
    if (!observe(session, &expected)) return Placement::Exited;
    while (session.observed != expected)
    {
        reportBoard(session, &expected, "请核对落点");
        if (!waitReady("修正落点后回车。若摆回走棋前的局面，回车后重走。"))
            return Placement::Exited;
        if (!observe(session, &expected)) return Placement::Exited;
        if (session.observed == before) return Placement::OriginalRestored;
    }
    return Placement::Confirmed;
}

static bool restoreAfterDriveFailure(Session &session, const chess::Board &before)
{
    // 驱动中断后先处理吸头和故障。停车得到确认，才回观察位核对原棋局。
    do
    {
        if (!waitReady("处理报错、清空吸头并恢复驱动后回车。程序将回观察位并核对原棋局。"))
            return false;
    } while (!Motion_stop() || !parkForObservation(session));

    return waitBoard(session, before);
}

// 只有相机看到的整张盘面与 expected 相同，后续动作才继续。
static bool waitBoard(Session &session, const chess::Board &expected)
{
    if (!observe(session, &expected)) return false;
    if (session.observed == expected) return true;
    do
    {
        reportBoard(session, &expected, "请恢复原棋局");
        if (!observe(session, &expected)) return false;
    } while (session.observed != expected);
    Operator_show(session, "原棋局已恢复。");
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
