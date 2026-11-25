// 从实盘建立棋局，逐手选棋、搬子、核对落点，再把已完成的走法记入棋谱。
#include "game.hpp"
#include "motion/execute.hpp"
#include <algorithm>
#include <stdexcept>

namespace scara
{
// 对局入口先开局，再逐手处理。
static bool Match_start(const GameConfig &config, Session &session);
static void Match_turn(Session &session);
static bool parkForObservation(Session &session);
static bool playHumanMove(Session &session, const chess::MoveList &legal, chess::Move &move);
static bool carryMove(Session &session, const chess::Move &move);

static bool restoreAfterDriveFailure(Session &session);
// 主程序、取流和执行器共用这个停止标志。
volatile std::sig_atomic_t gameRunning = 1;

//---------停止请求区---------
static void requestStop(int)
{
    // 信号回调先让各处循环退出，停车和释放资源由 Match_run 收尾。
    gameRunning = 0;
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
        if (Match_start(config, session))
        {
            Operator_show(session);
            while (gameRunning && session.game.result == chess::Result::Playing)
                Match_turn(session);
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
    Operator_show(session, failure);
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
static bool Match_start(const GameConfig &config, Session &session)
{
    if (!Operator_prepare()) return false;
    // 空吸头先升到搬运高度，再回观察位，让相机看全棋盘。
    while (gameRunning && !parkForObservation(session))
        if (!Operator_waitReady("请恢复驱动、清空吸头并确认可运动，回车继续。")) return false;
    while (gameRunning && Observe_board(session))
    {
        chess::Board initial = session.observed;
        if (!Operator_confirmOpening(initial)) continue;
        if (!gameRunning) return false;
        // 回车后重新看盘，防止确认时棋子已经移动。
        if (!Observe_board(session)) return false;
        if (session.observed != initial) continue;
        // 续局保留原来的轮次和棋谱。再次看到同一盘面，才接着走。
        if (!session.game.history.empty())
        {
            if (initial == session.game.board) return true;
            if (chess::validBoard(initial))
                throw std::runtime_error("实拍盘面与已走历史不符");
        }
        // 新局以眼前这张盘面为起点，行棋方和玩家执哪一方取自配置。
        else if (chess::Game_reset(session.game, initial, config.redToMove, config.humanRed)) return true;
        Operator_show(session, "开局盘面有误，请核对棋子和朝向。");
    }
    return false;
}

//---------走一手棋---------
static void Match_turn(Session &session)
{
    chess::Game &game = session.game;
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
        if (!playHumanMove(session, legal, move)) return;
    }
    else
    {
        if (std::count(legal.begin(), legal.end(), move) != 1)
            throw std::runtime_error("走法未通过本局合法表核对");
        if (!carryMove(session, move)) return;
    }

    // 人工走棋已核对合法。机械臂代走须确认实盘已到目标局面。
    if (!gameRunning) return;
    chess::Game_commit(game, move);
    session.pending = false;
    Operator_show(session);
}

//---------选棋区---------
// 面板选棋就让机械臂代走；玩家亲手搬棋就用相机核对。
static bool playHumanMove(Session &session, const chess::MoveList &legal, chess::Move &move)
{
    chess::Game &game = session.game;
    while (gameRunning)
    {
        Operator_show(session, {}, &legal);
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
            if (std::count(legal.begin(), legal.end(), move) == 1) return carryMove(session, move);
            Operator_show(session, "这步不合法，请重新选棋。");
            continue;
        }

        // 回车后从新盘面找出唯一合法的走法。原盘没变化时继续等待。
        if (!Observe_board(session)) return false;
        if (session.observed == game.board) continue;
        if (chess::inferMove(game.board, session.observed, legal, move)) return true;
        Operator_show(session, "请恢复原棋局后重新走棋。");
        if (!Observe_original(session)) return false;
    }
    return false;
}

//---------执行并核实一手棋---------
static bool carryMove(Session &session, const chess::Move &move)
{
    if (!Observe_original(session)) return false;
    chess::Board expected = session.game.board;
    chess::movePiece(expected, move);

    // 从标定表取基座 XY（毫米）。吃子还要核对弃子点是否离开棋盘。
    cv::Point2d source = session.layout.cell[move.fromRow][move.fromCol].xy;
    cv::Point2d target = session.layout.cell[move.toRow][move.toCol].xy;
    const cv::Point2d *discard = nullptr;
    if (session.game.board[move.toRow][move.toCol] != '.')
    {
        const Geometry &board = session.layout;
        cv::Point2d cell = Geometry_gridPoint(session.capturePlaceMm - board.origin.xy, board.rowStep.xy, board.columnStep.xy);
        // 从最外侧交点起至少半格才算盘外。
        bool outside = cell.x < -0.5 || cell.x > Columns - 0.5 || cell.y < -0.5 || cell.y > Rows - 0.5;
        if (!outside) throw std::runtime_error("弃子点必须在棋盘外");
        discard = &session.capturePlaceMm;
    }
    // 取、放、弃子和回观察位都排好以后，才下发这一手。
    std::vector<PickStep> steps;
    Plan_turn(session.arm, session.heights, source, target, discard, session.park, steps);

    while (gameRunning)
    {
        session.pending = true;
        Operator_show(session);
        if (!gameRunning) return false;
        bool moved = false;
        try { moved = Robot_execute("move", move, steps); }
        catch (const std::exception &error) { Operator_show(session, error.what()); }

        if (!gameRunning) return false;
        if (moved)
        {
            if (Observe_placement(session, expected)) return true;
        }
        else if (!restoreAfterDriveFailure(session)) return false;
        if (!gameRunning) return false;
        session.pending = false;
        Operator_show(session, "原棋局已恢复，重新执行这一手。");
    }
    return false;
}

//---------驱动故障后恢复原盘区---------
static bool restoreAfterDriveFailure(Session &session)
{
    // 驱动中断后先处理吸头和故障。停车得到确认，才回观察位核对原棋局。
    do
    {
        if (!Operator_waitReady("处理报错、清空吸头并恢复驱动后回车。程序将回观察位并核对原棋局。"))
            return false;
    } while (!Motion_stop() || !parkForObservation(session));

    return Observe_original(session);
}

//---------动作准备区---------
static bool parkForObservation(Session &session)
{
    // 准备动作按顺序关断磁铁、单独抬 J1、再让 J2 和 J3 到观察位。
    std::vector<PickStep> steps = {{PickAction::Release}, {PickAction::LiftZ, {session.heights.travelDeg}}};
    Plan_move(session.park, session.heights.travelDeg, steps);
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
