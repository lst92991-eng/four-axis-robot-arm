//正常对局按开局、玩家回合、机器人回合推进；异常通知显示并等待恢复
#include "game.hpp"
#include "motion/execute.hpp"
#include <algorithm>
#include <iostream>
#include <stdexcept>

namespace scara
{
static void observe(Session &session);
static void waitBoard(Session &session, const chess::Board &expected, EventType event);
static bool prepare(const GameConfig &config, Session &session);
static void planMove(const Session &session, const chess::Move &move, std::vector<PickStep> &steps);

//---------开局区---------
void Match_start(const GameConfig &config, Session &session)
{
    Command command;
    do
    {
        Operator_read("prepare", "确认空吸头已抬离棋盘、手已移开，回车进入观察位；quit退出。", command);
    }
    while (command != Command::Ready);
    while (!prepare(config, session))
        Operator_read("hardware_recovery", "请恢复驱动、清空吸头并确认可运动，回车继续。", command);
    while (gameRunning)
    {
        observe(session);
        chess::Board initial = session.observed.board;
        for (const auto &row : initial) std::cout << std::string(row.begin(), row.end()) << '\n';
        Operator_read("confirm_start", "核对开局盘面，回车确认；quit退出。", command);
        if (command != Command::Ready) continue;
        observe(session);
        if (session.observed.board != initial) continue;
        if (chess::Game_reset(session.game, initial, config.redToMove, config.humanRed)) return;
        Event_emit(EventType::Vision, session.game.error);
    }
}

//---------玩家回合区---------
void Match_human(Session &session, const std::vector<std::string> &legal)
{
    Command command;
    chess::Move move;
    while (gameRunning)
    {
        Match_publishStatus("wait_human", session);
        Operator_read("human", "走棋并移开手后回车；resign认输，draw和棋，quit退出。", command);
        if (command == Command::Draw || command == Command::Resign)
        {
            chess::Result result = command == Command::Draw ? chess::Result::Draw :
                (session.game.humanRed ? chess::Result::BlackWon : chess::Result::RedWon);
            chess::Game_finish(session.game, result);
            return;
        }
        observe(session);
        if (session.observed.board == session.game.board) continue;
        if (chess::inferMove(session.game.board, session.observed, session.game.redToMove, legal, move, session.game.error))
        {
            chess::Game_commit(session.game, session.observed.board, move);
            Event_emit(EventType::Restored, "");
            return;
        }
        Event_emit(EventType::IllegalMove, "请恢复原棋局后重新走棋。", &session.game.board, &session.observed.board);
        waitBoard(session, session.game.board, EventType::IllegalMove);
    }
}

//---------机器人回合区---------
void Match_robot(const GameConfig &config, Session &session, const std::vector<std::string> &legal,
                 const std::string &choice)
{
    chess::Move move;
    chess::Board expected;
    //引擎走法只核对一次，预期盘面在本回合局部保存
    if (!chess::parseMove(choice, move) || std::count(legal.begin(), legal.end(), choice) != 1 ||
        !chess::applyMove(session.game.board, session.game.redToMove, move, expected, session.game.error))
        throw std::runtime_error("引擎走法未通过本局合法表核对");
    while (gameRunning)
    {
        //思考期间有人移动棋子时，先恢复已确认盘面，再执行这一手
        waitBoard(session, session.game.board, EventType::BoardChanged);
        std::vector<PickStep> steps;
        planMove(session, move, steps);
        PickStep park;
        park.action = PickAction::Move;
        park.target = {session.heights.travelDeg, config.park.j2Deg, config.park.j3MotorDeg};
        steps.push_back(park);
        session.pending = true;
        Match_publishStatus("executing", session);
        bool completed = false;
        try { Robot_execute("move", choice, steps, completed); }
        catch (const std::exception &error) { Event_emit(EventType::Stop, error.what()); }
        if (completed)
        {
            observe(session);
            if (session.observed.board == expected)
            {
                chess::Game_commit(session.game, expected, move);
                session.pending = false;
                Event_emit(EventType::Restored, "");
                return;
            }
            Event_emit(EventType::Placement, "落子结果未确认，请按提示处理现场。", &expected, &session.observed.board);
        }
        //执行异常先确认设备恢复；显示回调只提示，主流程才决定何时继续
        Command command;
        do
        {
            Match_publishStatus("hardware_recovery", session);
            Operator_read("hardware_recovery", "处理报错、清空吸头并恢复驱动后回车；程序将回观察位并核对原棋局。", command);
        } while (command != Command::Ready || !Motion_stop() || !prepare(config, session));
        waitBoard(session, session.game.board, EventType::BoardChanged);
        session.pending = false;
        Event_emit(EventType::Restored, "设备和原棋局已恢复，继续下棋。");
    }
}

//---------观察与恢复区---------
static void observe(Session &session)
{
    session.layout.previous = {};
    session.layout.stableFrames = 0;
    Observation current;
    //持续读取新图，取得连续稳定的盘面后才交付上层
    int frames = 0;
    bool visualFault = false;
    while (gameRunning && !current.valid)
    {
        try
        {
            std::vector<Detection> pieces;
            Vision_observe(session.layout, session.lastFrameId, pieces);
            Geometry_observe(session.layout, session.lastFrameId, pieces, true, current);
            if (++frames % 3 == 0 && !current.valid)
            {
                visualFault = true;
                Event_emit(EventType::Vision, "盘面尚未看清，请移开遮挡并保持棋子稳定。");
                Match_publishStatus("observing", session);
            }
        }
        catch (const std::exception &error)
        {
            if (!gameRunning) throw;
            visualFault = true;
            Event_emit(EventType::Vision, error.what());
            Match_publishStatus("vision_recovery", session);
            Command command;
            do
            {
                Operator_read("vision_recovery", "恢复相机画面后回车继续；quit退出。", command);
            } while (command != Command::Ready);
            session.layout.previous = {};
            session.layout.stableFrames = 0;
        }
    }
    if (!gameRunning) throw std::runtime_error("对局已停止");
    if (visualFault) Event_emit(EventType::Restored, "视觉已恢复。");
    session.observed.valid = current.valid;
    for (int row = 0; row < Rows; ++row)
        for (int col = 0; col < Columns; ++col)
        {
            int piece = current.pieces[(Rows - 1 - row) * Columns + col];
            session.observed.board[row][col] = piece >= 0 && piece < YoloClasses ? PieceSymbols[piece] :
                                             (piece == Empty ? '.' : '?');
        }
    Match_publishStatus("observing", session);
}

static void waitBoard(Session &session, const chess::Board &expected, EventType event)
{
    bool waiting = false;
    while (gameRunning)
    {
        observe(session);
        if (session.observed.board == expected)
        {
            if (waiting) Event_emit(EventType::Restored, "原棋局已恢复，继续下棋。");
            return;
        }
        waiting = true;
        Event_emit(event, "请恢复原棋局，恢复后继续。", &expected, &session.observed.board);
        Match_publishStatus("restore_board", session);
    }
    throw std::runtime_error("对局已停止");
}

//---------动作准备区---------
static bool prepare(const GameConfig &config, Session &session)
{
    PickStep hold, lift, park;
    hold.action = PickAction::Hold;
    lift.action = PickAction::Move;
    lift.axes = ZAxis;
    lift.target[0] = session.heights.travelDeg;
    park.action = PickAction::Move;
    park.target = {session.heights.travelDeg, config.park.j2Deg, config.park.j3MotorDeg};
    bool completed = false;
    Robot_execute("prepare", "", {hold, lift, park}, completed);
    return completed;
}

static void planMove(const Session &session, const chess::Move &move, std::vector<PickStep> &steps)
{
    cv::Point2d source, destination;
    if (!Geometry_target(session.layout, move.fromCol, Rows - 1 - move.fromRow, source) ||
        !Geometry_target(session.layout, move.toCol, Rows - 1 - move.toRow, destination))
        throw std::runtime_error("取放格点未准备好");
    if (session.game.board[move.toRow][move.toCol] != '.')
        if (!Pick_plan(session.arm, session.heights, destination, session.capturePlaceMm, false, steps))
            throw std::runtime_error("吃子收纳位不可达");
    std::vector<PickStep> moving;
    if (!Pick_plan(session.arm, session.heights, source, destination, false, moving))
        throw std::runtime_error("取放目标不可达");
    steps.insert(steps.end(), moving.begin(), moving.end());
}

//---------事件说明与输入区---------
bool Command_parse(const std::string &text, Command &command)
{
    static const char *names[] = {"ready", "resign", "draw", "quit"};
    command = Command::Ready;
    if (text.empty()) return true;
    for (int i = 0; i < 4; ++i)
        if (text == names[i])
        {
            command = static_cast<Command>(i);
            return true;
        }
    command = Command::Quit;
    return false;
}
} // namespace scara
