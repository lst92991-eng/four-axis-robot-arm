#include "game.hpp"
#include <stdexcept>

namespace scara
{
bool Command_parse(const std::string &text, Command &command)
{
    command = Command::Quit;
    if (text.empty() || text == "ready")
        command = Command::Ready;
    else if (text == "retry")
        command = Command::Retry;
    else if (text == "accept")
        command = Command::Accept;
    else if (text == "restored")
        command = Command::Restored;
    else if (text == "resign")
        command = Command::Resign;
    else if (text == "draw")
        command = Command::Draw;
    else if (text != "quit")
        return false;
    return true;
}

static void readCommand(const std::string &phase, const std::string &prompt, Command &command)
{
    Operator_read(phase, prompt, command);
    if (command == Command::Quit)
    {
        gameRunning = 0;
        throw std::runtime_error("对局已停止");
    }
}

static bool finishCommand(Session &session, Command command)
{
    chess::Result winner;
    if (command == Command::Draw) return chess::Game_finish(session.game, chess::Result::Draw);
    if (command == Command::Resign)
    {
        winner = session.game.humanRed ? chess::Result::BlackWon : chess::Result::RedWon;
        return chess::Game_finish(session.game, winner);
    }
    return false;
}

static void observe(Session &session)
{
    session.stable.reset();
    Match_report("observing", session);
    for (int frame = 0; frame < 3; ++frame)
    {
        if (!gameRunning) throw std::runtime_error("对局已停止");
        std::string frameId;
        std::vector<Detection> pieces;
        Vision_observe(session.layout, frameId, pieces);
        Session_observe(session, frameId, pieces, true); // 已确认手移开，执行器已停在观察位。
    }
    Match_report("observed", session);
}

void Match_start(const GameConfig &config, Session &session)
{
    Command command = Command::Retry; // 操作员在当前阶段输入的命令。
    bool prepared = false;            // 执行器是否已到示教观察位。
    chess::Board initial;             // 等待操作员核对的开局盘面。
    PickStep hold, lift, park;        // 保持吸持、抬到搬运层、让开棋盘。
    std::vector<PickStep> prepare;    // 按执行顺序保存上述三个动作。

    hold.action = PickAction::Hold;
    lift.action = PickAction::MoveZ;
    lift.zDeg = session.heights.travelDeg;
    lift.speedDegS = session.heights.speedDegS;
    park.action = PickAction::MoveXY;
    park.joints = config.park;
    prepare.push_back(hold);
    prepare.push_back(lift);
    prepare.push_back(park);

    Match_report("prepare", session);
    while (command != Command::Ready)
        readCommand("prepare", "先将空吸头抬离棋盘并移开手，回车让执行器进入示教搬运高度和观察位；quit退出。", command);
    Robot_execute("prepare", "", prepare, prepared);
    if (!prepared) throw std::runtime_error("执行器未准备好；没有开始对局");
    while (gameRunning)
    {
        observe(session);
        if (!session.observed.valid)
        {
            Match_report("start_unreadable", session);
            readCommand("start_retry", "识别未稳定或棋子不完整；调整后回车重读，quit退出。", command);
            continue;
        }
        initial = session.observed.board;
        Match_report("confirm_start", session);
        readCommand("confirm_start", "核对预览和识别盘面；回车确认开局，retry重读，quit退出。", command);
        if (command != Command::Ready) continue;
        observe(session);
        if (session.observed.valid && session.observed.board == initial)
        {
            if (!chess::Game_reset(session.game, initial, config.redToMove, config.humanRed))
                throw std::runtime_error(session.game.error);
            Match_report("started", session);
            return;
        }
    }
    throw std::runtime_error("对局已停止");
}

void Match_human(Session &session, const std::vector<std::string> &legal)
{
    Command command;
    chess::Move move;
    while (gameRunning)
    {
        Match_report("wait_human", session);
        readCommand("human", "请走棋并移开手，回车确认；resign认输，draw和棋，quit退出。", command);
        if (finishCommand(session, command)) return;
        if (command != Command::Ready && command != Command::Retry) continue;
        observe(session);
        if (chess::Game_acceptHuman(session.game, session.observed, legal, move)) return;
        Match_report("human_rejected", session);
    }
    throw std::runtime_error("对局已停止");
}

void Match_robot(const GameConfig &config, Session &session, const std::vector<std::string> &legal,
                 const std::string &choice)
{
    Command command;
    bool completed = false; // 执行器是否已完成全部动作并停稳。
    PickStep park;          // 放子后让开棋盘，才能重新观察全部棋子。

    park.action = PickAction::MoveXY;
    park.joints = config.park;
    observe(session); // 搜索结束后重新取图，不能直接用搜索前的观察执行。
    while (!session.observed.valid || session.observed.board != session.game.board)
    {
        Match_report("board_changed", session);
        readCommand("restore_before_move", "盘面尚未稳定或已变化；恢复后回车，quit退出。", command);
        if (finishCommand(session, command)) return;
        observe(session);
    }
    if (!Session_plan(session, choice, legal)) throw std::runtime_error(session.game.error);
    session.steps.push_back(park);
    Match_report("executing", session);
    Robot_execute("move", choice, session.steps, completed);
    if (completed) observe(session);
    if (Session_confirm(session, completed)) return;
    while (gameRunning)
    {
        Match_report("recovery", session);
        readCommand(
            "recovery",
            "动作未确认：回车/retry只重读；accept确认实际已完成；restored确认全部复位、吸头空并回观察位后重新规划；quit退出。",
            command);
        if (finishCommand(session, command)) return;
        if (command == Command::Resign || command == Command::Draw) continue;
        if (command == Command::Accept) completed = true;
        observe(session);
        if (command == Command::Restored)
        {
            if (Session_restore(session)) return;
            continue;
        }
        if (Session_confirm(session, completed)) return;
    }
    throw std::runtime_error("对局已停止");
}
} // namespace scara
