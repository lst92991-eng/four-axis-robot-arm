#include "game.hpp"
#include <stdexcept>

namespace scara
{
static bool finishCommand(Session &session, Command command)
{
    if (command == Command::Quit)
    {
        gameRunning = 0;
        throw std::runtime_error("对局已停止");
    }
    if (command == Command::Draw) return chess::Game_finish(session.game, chess::Result::Draw);
    if (command == Command::Resign)
        return chess::Game_finish(session.game,
                                  session.game.humanRed ? chess::Result::BlackWon : chess::Result::RedWon);
    return false;
}

static void observe(Session &session)
{
    session.stable.reset();
    IO_report("observing", session);
    for (int frame = 0; frame < 3; ++frame)
    {
        if (!gameRunning) throw std::runtime_error("对局已停止");
        std::string frameId;
        std::vector<Detection> pieces;
        IO_observe(session.layout, frameId, pieces);
        Session_observe(session, frameId, pieces, true); // 已确认手移开，执行器已停在观察位。
    }
    IO_report("observed", session);
}

void Match_start(const GameConfig &config, Session &session)
{
    IO_report("prepare", session);
    while (gameRunning)
    {
        Command command =
            IO_command("prepare", "先将空吸头抬离棋盘并移开手，回车让执行器进入示教搬运高度和观察位；quit退出。");
        if (command == Command::Quit) finishCommand(session, command);
        if (command == Command::Ready) break;
    }
    std::vector<PickStep> prepare{{PickAction::Hold},
                                  {PickAction::MoveZ, {}, session.heights.travelDeg, session.heights.speedDegS},
                                  {PickAction::MoveXY, config.park}};
    bool prepared = false;
    IO_execute("prepare", "", prepare, prepared);
    if (!prepared) throw std::runtime_error("执行器未准备好；没有开始对局");
    while (gameRunning)
    {
        observe(session);
        if (!session.observed.valid)
        {
            IO_report("start_unreadable", session);
            Command command = IO_command("start_retry", "识别未稳定或棋子不完整；调整后回车重读，quit退出。");
            if (command == Command::Quit) finishCommand(session, command);
            continue;
        }
        auto initial = session.observed.board;
        IO_report("confirm_start", session);
        Command command = IO_command("confirm_start", "核对预览和识别盘面；回车确认开局，retry重读，quit退出。");
        if (command == Command::Quit) finishCommand(session, command);
        if (command != Command::Ready) continue;
        observe(session);
        if (session.observed.valid && session.observed.board == initial)
        {
            if (!chess::Game_reset(session.game, initial, config.redToMove, config.humanRed))
                throw std::runtime_error(session.game.error);
            IO_report("started", session);
            return;
        }
    }
    throw std::runtime_error("对局已停止");
}

void Match_human(Session &session, const std::vector<std::string> &legal)
{
    while (gameRunning)
    {
        IO_report("wait_human", session);
        Command command = IO_command("human", "请走棋并移开手，回车确认；resign认输，draw和棋，quit退出。");
        if (finishCommand(session, command)) return;
        if (command != Command::Ready && command != Command::Retry) continue;
        observe(session);
        chess::Move move;
        if (Session_acceptHuman(session, legal, move)) return;
        IO_report("human_rejected", session);
    }
    throw std::runtime_error("对局已停止");
}

void Match_robot(const GameConfig &config, Session &session, const std::vector<std::string> &legal,
                 const std::string &choice)
{
    observe(session); // 搜索结束后重新取图，不能直接用搜索前的观察执行。
    while (!session.observed.valid || session.observed.board != session.game.board)
    {
        IO_report("board_changed", session);
        Command command = IO_command("restore_before_move", "盘面尚未稳定或已变化；恢复后回车，quit退出。");
        if (finishCommand(session, command)) return;
        observe(session);
    }
    if (!Session_plan(session, choice, legal)) throw std::runtime_error(session.game.error);
    session.steps.push_back({PickAction::MoveXY, config.park}); // 放子抬升后让开棋盘，才能视觉复核。
    IO_report("executing", session);
    bool completed = false;
    IO_execute("move", choice, session.steps, completed);
    if (completed)
    {
        observe(session);
        if (Session_confirm(session, true)) return;
    }
    else
        Session_confirm(session, false);
    while (gameRunning)
    {
        IO_report("recovery", session);
        Command command = IO_command(
            "recovery",
            "动作未确认：回车/retry只重读；accept确认实际已完成；restored确认全部复位、吸头空并回观察位后重新规划；quit退出。");
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
