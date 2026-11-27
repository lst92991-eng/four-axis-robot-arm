// 从实盘建立棋局，逐手选棋、搬子、核对落点，再把已完成的走法记入棋谱。
#include "game.hpp"
#include "motion/execute.hpp"
#include <optional>
#include <stdexcept>

namespace scara
{
namespace
{
// 固定棋盘各格的 J2/J3 关节目标，按棋盘的行列查找。
struct BoardTargets
{
    JointTarget cells[Rows][Columns];
    std::optional<JointTarget> discard; // 已示教且可达的盘外弃子位。
    cv::Point discardSide;  // 跟弃子位同一侧的盘外棋子不参与归格。
};

enum class TurnMove { None, ByHand, ByArm };
enum class Landing { Placed, Restored, Stopped };

// 开局时绑定本机配置、格点目标和本局状态，供每一手棋使用。
class MatchSession
{
public:
    MatchSession(const StaticParams &site, const BoardTargets &targets, DynamicParams &match)
        : site(site), targets(targets), match(match) {}

    bool start();
    void playTurn();

private:
    const StaticParams &site;
    const BoardTargets &targets;
    DynamicParams &match;

    TurnMove selectMove(chess::Move &move);
    void planMove(const chess::Move &move, std::vector<PickStep> &steps) const;
    bool readBoard(const chess::Board *expected = nullptr);
    void showBoardDifferences(const chess::Board *expected, std::string message);
    bool waitForOriginal();
    Landing confirmLanding(const chess::Board &expected);
    bool parkForObservation();
    bool restorePark(const char *prompt);
    bool executeMotion(const std::vector<PickStep> &steps, const chess::Move *move = nullptr);
    bool executeAndConfirm(const chess::Move &move, const std::vector<PickStep> &steps);
};
} // namespace

// 主程序、取流和执行器共用这个停止标志。
volatile std::sig_atomic_t gameRunning = 1;

//---------开局准备固定关节目标---------
static BoardTargets prepareBoardTargets(const StaticParams &site)
{
    BoardTargets targets;
    const Geometry &board = site.layout;
    for (int row = 0; row < Rows; ++row)
        for (int col = 0; col < Columns; ++col)
        {
            cv::Point2d xyMm = board.cell[row][col].xy;
            if (!Arm_inverse(site.arm, xyMm, targets.cells[row][col]))
                throw std::runtime_error("安装表的格点不可达 (" + std::to_string(row) + "," + std::to_string(col) + ")");
        }

    // 向外超过半格才把该点用作弃子位。
    cv::Point2d offsetMm = site.capturePlaceMm - board.cell[0][0].xy;
    cv::Point2d cell = Geometry_gridPoint(offsetMm, board.rowStep.xy, board.columnStep.xy);
    cv::Point side = Geometry_boardSide(cell);
    JointTarget discard;
    if (side != cv::Point{} && Arm_inverse(site.arm, site.capturePlaceMm, discard))
    {
        targets.discard = discard;
        targets.discardSide = side;
    }
    return targets;
}

//---------整局收尾---------
int Match_finish(const DynamicParams &match, std::string failure)
{
    if (!failure.empty()) failure += "。请排除故障后重新开局。";
    // 请求停车并回收引擎、相机和模型。返回 false 表示收尾未确认。
    if (!Runtime_close())
    {
        // failure 可能已有运行报错，把收尾问题接在后面一起报告。
        if (!failure.empty()) failure += "。";
        failure += "机械停止或资源回收未确认";
    }
    Operator_show(match, failure);
    // 退出码 0 表示正常收尾，1 表示有错误需要处理。
    return failure.empty() ? 0 : 1;
}

//---------完整对局入口---------
// site 提供本机配置，match 记录这局的盘面、轮次和观察结果。
int Match_run(const StaticParams &site, DynamicParams &match)
{
    // 这局还没确认过实盘。从第一张新帧开始计数。
    match.lastFrameId = 0;
    match.stable = false;
    gameRunning = 1;

    // 运行错误留到统一收尾后报告。
    std::string failure;
    try
    {
        // 固定棋盘的 90 格在开局一次算好 J2/J3 目标角，走棋时按行列查。
        const BoardTargets targets = prepareBoardTargets(site);
        MatchSession session(site, targets, match);

        // 得到开局确认后发布起始盘面。开局失败或取消就跳过走棋循环。
        if (session.start())
            Operator_show(match);
        else
            gameRunning = 0;

        // 每轮处理一手棋。玩家退出或棋局已有结果，就离开循环。
        while (gameRunning && match.game.result == chess::Result::Playing)
            session.playTurn();
    }
    catch (const std::exception &error)
    {
        // 运行中遇到设备或数据错误时记下原因。主动退出后进入设备收尾。
        if (gameRunning) failure = error.what();
    }
    // 正常终局、退出、开局失败和运行报错都从这里收尾。
    return Match_finish(match, failure);
}

namespace
{
//---------开局区---------
bool MatchSession::start()
{
    if (!Operator_prepare()) return false;
    // 空吸头先升到搬运高度，再回观察位，让相机看全棋盘。
    if (!parkForObservation() && !restorePark("请恢复驱动、清空吸头并确认可运动，回车继续。"))
        return false;
    while (gameRunning && readBoard())
    {
        chess::Board initial = match.observed;
        if (!Operator_confirmOpening(initial)) continue;
        if (!gameRunning) return false;
        // 回车后重新看盘，防止确认时棋子已经移动。
        if (!readBoard()) return false;
        if (match.observed != initial) continue;
        // 新局把确认的实盘作为起点。续局还要对上已有棋谱。
        chess::Game &game = match.game;
        if (game.history.empty())
        {
            if (chess::Game_reset(game, initial, site.redToMove, site.humanRed)) return true;
        }
        else if (initial == game.board) return true;
        else if (chess::validBoard(initial)) throw std::runtime_error("实拍盘面与已走历史不符");
        Operator_show(match, "开局盘面有误，请核对棋子和朝向。");
    }
    return false;
}

//---------走一手棋---------
void MatchSession::playTurn()
{
    chess::Move move;
    TurnMove selected = selectMove(move);
    if (selected == TurnMove::None) return;

    // 面板选棋和电脑应手都需要机械臂搬棋。
    if (selected == TurnMove::ByArm)
    {
        if (!readBoard(&match.game.board) || !waitForOriginal()) return;
        std::vector<PickStep> steps;
        planMove(move, steps);
        if (!executeAndConfirm(move, steps)) return;
    }

    // 手走已在选棋时核对，代走已在执行后核对。
    if (!gameRunning) return;
    chess::Game_commit(match.game, move);
    match.pending = false;
    Operator_show(match);
}

//---------选棋区---------
// 引擎交付本轮合法表。电脑直接选一步，玩家在面板选棋或亲手搬棋。
TurnMove MatchSession::selectMove(chess::Move &move)
{
    chess::Game &game = match.game;
    chess::MoveList legal;
    Engine_query(game, legal, move);
    if (!gameRunning) return TurnMove::None;
    for (const chess::Move &candidate : legal)
        if (!chess::validTransfer(game.board, game.redToMove, candidate))
            throw std::runtime_error("引擎走法与当前盘面不符");
    chess::Game_judge(game, legal);
    if (game.result != chess::Result::Playing) return TurnMove::None;

    if (game.redToMove != game.humanRed)
    {
        if (chess::countMove(legal, move) != 1)
            throw std::runtime_error("走法未通过本局合法表核对");
        return TurnMove::ByArm;
    }

    while (gameRunning)
    {
        Operator_show(match, {}, &legal);
        Command command = Operator_read("在上位机选棋，或手动走棋后回车。resign 认输，draw 和棋，quit 退出。", &move);
        if (!gameRunning) return TurnMove::None;
        if (command == Command::Draw)
        {
            game.result = chess::Result::Draw;
            return TurnMove::None;
        }
        if (command == Command::Resign)
        {
            game.result = game.humanRed ? chess::Result::BlackWon : chess::Result::RedWon;
            return TurnMove::None;
        }
        if (command == Command::Move)
        {
            if (chess::countMove(legal, move) == 1) return TurnMove::ByArm;
            Operator_show(match, "这步不合法，请重新选棋。");
            continue;
        }

        // 回车后从新盘面找出唯一合法的走法。原盘没变化时继续等待。
        if (!readBoard()) return TurnMove::None;
        if (chess::inferMove(game.board, match.observed, legal, move)) return TurnMove::ByHand;
        if (match.observed == game.board) continue;
        Operator_show(match, "请恢复原棋局后重新走棋。");
        if (!waitForOriginal()) return TurnMove::None;
    }
    return TurnMove::None;
}

//---------整手取放规划区---------
void MatchSession::planMove(const chess::Move &move, std::vector<PickStep> &steps) const
{
    const JointTarget &source = targets.cells[move.fromRow][move.fromCol];
    const JointTarget &target = targets.cells[move.toRow][move.toCol];
    const JointTarget *discard = nullptr;
    if (match.game.board[move.toRow][move.toCol] != '.')
    {
        if (!targets.discard) throw std::runtime_error("吃子需要示教棋盘外可达的弃子点");
        discard = &*targets.discard;
    }
    Plan_turn(site.heights, source, target, discard, site.park, steps);
}

//---------执行动作并核实落点---------
bool MatchSession::executeAndConfirm(const chess::Move &move, const std::vector<PickStep> &steps)
{
    chess::Board expected = match.game.board;
    chess::movePiece(expected, move);
    while (gameRunning)
    {
        bool moved = executeMotion(steps, &move);

        if (!gameRunning) return false;
        if (moved)
        {
            Landing landing = confirmLanding(expected);
            if (landing == Landing::Placed) return true;
            if (landing == Landing::Stopped) return false;
        }
        else
        {
            if (!restorePark("处理报错、清空吸头并恢复驱动后回车。程序将回观察位并核对原棋局。"))
                return false;
            if (!readBoard(&match.game.board) || !waitForOriginal()) return false;
        }
        if (!gameRunning) return false;
        match.pending = false;
        Operator_show(match, "原棋局已恢复，重新执行这一手。");
    }
    return false;
}

bool waitReady(const char *prompt)
{
    while (gameRunning)
    {
        if (Operator_read(prompt) == Command::Ready) return gameRunning;
    }
    return false;
}

// 把需要核对的格号写进提示，保留相机实际交付的盘面。
void MatchSession::showBoardDifferences(const chess::Board *expected, std::string message)
{
    if (expected)
        for (int row = 0; row < Rows; ++row)
            for (int col = 0; col < Columns; ++col)
                if ((*expected)[row][col] != match.observed[row][col])
                    message += " (" + std::to_string(row) + "," + std::to_string(col) + ")";
    Operator_show(match, message);
}

//---------观察与恢复区---------
bool MatchSession::readBoard(const chess::Board *expected)
{
    int same = 0;
    bool warned = false; // 同一处遮挡只提示一次，格子变化后再更新。
    match.stable = false;
    Operator_show(match);
    while (gameRunning && same < 3)
    {
        uint64_t frameId = 0;
        std::vector<Detection> pieces;
        Vision_observe(frameId, pieces);
        if (!gameRunning) return false;
        // 回退或重复的帧不计数。缺帧进入无效盘面处理，清除连续计数。
        if (frameId && frameId <= match.lastFrameId) continue;
        if (frameId) match.lastFrameId = frameId;
        BoardObservation frame = Geometry_assign(site.layout, pieces, frameId != 0, targets.discardSide);
        bool boardChanged = frame.cells != match.observed;
        match.observed = frame.cells;
        if (boardChanged) warned = false;
        if (frame.valid) same = boardChanged ? 1 : same + 1;
        else
        {
            same = 0;
            if (!warned) showBoardDifferences(expected, "棋子偏离格点或受遮挡，请检查实盘。");
            warned = true;
        }
    }
    if (!gameRunning) return false;
    match.stable = true;
    Operator_show(match);
    return true;
}

// 使用调用者刚读到的实盘。如盘面不同，才等待并重新读取。
bool MatchSession::waitForOriginal()
{
    const chess::Board &expected = match.game.board;
    if (match.observed == expected) return true;
    do
    {
        showBoardDifferences(&expected, "请恢复原棋局");
        if (!readBoard(&expected)) return false;
    } while (match.observed != expected);
    Operator_show(match, "原棋局已恢复。");
    return true;
}

Landing MatchSession::confirmLanding(const chess::Board &expected)
{
    if (!readBoard(&expected)) return Landing::Stopped;
    while (match.observed != expected)
    {
        showBoardDifferences(&expected, "请核对落点");
        if (!waitReady("修正落点后回车。若摆回走棋前的局面，回车后重走。"))
            return Landing::Stopped;
        if (!readBoard(&expected)) return Landing::Stopped;
        if (match.observed == match.game.board) return Landing::Restored;
    }
    return Landing::Placed;
}

//---------动作准备区---------
bool MatchSession::executeMotion(const std::vector<PickStep> &steps, const chess::Move *move)
{
    // 机械动作开始后，动作前的视觉结果不再有效。
    match.stable = false;
    if (move)
    {
        match.pending = true;
        Operator_show(match);
    }
    if (!gameRunning) return false;
    try
    {
        if (move) return Robot_execute("move", *move, steps);
        return Robot_execute("prepare", {}, steps);
    }
    catch (const std::exception &error)
    {
        if (gameRunning) Operator_show(match, error.what());
        return false;
    }
}

bool MatchSession::parkForObservation()
{
    // 准备动作按顺序关断磁铁、单独抬 J1、再让 J2 和 J3 到观察位。
    double travel = site.heights.travelDeg;
    std::vector<PickStep> steps = {{PickAction::Release}, {PickAction::LiftZ, {travel}}};
    Plan_move(site.park, travel, steps);
    return executeMotion(steps);
}

//---------驱动故障后恢复原盘区---------
bool MatchSession::restorePark(const char *prompt)
{
    // 驱动中断后先处理吸头和故障。停车得到确认，才再次回观察位。
    do
    {
        if (!waitReady(prompt)) return false;
    } while (!Motion_stop() || !parkForObservation());
    return true;
}

} // namespace

} // namespace scara
