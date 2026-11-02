#include "app/config.hpp"
#include "app/journal.hpp"
#include "chess/pikafish.hpp"
#include "vision/camera.hpp"
#include <atomic>
#include <algorithm>
#include <chrono>
#include <csignal>
#include <cmath>
#include <iostream>
#include <memory>
#include <mutex>
#include <poll.h>
#include <sstream>
#include <thread>
#include <unistd.h>

namespace {
volatile std::sig_atomic_t interrupted = 0;
void interrupt(int) { interrupted = 1; }

// 同一输入入口：运动/思考期间只接受stop/quit；部分输入也不会阻塞反馈。
class Input {
public:
    bool cancelled() {
        const auto line = read(false);
        if (line == "stop" || line == "quit" || line == "exit") interrupted = 1;
        return interrupted != 0;
    }
    std::string read(bool wait) {
        while (1) {
            const auto end = pending_.find('\n');
            if (end != std::string::npos) {
                std::string line = pending_.substr(0,end); pending_.erase(0,end+1);
                if (!line.empty() && line.back() == '\r') line.pop_back();
                return line;
            }
            pollfd p{STDIN_FILENO, POLLIN, 0};
            const int ready = ::poll(&p,1,wait ? 100 : 0);
            if (ready > 0) {
                char buffer[128]; const auto size = ::read(STDIN_FILENO,buffer,sizeof(buffer));
                if (size <= 0) { interrupted = 1; return "quit"; }
                pending_.append(buffer,static_cast<std::size_t>(size));
                if (pending_.size() > 256) { pending_.clear(); interrupted = 1; return "stop"; }
            }
            if (interrupted || (!wait && ready <= 0)) return {};
        }
    }
private:
    std::string pending_;
};

chess::Observation scan(const app::Config& c, Input& input) {
#ifndef SCARA_WITH_RKNN
    (void)c; (void)input;
    throw std::runtime_error("本构建没有RKNN；不能以假识别或CPU回退运行实机对局");
#else
    Camera camera(c.cameraWidth,c.cameraHeight);
    camera.cancelled = [&] { return input.cancelled(); };
    BoardVision vision(c.vision);
    std::string error;
    std::unique_ptr<Detector,decltype(&detector_close)> detector(detector_load(c.model.c_str(),error),detector_close);
    if (!detector) throw std::runtime_error(error);
    if (!camera.start(c.cameraScript,error)) throw std::runtime_error(error);
    chess::Observation result;
    for (int attempt = 0; attempt < 30 && !input.cancelled(); ++attempt) {
        cv::Mat bgr,rgb;
        if (!camera.read(bgr,error)) break;
        if (!vision.locate(bgr,rgb,error)) continue;
        std::vector<Detection> detections;
        if (!detector_detect(detector.get(),rgb,detections,error)) break;
        result = vision.interpret(detections);
        if (result.complete && result.stable && !result.duplicate) break;
    }
    if (!camera.stop()) throw std::runtime_error("相机尚未回收，禁止启动引擎");
    if (input.cancelled()) throw std::runtime_error("用户停止");
    if (!result.complete || !result.stable || result.duplicate)
        throw std::runtime_error(error.empty() ? "尚未识别到稳定完整盘面" : error);
    return result;
#endif
}

// 运动观测只保留最新帧；图像采集/标记识别不占用CAN反馈循环。
class Observer {
public:
    explicit Observer(const app::Config& config) : c_(config), camera_(config.cameraWidth,config.cameraHeight) {
        if (!c_.latencyCalibrated || !c_.vision.tcpCalibrated) return;
        camera_.cancelled = [&] { return quit_.load(); };
        std::string error;
        if (!camera_.start(c_.cameraScript,error)) throw std::runtime_error(error);
        worker_ = std::thread([this] {
            try {
                BoardVision vision(c_.vision);
                while (!quit_) {
                    const double z = z_.load();
                    const double lowerTime = motion::seconds() - c_.captureLatencyBoundSeconds;
                    cv::Mat frame; std::string error;
                    if (!camera_.read(frame,error)) {
                        if (!quit_) { std::lock_guard<std::mutex> lock(mutex_); failure_ = error; }
                        break;
                    }
                    auto seen = vision.observeTcp(frame,lowerTime,z);
                    if (std::abs(z_.load()-z) > c_.vision.tcpPlaneToleranceMm) seen.valid = false;
                    std::lock_guard<std::mutex> lock(mutex_);
                    latest_ = {seen.xy,seen.stamp,seen.valid}; sampledZ_ = z;
                }
            } catch (const std::exception& e) {
                std::lock_guard<std::mutex> lock(mutex_); failure_ = e.what();
            }
        });
    }
    ~Observer() { close(); }
    motion::TcpObservation latest(double z) {
        z_ = z;
        std::lock_guard<std::mutex> lock(mutex_);
        if (!failure_.empty()) throw std::runtime_error(failure_);
        auto result = latest_;
        if (std::abs(z-sampledZ_) > c_.vision.tcpPlaneToleranceMm) result.valid = false;
        return result;
    }
    bool close() {
        quit_ = true;
        if (worker_.joinable()) worker_.join();
        return camera_.stop();
    }
private:
    const app::Config& c_;
    Camera camera_;
    std::atomic<bool> quit_{false};
    std::atomic<double> z_{-1.e9};
    std::thread worker_;
    std::mutex mutex_;
    motion::TcpObservation latest_{};
    double sampledZ_ = -1.e9;
    std::string failure_;
};

void restore(const app::Config& c, const app::Journal& j, chess::Game& game, Input& input) {
    if (j.baseFen.empty()) { game.reset(chess::initialBoard(),true,!c.robotRed); return; }
    Pikafish engine; engine.cancelled = [&] { return input.cancelled(); };
    if (!engine.start(c.engineDirectory.c_str())) throw std::runtime_error(engine.error);
    std::istringstream history(j.history); std::string move,prefix;
    while (history >> move) {
        std::vector<std::string> legal;
        if (!engine.legalMoves(j.baseFen,prefix,legal) ||
            std::find(legal.begin(),legal.end(),move) == legal.end())
            throw std::runtime_error("检查点包含无法核验的历史走法");
        if (!prefix.empty()) prefix += ' ';
        prefix += move;
    }
    if (!engine.close()) throw std::runtime_error("恢复核验引擎未退出");
    if (!game.restore(j.baseFen,j.history,!c.robotRed)) throw std::runtime_error(game.error);
}

motion::Pose square(const BoardVision& vision, const app::Config& c, int row, int col) {
    const auto xy = vision.squareXY(row,col);
    return {xy.x,xy.y,c.pieceContactZ,c.pieceYaw};
}

void run(const app::Config& c) {
    Input input;
    app::Journal journal = app::loadJournal(c.journal);
    if (journal.result != int(chess::Result::Playing)) {
        std::cout << "检查点已终局或待裁定，结果=" << journal.result << "。请保留记录，复核后用新对局文件开始。\n";
        return;
    }
    chess::Game game;
    restore(c,journal,game,input);
    std::cout << "机器人执" << (c.robotRed ? "红" : "黑") << "，红方先走。\n"
              << "确认各轴定位有效、工具空载、磁铁已关、棋盘及收纳区与检查点一致，输入ready。\n";
    if (journal.phase != "ready")
        std::cout << "上次中断于" << journal.phase << "，步骤" << journal.nextStep
                  << "。先人工还原到已提交盘面和收纳区，禁止重放中断动作。\n";
    std::cout << "已提交FEN: " << chess::fen(game.board(),game.redToMove()) << '\n';
    if (input.read(true) != "ready" || interrupted) return;
    auto seen = scan(c,input);
    if (seen.board != game.board()) throw std::runtime_error("实物盘面与已提交记录不同，未打开运动设备");
    robot::Can can(c.canInterface.c_str());
    std::array<std::unique_ptr<robot::Motor>,4> motors;
    std::array<robot::Motor*,4> references{};
    for (std::size_t i = 0; i < 4; ++i) {
        motors[i] = std::make_unique<robot::Motor>(can,c.motion.axes[i].node,c.motion.axes[i].pulsesPerRev);
        references[i] = motors[i].get();
    }
    motion::Motion arm(can,references,c.motion);
    arm.prepare([&] { return input.cancelled(); });
    robot::Magnet magnet;
    magnet.open(c.gpioChip,c.gpioLine,c.gpioActiveHigh,false);
    task::PickPlace pick(arm,magnet,c.task);
    BoardVision geometry(c.vision);
    app::committed(journal,game); app::saveJournal(c.journal,journal);
    pick.setProgressObserver([&](const task::Progress& p) {
        journal.nextStep = int(p.nextStep); journal.carrying = p.carrying;
        app::saveJournal(c.journal,journal);
    });
    try {
        while (game.result() == chess::Result::Playing && !input.cancelled()) {
            const bool robotTurn = game.redToMove() == c.robotRed;
            // 先裁定当前方是否还有合法走法；将死/困毙时不再等待人移动棋子。
            Pikafish engine; engine.cancelled = [&] { return input.cancelled(); };
            if (!engine.start(c.engineDirectory.c_str())) throw std::runtime_error(engine.error);
            std::vector<std::string> legal;
            if (!engine.legalMoves(game.baseFen(),game.history(),legal)) throw std::runtime_error(engine.error);
            if (!game.judge(legal)) throw std::runtime_error(game.error);
            std::string choice;
            if (game.result() == chess::Result::Playing && robotTurn &&
                !engine.think(game.baseFen(),game.history(),c.thinkMs,choice))
                throw std::runtime_error(engine.error);
            if (!engine.close()) throw std::runtime_error("引擎尚未回收，禁止进入相机/运动阶段");
            if (game.result() != chess::Result::Playing) break;
            if (!robotTurn) {
                std::cout << "请走棋后输入scan；resign认输，draw确认双方和棋，stop/quit停止。\n";
                const std::string command = input.read(true);
                if (interrupted || command == "quit" || command == "stop") break;
                if (command == "resign") { game.resign(!c.robotRed); break; }
                if (command == "draw") { game.agreeDraw(); break; }
                if (command != "scan") continue;
                seen = scan(c,input);
            }
            chess::Move move;
            if (!robotTurn) {
                if (!game.acceptHuman(seen,legal,move)) { std::cout << game.error << "；请还原或重新走棋。\n"; continue; }
                app::committed(journal,game); app::saveJournal(c.journal,journal);
                continue;
            }
            if (!game.prepareRobot(seen,choice,legal,move)) throw std::runtime_error(game.error);
            task::Request request;
            request.source = square(geometry,c,move.fromRow,move.fromCol);
            request.destination = square(geometry,c,move.toRow,move.toCol);
            const char target = game.board()[move.toRow][move.toCol];
            request.capture = target != '.';
            const int capturedSide = target >= 'A' && target <= 'Z' ? 0 : 1;
            if (request.capture) {
                const auto slot = static_cast<std::size_t>(journal.captured[capturedSide]);
                if (slot >= c.captureSlots[capturedSide].size()) throw std::runtime_error("吃子收纳区已满");
                request.captureSlot = c.captureSlots[capturedSide][slot];
            }
            journal.phase = "moving"; journal.pendingMove = choice;
            journal.nextStep = 0; app::saveJournal(c.journal,journal);
            Observer observer(c);
            arm.setObserver([&](double z) { return observer.latest(z); });
            const auto xyBefore = arm.xyUpdates();
            try { pick.execute(request,[&] { return input.cancelled(); }); }
            catch (...) { arm.setObserver({}); observer.close(); throw; }
            arm.setObserver({});
            if (!observer.close()) throw std::runtime_error("末端相机未退出，停止后续识别");
            if (c.requireTcp && arm.xyUpdates() == xyBefore)
                throw std::runtime_error("本次运动未融合有效独立末端观测，待人工复核");
            journal.phase = "verify"; app::saveJournal(c.journal,journal);
            seen = scan(c,input);
            if (!game.confirmRobot(seen,true)) throw std::runtime_error(game.error);
            if (request.capture) ++journal.captured[capturedSide];
            app::committed(journal,game); app::saveJournal(c.journal,journal);
            std::cout << "已执行并复拍确认：" << choice << "；本步XY融合次数=" << arm.xyUpdates()-xyBefore << '\n';
        }
        if (!arm.stop()) throw std::runtime_error("结束时未确认四轴停止");
        app::committed(journal,game); app::saveJournal(c.journal,journal);
        std::cout << "对局状态=" << int(game.result()) << "，历史=" << game.history() << '\n';
    } catch (const std::exception& e) {
        const bool stopped = arm.stop();
        journal.phase = "fault"; journal.carrying = pick.progress().carrying || magnet.on();
        try { app::saveJournal(c.journal,journal); } catch (const std::exception& log) { std::cerr << log.what() << '\n'; }
        std::cerr << e.what() << "；停止确认=" << stopped << '\n';
        if (journal.carrying) {
            // 此策略只有fault_release_safe经现场确认才可启用；不重置已通电计时。
            const double deadline = pick.energizedAt() + c.task.maximumHoldSeconds;
            std::cerr << "可能带子，请托住棋子，输入release断磁；达到已配置最大通电时间也会断磁。\n";
            interrupted = 0;
            while (motion::seconds() < deadline) {
                if (input.read(false) == "release") break;
                std::this_thread::sleep_for(std::chrono::milliseconds(20));
            }
            magnet.set(false);
        }
        throw;
    }
}
}

int main(int argc, char** argv) {
    std::signal(SIGINT,interrupt); std::signal(SIGTERM,interrupt);
    if (argc != 3 || (std::string(argv[1]) != "--check" && std::string(argv[1]) != "--run")) {
        std::cout << "用法：scara_robot --check|--run config.yml\n--check只校验文件，不打开设备。\n";
        return argc == 1 ? 0 : 2;
    }
    try {
        const auto config = app::loadConfig(argv[2]);
        app::validateConfig(config);
        if (std::string(argv[1]) == "--check") { std::cout << "配置静态检查通过；不代表实机已验收。\n"; return 0; }
#ifndef SCARA_WITH_RKNN
        throw std::runtime_error("--run需要启用RKNN的板端构建");
#endif
        run(config);
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "停止：" << e.what() << '\n';
        return 1;
    }
}
