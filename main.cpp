#include "app/config.hpp"
#include "app/commands.hpp"
#include "chess/board.hpp"
#include "chess/pikafish.hpp"
#include "robot/simulator.hpp"
#include "vision/board_vision.hpp"
#include "vision/camera.hpp"
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <cerrno>
#include <csignal>
#include <cmath>
#include <exception>
#include <fcntl.h>
#include <iostream>
#include <poll.h>
#include <sstream>
#include <sys/stat.h>
#include <unistd.h>

static volatile sig_atomic_t stopped=0;
static void onStop(int) { stopped=1; }

struct Runtime {
    std::string engine, model, cameraScript, output;
    int thinkMs=1000;
    double speed=8;
};

static chess::Observation virtualObservation(const chess::Board& board)
{
    return {board,true,true,false};
}

#ifdef SCARA_WITH_RKNN
// 每次识别结束只发送一组预览图，不把图片混进20Hz的机械臂状态。
static std::string previewJpeg(const cv::Mat& image)
{
    if (image.empty()) return {};
    cv::Mat small;
    cv::resize(image,small,{},0.5,0.5,cv::INTER_AREA);
    std::vector<uchar> bytes;
    if (!cv::imencode(".jpg",small,bytes,{cv::IMWRITE_JPEG_QUALITY,75})) return {};
    const char* alphabet="ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    std::string result;
    for (size_t i=0;i<bytes.size();i+=3) {
        unsigned value=unsigned(bytes[i])<<16;
        if (i+1<bytes.size()) value|=unsigned(bytes[i+1])<<8;
        if (i+2<bytes.size()) value|=bytes[i+2];
        result+=alphabet[(value>>18)&63]; result+=alphabet[(value>>12)&63];
        result+=i+1<bytes.size()?alphabet[(value>>6)&63]:'=';
        result+=i+2<bytes.size()?alphabet[value&63]:'=';
    }
    return result;
}
#endif

// 标准输出是给预览界面的数据接口，不在这里逐步打印调试日志。
static void show(const chess::Game& game, const robot::Simulator& arm,
                 const std::string& phase, const std::string& message,
                 const std::vector<std::string>& legal, const std::string& move,
                 const std::string& source)
{
    const char* results[]={"playing","red_won","black_won","draw","review"};
    cv::FileStorage out("",cv::FileStorage::WRITE|cv::FileStorage::MEMORY|cv::FileStorage::FORMAT_JSON);
    out << "opencv" << cv::getVersionString() << "mode" << "simulation"
        << "phase" << phase << "message" << message << "source" << source
        << "turn" << (game.redToMove()?"red":"black")
        << "result" << results[int(game.result())] << "history" << game.history() << "move" << move;
#ifdef SCARA_WITH_RKNN
    out << "camera_available" << 1;
#else
    out << "camera_available" << 0;
#endif
    out << "board" << "[";
    for (const auto& row:game.board()) out << std::string(row.begin(),row.end());
    out << "]" << "world" << "[";
    for (const auto& row:arm.board()) out << std::string(row.begin(),row.end());
    out << "]" << "legal" << "[";
    for (const auto& value:legal) out << value;
    const auto& q=arm.joints();
    const auto p=arm.tcp();
    out << "]" << "joints" << "[" << q.shoulder << q.elbow << q.z << q.wrist << "]"
        << "tcp" << "[" << p.x << p.y << p.z << p.yaw << "]"
        << "carried" << std::string(1,arm.carried()) << "vacuum" << int(arm.vacuumOn());
    const auto origin=project::squarePose(0,0), rowStep=project::squarePose(1,0), colStep=project::squarePose(0,1);
    out << "geometry" << "{" << "links" << "[" << scara::L1 << scara::L2 << "]"
        << "origin" << "[" << origin.x << origin.y << "]"
        << "row_step" << "[" << rowStep.x-origin.x << rowStep.y-origin.y << "]"
        << "col_step" << "[" << colStep.x-origin.x << colStep.y-origin.y << "]" << "}";
    for (bool red:{true,false}) {
        const auto& slots=arm.captureSlots(red);
        out << (red?"captured_red":"captured_black") << std::string(slots.begin(),slots.end());
    }
    std::string line=out.releaseAndGetString();
    line.erase(std::remove(line.begin(),line.end(),'\n'),line.end());
    line.erase(std::remove(line.begin(),line.end(),'\r'),line.end());
    std::cout << line << std::endl;
}

static bool scanBoard(const Runtime& runtime, chess::Observation& observation,
                      const std::function<bool()>& cancelled, bool& fatal, std::string& error)
{
#ifndef SCARA_WITH_RKNN
    (void)runtime; (void)observation; (void)cancelled; (void)fatal;
    error="这个电脑构建用于仿真；摄像头识别请运行带RKNN的RV1106构建";
    return false;
#else
    Camera camera;
    camera.cancelled=cancelled;
    BoardVision vision;
    cv::Mat bgr,rgb;
    Detector* detector=detector_load(runtime.model.c_str(),error);
    if (!detector) return false;
    bool success=false;
    try {
        if (camera.start(runtime.cameraScript,error)) {
            for (int attempt=0;attempt<20 && !cancelled();++attempt) {
                if (!camera.read(bgr,error)) break;
                if (!vision.locate(bgr,rgb,error)) continue;
                std::vector<Detection> pieces;
                if (!detector_detect(detector,rgb,pieces,error)) break;
                observation=vision.interpret(pieces);
                if (observation.stable) { success=true; break; }
            }
        }
    } catch (const std::exception&) {
        error="图像处理失败，本次盘面不提交";
        success=false;
    }
    // 正常与异常路径都先释放视觉资源，再允许下一阶段加载棋力模型。
    if (!camera.stop()) { fatal=true; success=false; error="采集进程未能回收，程序停止"; }
    detector_close(detector);
    try {
        if (!bgr.empty() && !vision.view().empty()) cv::imwrite(runtime.output+"/camera.jpg",vision.view());
        if (!vision.boardImage().empty()) cv::imwrite(runtime.output+"/board.jpg",vision.boardImage());
        std::cout << "{\"camera_jpeg\":\"" << previewJpeg(vision.view())
                  << "\",\"board_jpeg\":\"" << previewJpeg(vision.boardImage()) << "\"}" << std::endl;
    } catch (const cv::Exception&) { error="诊断图片保存失败"; success=false; }
    if (!success && error.empty()) error="没有得到完整、连续稳定的象棋盘面";
    return success;
#endif
}

int main(int argc,char* argv[])
{
    if (argc!=2) { std::cerr << "用法：scara_robot config/robot.json\n"; return 1; }
    Runtime runtime;
    cv::FileStorage settings(argv[1],cv::FileStorage::READ);
    if (!settings.isOpened()) { std::cerr << "无法读取配置\n"; return 1; }
    settings["engine_dir"] >> runtime.engine;
    settings["model_path"] >> runtime.model;
    settings["camera_module_script"] >> runtime.cameraScript;
    settings["output_dir"] >> runtime.output;
    settings["think_ms"] >> runtime.thinkMs;
    settings["simulation_speed"] >> runtime.speed;
    if (runtime.engine.empty() || runtime.output.empty() || runtime.thinkMs<50 ||
        runtime.thinkMs>10000 || !std::isfinite(runtime.speed) || runtime.speed<=0 || runtime.speed>100) {
        std::cerr << "配置缺少路径或时间/仿真速度超出范围\n"; return 1;
    }
    if (mkdir(runtime.output.c_str(),0700)<0 && errno!=EEXIST) return 1;
    cv::setNumThreads(1);
    struct sigaction action{};
    action.sa_handler=onStop;
    sigemptyset(&action.sa_mask);
    sigaction(SIGINT,&action,nullptr);
    sigaction(SIGTERM,&action,nullptr);
    app::InputCommands commands;
    if (commands.eof()) return 1;
    auto cancelled=[&] { commands.poll(); return stopped || commands.cancelled(); };

    chess::Game game;
    robot::Simulator arm;
    Pikafish engine;
    engine.cancelled=cancelled;
    std::vector<std::string> legal;
    std::string phase="waiting", message="人执红，点击起点和终点走棋；机械臂执行为仿真";
    std::string line,lastMove,source="virtual";
    bool fatal=false, needLegal=true, needThink=false;
    int demoRounds=0;
    auto last=cv::getTickCount();

    while (!stopped && !commands.eof() && !fatal) {
        commands.poll();
        if (commands.eof()) break;
        // 1. 每轮先取得真实引擎的合法走法；结果确认前不更改棋局历史。
        if (needLegal && !commands.cancelled() && game.result()==chess::Result::Playing) {
            show(game,arm,"thinking","正在核对合法走法",legal,lastMove,source);
            if (!engine.start(runtime.engine.c_str()) ||
                !engine.legalMoves(game.baseFen(),game.history(),legal) || !game.judge(legal)) {
                phase="error"; message=engine.error.empty()?game.error:engine.error;
            } else if (game.result()!=chess::Result::Playing) {
                phase="finished"; message="对局结束或重复局面待核对";
            }
            if (!engine.close()) { phase="error"; message=engine.error; fatal=true; }
            needLegal=false;
        }

        // 2. 处理人的走法或摄像头观察；按钮不会绕过合法性检查。
        bool hasCommand=commands.pop(line);
        if (!hasCommand && !commands.cancelled() && demoRounds>0 && phase=="waiting" && game.redToMove() && !legal.empty()) {
            line="move "+legal.front(); hasCommand=true;
        }
        if (hasCommand && !fatal) {
            if (line=="quit") break;
            if (line=="new") {
                game.reset(chess::initialBoard()); arm.reset(game.board());
                phase="waiting"; message="已重建标准开局的虚拟场景";
                legal.clear(); lastMove.clear(); source="virtual";
                needLegal=true; needThink=false; demoRounds=0;
            } else if (line=="pause") {
                arm.stop(); phase="paused"; message="已停止，保留中间场景；新局可复位虚拟世界";
                demoRounds=0; needThink=false;
            } else if (line.rfind("demo ",0)==0 && phase=="waiting") {
                std::istringstream in(line.substr(5)); int rounds=0; std::string extra;
                if (in>>rounds && !(in>>extra) && rounds>=1 && rounds<=20) {
                    demoRounds=rounds; message="开始虚拟闭环演示";
                }
            } else if ((line=="resign" || line=="draw") && !game.pending()) {
                bool ok=line=="resign" ? game.resign(true) : game.agreeDraw();
                if (ok) { phase="finished"; demoRounds=0; message="对局结束"; }
            } else if (phase=="waiting" && game.redToMove() && game.result()==chess::Result::Playing) {
                chess::Observation observation;
                bool observed=false;
                if (line.rfind("move ",0)==0) {
                    chess::Move move;
                    chess::Board after;
                    const std::string choice=line.substr(5);
                    if (std::find(legal.begin(),legal.end(),choice)!=legal.end() &&
                        chess::parseMove(choice,move) && chess::applyMove(game.board(),true,move,after,message)) {
                        observation=virtualObservation(after); observed=true; source="virtual";
                    } else message="这步走法不合法，棋局未改变";
                } else if (line=="scan") {
                    show(game,arm,"scanning","正在通过MIPI、OpenCV5和NPU识别棋盘",legal,lastMove,"camera");
                    observed=scanBoard(runtime,observation,cancelled,fatal,message);
                    if (observed) source="camera";
                    if (fatal) phase="error";
                    if (observed && observation.board==game.board()) {
                        observed=false; message="实物棋盘与当前确认棋局一致，等待走棋";
                    }
                }
                if (observed) {
                    chess::Move move;
                    const chess::Board before=game.board();
                    if (!game.acceptHuman(observation,legal,move)) message=game.error;
                    else if (!arm.observeHuman(before,move)) { phase="error"; message=arm.error(); }
                    else {
                        lastMove=chess::uci(move); message="已确认红方走法 "+lastMove;
                        needThink=true;
                    }
                }
            }
        }

        // 3. 模型输入已释放后，再让皮卡鱼计算；它只提供候选，不直接改盘面。
        if (needThink && !cancelled() && !fatal) {
            phase="thinking";
            show(game,arm,phase,"皮卡鱼思考中",legal,lastMove,source);
            std::string choice;
            bool ok=engine.start(runtime.engine.c_str()) &&
                    engine.legalMoves(game.baseFen(),game.history(),legal) && game.judge(legal);
            if (ok && game.result()==chess::Result::Playing)
                ok=engine.think(game.baseFen(),game.history(),runtime.thinkMs,choice);
            if (!ok) { phase="error"; message=engine.error.empty()?game.error:engine.error; }
            if (!engine.close()) { phase="error"; message=engine.error; fatal=true; }
            if (commands.cancelled() && !fatal) {
                arm.stop(); phase="paused"; message="已取消计算，等待新局复位"; demoRounds=0;
            } else if (ok && game.result()!=chess::Result::Playing && !fatal) {
                phase="finished"; message="对局结束或重复局面待核对";
            } else if (ok && !fatal) {
                chess::Move move;
                if (!game.prepareRobot(virtualObservation(arm.board()),choice,legal,move) ||
                    !arm.start(game.board(),move)) {
                    phase="error"; message=game.error.empty()?arm.error():game.error;
                } else { phase="moving"; lastMove=choice; message="虚拟机械臂执行 "+choice; }
            }
            needThink=false;
            last=cv::getTickCount();
        }

        // 4. 关节按时间推进，只有完整执行且虚拟观察一致才提交机器走法。
        const auto now=cv::getTickCount();
        const double dt=double(now-last)/cv::getTickFrequency();
        last=now;
        if (phase=="moving") {
            arm.step(std::min(dt,0.1)*runtime.speed);
            if (!arm.busy()) {
                if (!game.confirmRobot(virtualObservation(arm.board()),arm.succeeded())) {
                    phase="error"; message=arm.error().empty()?game.error:arm.error();
                } else {
                    phase="waiting"; message="机器走法已执行并在虚拟棋盘复核，轮到红方";
                    needLegal=true;
                    if (demoRounds>0) --demoRounds;
                }
            }
        }
        show(game,arm,phase,message,legal,lastMove,source);
        pollfd input{STDIN_FILENO,POLLIN,0};
        poll(&input,1,50);
    }
    if (!engine.close()) { phase="error"; message=engine.error; fatal=true; }
    arm.stop();
    return fatal ? 1 : (stopped ? 130 : 0);
}
