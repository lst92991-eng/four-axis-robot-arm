// Compile the actual live wrapper, replacing only its external devices/process API.
#include "app/game_files.hpp"
#include "motion/execute.hpp"
#include "chess/pikafish.hpp"
#include "vision/capture.hpp"
#include "vision/io.hpp"
#include <opencv2/imgcodecs.hpp>
#include <poll.h>
#include <cstring>
#include <iostream>

volatile std::sig_atomic_t scara::gameRunning = 1;
namespace fake
{
bool camera = false, npu = false, engine = false, failSearch = false, failModel = false;
bool motionSuccess = true, stopSuccess = true, cancelMotion = false;
int executions = 0, stops = 0;
std::vector<scara::PickStep> receivedSteps;
std::vector<std::string> events;
std::string keyboard;
void check(bool value, const char *message)
{
    if (!value) throw std::runtime_error(message);
}
} // namespace fake

bool Pikafish::start(const char *)
{
    fake::check(!fake::camera && !fake::npu, "Engine started while vision still held resources");
    fake::engine = true;
    fake::events.push_back("engine-open");
    return true;
}
bool Pikafish::legalMoves(const chess::Game &, std::vector<std::string> &out)
{
    out = {"a3a4"};
    error = fake::failSearch ? "fixture error" : "";
    return !fake::failSearch;
}
bool Pikafish::think(const chess::Game &, int, std::string &out)
{
    out = "a3a4";
    return true;
}
bool Pikafish::close()
{
    fake::engine = false;
    return true;
}

namespace scara
{
void Motion_execute(const std::vector<PickStep> &steps, const volatile std::sig_atomic_t &running, bool &success)
{
    fake::check(!fake::camera && !fake::npu && !fake::engine, "Motion started before resource cleanup");
    success = false;
    if (!running) return;
    ++fake::executions;
    fake::receivedSteps = steps;
    if (fake::cancelMotion)
    {
        gameRunning = 0; // 模拟执行过程中收到停止信号，不发送实际命令。
        Motion_stop();
        return;
    }
    success = fake::motionSuccess;
}
bool Motion_stop()
{
    ++fake::stops;
    return fake::stopSuccess;
}
void Camera_open(const std::string &)
{
    fake::camera = true;
    fake::events.push_back("camera-open");
}
void Camera_read(cv::Mat &image)
{
    fake::check(fake::camera, "Read without camera");
    image = cv::Mat(720, 1280, CV_8UC3, cv::Scalar(100, 100, 100));
    fake::events.push_back("read");
}
void Camera_stop()
{
}
void Camera_close()
{
    fake::camera = false;
    fake::events.push_back("camera-close");
}
bool Yolo_open(std::string)
{
    fake::check(!fake::engine, "Vision restarted before engine was reaped");
    fake::npu = true;
    fake::events.push_back("npu-open");
    return !fake::failModel;
}
void Yolo_run(const cv::Mat &, YoloHead heads[YoloHeads])
{
    fake::check(fake::npu, "Run without NPU");
    for (int i = 0; i < YoloHeads; ++i)
        heads[i] = {};
    fake::events.push_back("run");
}
void Yolo_close()
{
    fake::npu = false;
    fake::events.push_back("npu-close");
}
void Vision_setup(cv::Size, cv::Rect)
{
}
void Vision_prepare(const cv::Mat &, cv::Mat &image)
{
    image = cv::Mat(640, 640, CV_8UC3);
    fake::events.push_back("prepare");
}
void Vision_decode(const YoloHead[YoloHeads], std::vector<Detection> &pieces)
{
    pieces = {{13, .9, {100, 100, 20, 20}}};
    fake::events.push_back("decode");
}
void Vision_draw(const cv::Mat &source, const std::vector<Detection> &, const cv::Mat &, cv::Mat &preview)
{
    preview = source.clone();
    fake::events.push_back("draw");
}
} // namespace scara
extern "C" int poll(pollfd *, unsigned long, int)
{
    if (fake::keyboard.empty()) scara::gameRunning = 0;
    return fake::keyboard.empty() ? 0 : 1;
}
extern "C" std::ptrdiff_t read(int, void *buffer, size_t)
{
    if (fake::keyboard.empty()) return 0;
    *static_cast<char *>(buffer) = fake::keyboard.front();
    fake::keyboard.erase(0, 1);
    return 1;
}

int main()
{
    try
    {
        using namespace scara;
        GameConfig config;
        Session session;
        Match_load("tests/fixtures/game-app/simulation.json", config, session);
        config.cameraUrl = "fake-stream";
        config.labelPath = "config/vision/chinese-labels.png";
        config.engineDir = "fake-engine";
        config.workDir = "output/game-direct-live-wrapper-test";
        Runtime_open(config, "fake-model");
        std::string frame, previous, choice;
        std::vector<Detection> pieces;
        std::vector<std::string> legal;
        Vision_observe(session.layout, frame, pieces);
        fake::check(fake::events ==
                        std::vector<std::string>{"npu-open", "camera-open", "read", "prepare", "run", "decode", "draw"},
                    "Live observation no longer follows the verified order");
        std::string bytes;
        readBytes(Runtime_path("preview.jpg"), bytes);
        auto preview = cv::imdecode(std::vector<unsigned char>(bytes.begin(), bytes.end()), cv::IMREAD_COLOR);
        fake::check(preview.cols == 2560 && preview.rows == 720, "Preview changed full-frame geometry");
        previous = frame;
        session.game.humanRed = false;
        Engine_query(session.game, legal, choice);
        fake::check(choice == "a3a4" && !fake::camera && !fake::npu && !fake::engine,
                    "Search retained overlapping resources");
        Vision_observe(session.layout, frame, pieces);
        fake::check(frame != previous, "Observation identity was reused after search");

        PickStep lift, release;
        lift.action = PickAction::MoveZ;
        lift.zDeg = 123;
        lift.speedDegS = 180;
        release.action = PickAction::ReleasePulse;
        release.durationMs = 20;
        std::vector<PickStep> steps{lift, release};
        bool completed = false;
        Robot_execute("move", "a3a4", steps, completed);
        fake::check(completed && fake::executions == 1 && fake::receivedSteps.size() == 2,
                    "Direct execution did not receive the action array or return completion");
        fake::check(fake::receivedSteps[0].zDeg == 123 && fake::receivedSteps[0].speedDegS == 180 &&
                        fake::receivedSteps[1].action == PickAction::ReleasePulse &&
                        fake::receivedSteps[1].durationMs == 20,
                    "Direct interface changed action units or pulse duration");
        fake::motionSuccess = false;
        Robot_execute("move", "a3a4", steps, completed);
        fake::check(!completed, "Failed direct execution became success");
        fake::motionSuccess = true;
        fake::cancelMotion = true;
        bool cancelled = false;
        try
        {
            Robot_execute("move", "a3a4", steps, completed);
        }
        catch (const std::exception &)
        {
            cancelled = true;
        }
        fake::check(cancelled && !completed && fake::stops == 1, "Cancellation did not stop and leave execution");
        int executed = fake::executions;
        try
        {
            Robot_execute("move", "a3a4", steps, completed);
        }
        catch (const std::exception &)
        {
        }
        fake::check(!completed && fake::executions == executed, "A stopped run started another move");
        fake::cancelMotion = false;
        gameRunning = 1;
        int stopped = fake::stops;
        Runtime_stop();
        fake::check(fake::stops == stopped, "Signal callback performed blocking hardware I/O");
        fake::stopSuccess = false;
        fake::check(!Runtime_close(), "Failed stop was reported as confirmed");
        fake::stopSuccess = true;
        fake::check(Runtime_close(), "Confirmed stop or cleanup failed");

        fake::failSearch = true;
        bool failed = false;
        try
        {
            Engine_query(session.game, legal, choice);
        }
        catch (const std::exception &)
        {
            failed = true;
        }
        fake::check(failed && !fake::camera && !fake::npu && !fake::engine,
                    "Engine failure became a usable empty move list");
        fake::failModel = true;
        failed = false;
        try
        {
            Vision_observe(session.layout, frame, pieces);
        }
        catch (const std::exception &)
        {
            failed = true;
        }
        Runtime_close();
        fake::check(failed && !fake::camera && !fake::npu, "Partially opened model was not released");
        fake::keyboard = "unknown\nready\n";
        Command command;
        Operator_read("human", "fixture", command);
        fake::check(command == Command::Ready, "Console command parsing failed");
        Runtime_stop();
        fake::check(Runtime_close(), "Live wrapper cleanup failed");
        std::cout
            << "PASS actual live wrapper: vision, engine, direct execution/result/cancellation/stop; external APIs mocked\n";
        return 0;
    }
    catch (const std::exception &error)
    {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
