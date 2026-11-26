// 在桌面测试 RV1106 对接代码，用替身提供相机、NPU、引擎和驱动反馈。
#include "app/game.hpp"
#include "motion/execute.hpp"
#include "chess/pikafish.hpp"
#include "vision/capture.hpp"
#include "app/site.hpp"
#include <opencv2/imgcodecs.hpp>
#include <poll.h>
#include <cstring>
#include <fstream>
#include <iterator>
#include <iostream>
#ifdef __linux__
#include <sys/syscall.h>
extern "C" long syscall(long, ...) noexcept;
#endif

namespace fake
{
bool camera = false, npu = false, engine = false, failSearch = false, failModel = false;
bool motionSuccess = true, stopSuccess = true, cancelMotion = false;
cv::Size frameSize{1280, 720};
int executions = 0, stops = 0;
std::vector<scara::PickStep> receivedSteps;
std::vector<std::string> events;
std::string keyboard;
bool eof = false;
void check(bool value, const char *message)
{
    if (!value) throw std::runtime_error(message);
}
} // namespace fake

bool Pikafish_query(const char *, const chess::Game &, int, const volatile std::sig_atomic_t &,
                    chess::MoveList &legal, chess::Move &choice)
{
    fake::check(!fake::camera && !fake::npu, "Engine started while vision still held resources");
    fake::events.push_back("engine-open");
    legal = {{6,0,5,0}};
    choice = {6,0,5,0};
    return !fake::failSearch;
}
bool Pikafish_close()
{
    fake::engine = false;
    return true;
}

namespace scara
{
bool Motion_execute(const std::vector<PickStep> &steps, const MotionLimits &,
                    const volatile std::sig_atomic_t &running)
{
    fake::check(!fake::camera && !fake::npu && !fake::engine, "Motion started before resource cleanup");
    if (!running) return false;
    ++fake::executions;
    fake::receivedSteps = steps;
    if (fake::cancelMotion)
    {
        gameRunning = 0; // 模拟执行过程中收到停止信号，不发送实际命令。
        Motion_stop();
        return false;
    }
    return fake::motionSuccess;
}
bool Motion_stop()
{
    ++fake::stops;
    return fake::stopSuccess;
}
void Camera_open()
{
    fake::camera = true;
    fake::events.push_back("camera-open");
}
cv::Mat Camera_read()
{
    fake::check(fake::camera, "Read without camera");
    fake::events.push_back("read");
    return cv::Mat(fake::frameSize, CV_8UC3, cv::Scalar(100, 100, 100));
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
bool Yolo_run(const cv::Mat &, cv::Point, std::vector<Detection> &pieces)
{
    fake::check(fake::npu, "Run without NPU");
    fake::events.push_back("run");
    pieces = {{13, .9, {100, 100, 20, 20}}};
    return true;
}
void Yolo_close()
{
    fake::npu = false;
    fake::events.push_back("npu-close");
}
cv::Mat Vision_prepare(const cv::Mat &, cv::Point)
{
    fake::events.push_back("prepare");
    return cv::Mat(640, 640, CV_8UC3);
}
} // namespace scara
extern "C" int poll(pollfd *, unsigned long, int)
{
    if (fake::eof) return 1;
    if (fake::keyboard.empty()) scara::gameRunning = 0;
    return fake::keyboard.empty() ? 0 : 1;
}
extern "C" std::ptrdiff_t read(int descriptor, void *buffer, size_t size)
{
#ifdef __linux__
    // 只替换终端输入，其余读取仍交给系统。
    if (descriptor != 0) return syscall(SYS_read, descriptor, buffer, size);
#endif
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
        StaticParams staticParams;
        DynamicParams dynamicParams;
        // 未导入的旧表应在连接设备前报错。
        bool needsImport = false;
        try { Site_loadFile(staticParams, "tests/fixtures/game-app/installation-v1.toml"); }
        catch (const std::runtime_error &error) { needsImport = std::string(error.what()).find("15-site") != std::string::npos; }
        fake::check(needsImport, "Unconverted installation file did not request import");
        std::ifstream newInput("config/installation.toml");
        std::string newText{std::istreambuf_iterator<char>(newInput), {}};
        auto cameraSection = newText.find("[camera]");
        auto nextSection = newText.find("\n[", cameraSection);
        newText.replace(cameraSection, nextSection - cameraSection, "[camera]\ncrop_px = [305, 50]\n");
        Site_parse(newText, staticParams);
        fake::check(staticParams.visionCropOrigin == cv::Point(305, 50), "Named crop origin changed");
        Site_loadFile(staticParams, "config/installation.toml");
        staticParams.engineDir = "fake-engine";
        Runtime_open(staticParams, "fake-model", dynamicParams);
        std::string frame, previous;
        chess::Move choice;
        std::vector<Detection> pieces;
        chess::MoveList legal;
        Vision_observe(staticParams.layout, frame, pieces);
        fake::check(fake::events ==
                        std::vector<std::string>{"npu-open", "camera-open", "read", "prepare", "run"},
                    "Live observation no longer follows the verified order");
        fake::check(!frame.empty() && pieces.size() == 1 && pieces[0].classId == 13,
                    "Live observation did not return the new frame and detections");
        previous = frame;
        for (cv::Size size : {cv::Size{}, cv::Size(640, 480)})
        {
            fake::frameSize = size;
            fake::events.clear();
            bool readFailed = false;
            try { Vision_observe(staticParams.layout, frame, pieces); }
            catch (const std::runtime_error&) { readFailed = true; }
            fake::check(readFailed && frame == previous && fake::events == std::vector<std::string>{"read"},
                        "An invalid frame reached inference or was published");
        }
        fake::check(Runtime_close() && fake::stops == 1 && !fake::camera && !fake::npu,
                    "Reset after a camera failure did not stop and close the devices");
        fake::frameSize = {1280, 720};
        Runtime_open(staticParams, "fake-model", dynamicParams);
        dynamicParams.game.humanRed = false;
        Engine_query(dynamicParams.game, legal, choice);
        fake::check(choice == chess::Move{6,0,5,0} && !fake::camera && !fake::npu && !fake::engine,
                    "Search retained overlapping resources");
        fake::check(legal == chess::MoveList{{6,0,5,0}}, "GUI selected a different numeric move");
        Vision_observe(staticParams.layout, frame, pieces);
        fake::check(frame != previous, "Observation identity was reused after search");

        PickStep lift, release;
        lift.action = PickAction::LiftZ;
        lift.target[0] = 123;
        release.action = PickAction::Release;
        release.durationMs = 0;
        std::vector<PickStep> steps{lift, release};
        bool completed = false;
        completed = Robot_execute("move", {6,0,5,0}, steps);
        fake::check(completed && fake::executions == 1 && fake::receivedSteps.size() == 2,
                    "Direct execution did not receive the action array or return completion");
        fake::check(fake::receivedSteps[0].target[0] == 123 && fake::receivedSteps[0].action == PickAction::LiftZ &&
                        fake::receivedSteps[1].action == PickAction::Release &&
                        fake::receivedSteps[1].durationMs == 0,
                    "Direct interface changed action units or pulse duration");
        fake::motionSuccess = false;
        completed = Robot_execute("move", {6,0,5,0}, steps);
        fake::check(!completed, "Failed direct execution became success");
        fake::motionSuccess = true;
        int beforeCancel = fake::stops;
        fake::cancelMotion = true;
        completed = Robot_execute("move", {6,0,5,0}, steps);
        fake::check(!gameRunning && !completed && fake::stops == beforeCancel + 1, "Cancellation did not stop and leave execution");
        int executed = fake::executions;
        completed = Robot_execute("move", {6,0,5,0}, steps);
        fake::check(!completed && fake::executions == executed, "A stopped run started another move");
        fake::cancelMotion = false;
        gameRunning = 1;
        fake::stopSuccess = false;
        fake::check(!Runtime_close(), "Failed stop was reported as confirmed");
        fake::stopSuccess = true;
        fake::check(Runtime_close(), "Confirmed stop or cleanup failed");

        fake::failSearch = true;
        bool failed = false;
        try
        {
            Engine_query(dynamicParams.game, legal, choice);
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
            Vision_observe(staticParams.layout, frame, pieces);
        }
        catch (const std::exception &)
        {
            failed = true;
        }
        Runtime_close();
        fake::check(failed && !fake::camera && !fake::npu, "Partially opened model was not released");
        fake::keyboard = "unknown\nready\n";
        Command command = Operator_read("fixture");
        fake::check(command == Command::Ready, "Console command parsing failed");
        fake::keyboard = "draw\nready\n";
        fake::check(Operator_prepare(), "Console preparation did not wait for confirmation");
        fake::keyboard = "ready\n";
        fake::check(Operator_confirmOpening(dynamicParams.observed), "Console opening confirmation did not read the operator input");
        fake::keyboard = "draw\n";
        fake::check(!Operator_confirmOpening(dynamicParams.observed) && gameRunning, "Draw approved the opening");
        fake::keyboard = "quit\n";
        fake::check(!Operator_confirmOpening(dynamicParams.observed) && !gameRunning, "Opening quit did not stop the game");
        //退出命令和EOF均由输入入口设置停止标志，机械收尾仍交给主程序
        for (const char *input : {"quit\n", ""})
        {
            fake::keyboard = input;
            fake::eof = fake::keyboard.empty();
            gameRunning = 1;
            int beforeStop = fake::stops;
            command = Operator_read("exit fixture");
            fake::check(command == Command::Quit && !gameRunning && fake::stops == beforeStop,
                        "Quit/EOF did not leave through nonblocking input shutdown");
        }
        fake::eof = false;
        gameRunning = 1;
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
