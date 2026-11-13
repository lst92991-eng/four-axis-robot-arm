#include "capture.hpp"
#include <opencv2/videoio.hpp>
#include <thread>
#include <mutex>
#include <condition_variable>

namespace scara
{
static cv::VideoCapture Video;
static cv::Mat Latest;
static std::thread Reader;
static std::mutex Mutex;
static std::condition_variable Ready;
static bool Running = false;

// 线程只负责持续取流；任何读取失败都变成“无新帧”，由主流程结束。
static void receiveFrames()
{
    try
    {
        for (;;)
        {
            cv::Mat frame;
            if (!Video.read(frame)) break;
            std::lock_guard<std::mutex> lock(Mutex);
            if (!Running) break;
            Latest = frame;
            Ready.notify_one();
        }
    }
    catch (...)
    {
    } // OpenCV 异常同样进入下面的停止出口。
    std::lock_guard<std::mutex> lock(Mutex);
    Running = false;
    Ready.notify_all();
}

bool Camera_open(const std::string &source)
{
    Camera_close();
    if (!Video.open(source, cv::CAP_FFMPEG,
                    {cv::CAP_PROP_OPEN_TIMEOUT_MSEC, 5000, cv::CAP_PROP_READ_TIMEOUT_MSEC, 5000}))
        return false;
    Running = true;
    Reader = std::thread(receiveFrames);
    return true;
}

bool Camera_read(cv::Mat &frame)
{
    std::unique_lock<std::mutex> lock(Mutex);
    Latest.release();
    while (Running && Latest.empty())
        Ready.wait(lock);
    frame = Latest;
    return !frame.empty();
}

void Camera_close()
{
    {
        std::lock_guard<std::mutex> lock(Mutex);
        Running = false;
        Latest.release();
    }
    Ready.notify_all();
    if (Reader.joinable()) Reader.join();
    Video.release();
}
} // namespace scara
