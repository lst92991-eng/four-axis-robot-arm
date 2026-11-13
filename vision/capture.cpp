#include "capture.hpp"
#include <opencv2/videoio.hpp>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <stdexcept>
#include <atomic>
#include <chrono>

namespace scara
{
//---------相机与交帧数据区---------
static cv::VideoCapture video;              // OpenCV 视频流，负责连接与解码。
static cv::Mat latestFrame;                 // 只保留后台收到的最新一帧 BGR 原图。
static std::thread captureThread;           // 持续取流的后台线程。
static std::mutex frameMutex;               // 交帧时使用的锁，保护缓存和接收状态。
static std::condition_variable frameReady;  // 新帧到达或接收结束时，唤醒等候者。
static std::atomic<bool> capturing = false; // 接收开关，允许两个线程直接读取和设置。

//---------后台接收区---------
// 推理期间也持续读流，新帧覆盖缓存，避免视频在接收端积压。
static void receiveFrames()
{
    try
    {
        while (capturing)
        {
            cv::Mat frame; // 每轮新建，避免下一次解码改写主线程正在使用的图。
            bool received = video.read(frame);
            std::unique_lock<std::mutex> lock(frameMutex);
            if (received && capturing)
            {
                latestFrame = frame;
                frameReady.notify_one();
            }
            else
                frameReady.wait_for(lock, std::chrono::milliseconds(10)); // 暂时没图，解锁稍等后重试。
        }
    }
    catch (...)
    {
    } // 后台异常也走下面的结束通知，不能让主线程一直等图。
    std::lock_guard<std::mutex> lock(frameMutex);
    capturing = false;
    frameReady.notify_all();
}

//---------打开相机区---------
void Camera_open(const std::string &cameraUrl)
{
    Camera_close(); // 允许再次打开：先结束上一次接收。
    // 由 FFmpeg 解码；打开和读取的等待上限均为 5000 毫秒。
    if (!video.open(cameraUrl, cv::CAP_FFMPEG,
                    {cv::CAP_PROP_OPEN_TIMEOUT_MSEC, 5000, cv::CAP_PROP_READ_TIMEOUT_MSEC, 5000}))
        throw std::runtime_error("Cannot open camera stream");
    capturing = true;
    captureThread = std::thread(receiveFrames);
}

//---------向主流程交帧区---------
void Camera_read(cv::Mat &frame)
{
    std::unique_lock<std::mutex> lock(frameMutex);
    frame.release();
    latestFrame.release(); // 丢弃调用前缓存的图，等调用后收到的新帧。
    auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (capturing && latestFrame.empty())
    {
        if (frameReady.wait_until(lock, deadline) == std::cv_status::timeout && latestFrame.empty())
            throw std::runtime_error("Camera frame timeout");
    }
    frame = latestFrame; // 将图像交给输出参数，共享像素，不复制整张图。
    if (frame.empty()) throw std::runtime_error("Camera stream ended");
}

//---------关闭相机区---------
void Camera_close()
{
    {
        std::lock_guard<std::mutex> lock(frameMutex);
        capturing = false;
        latestFrame.release();
    } // 先解锁，后台才能完成结束通知。
    frameReady.notify_all();
    if (captureThread.joinable()) captureThread.join(); // 等后台结束，避免边读边关闭。
    video.release();
}
} // namespace scara
