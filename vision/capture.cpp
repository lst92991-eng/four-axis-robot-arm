// 后台持续解码视频，前台取一张新原图交给 YOLO。
#include "capture.hpp"
#include <opencv2/videoio.hpp>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <stdexcept>
#include <atomic>
#include <chrono>
#include <vector>

namespace scara
{
//---------相机与交帧数据区---------
static cv::VideoCapture video;              // OpenCV 视频流，负责连接与解码。
static cv::Mat latestFrame;                 // 只保留后台收到的最新一帧 BGR 原图。
static std::thread captureThread;           // 持续取流的后台线程。
static std::mutex frameMutex;               // 交帧时使用的锁，保护缓存和接收状态。
static std::condition_variable frameReady;  // 新帧到达或接收结束时，唤醒等候者。
static std::atomic<bool> capturing = false; // 接收开关，允许两个线程直接读取和设置。
static const std::vector<int> cameraOptions = {
    cv::CAP_PROP_OPEN_TIMEOUT_MSEC, 5000, // 打开连接的等待上限，毫秒。
    cv::CAP_PROP_READ_TIMEOUT_MSEC, 5000  // 单次读帧的等待上限，毫秒。
};

static void receiveFrames();

//---------打开相机区---------
void Camera_open(const std::string &cameraUrl)
{
    Camera_close(); // 允许再次打开：先结束上一次接收。
    capturing = video.open(cameraUrl, cv::CAP_FFMPEG, cameraOptions);
    if (capturing) captureThread = std::thread(receiveFrames);
}

//---------向主流程交帧区---------
void Camera_read(cv::Mat &frame, int timeoutMs)
{
    std::unique_lock<std::mutex> lock(frameMutex);
    latestFrame.release(); // 丢弃调用前缓存的图，等调用后收到的新帧。
    // timeoutMs 约束调用方的总等待时间，FFmpeg 的超时单独约束一次底层读取。
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeoutMs);
    while (capturing && latestFrame.empty() && (timeoutMs <= 0 || std::chrono::steady_clock::now() < deadline))
        frameReady.wait_for(lock, std::chrono::milliseconds(10)); // 及时检查停止请求和超时。
    // 将图像交给输出参数，共享像素，不复制整张图。
    frame = latestFrame;
    if (frame.empty()) throw std::runtime_error(capturing ? "相机读取超时，未收到新帧" : "Camera stream ended");
}

// 停止回调只改原子开关；不在信号回调中加锁、等线程或操作视频。
void Camera_stop()
{
    capturing = false;
}

//---------关闭相机区---------
void Camera_close()
{
    Camera_stop();
    frameReady.notify_all();
    if (captureThread.joinable()) captureThread.join(); // 等后台结束，避免边读边关闭。
    video.release();
}
//---------后台接收区---------
// 推理期间也持续读流，新帧覆盖缓存，避免视频在接收端积压。
static void receiveFrames()
{
    try
    {
        while (capturing)
        {
            cv::Mat frame; // 每轮新建，避免下一次解码改写主线程正在使用的图。
            // video.read 可能等待网络数据，读完才锁住共享帧缓存。
            bool received = video.read(frame);
            std::unique_lock<std::mutex> lock(frameMutex);
            if (received && !frame.empty())
            {
                // 交换 Mat 引用后通知等图的线程，推理可以与下一帧解码同时进行。
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
    // 退出接收线程时，在同一把锁下清除缓存并唤醒所有等图者。
    std::lock_guard<std::mutex> lock(frameMutex);
    capturing = false;
    latestFrame.release(); // 当前读取完成后统一清空，关闭时允许这一轮收尾。
    frameReady.notify_all();
}

} // namespace scara
