#include "capture.hpp"
#include <opencv2/videoio.hpp>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <stdexcept>
#include <thread>

namespace scara {
//---------摄像头数据区---------
static struct CameraState
{
  cv::VideoCapture video;
  cv::Mat frame;
  std::thread task;
  std::mutex mutex;
  std::condition_variable ready;
  std::atomic<bool> stopping{true};
  std::string error;
  ~CameraState() { Camera_close(); }
} Capture;

static void receiveFrames();
static bool frameReady();

//---------打开摄像头---------
void Camera_open(const std::string& source)
{
  Camera_close();
  if (!Capture.video.open(source, cv::CAP_FFMPEG,
       {cv::CAP_PROP_OPEN_TIMEOUT_MSEC,5000,cv::CAP_PROP_READ_TIMEOUT_MSEC,5000}))
    throw std::runtime_error("Cannot open camera stream");
  Capture.error.clear();
  Capture.stopping = false;
  Capture.task = std::thread(receiveFrames);
}

//---------取得下一帧---------
cv::Mat Camera_read()
{
  std::unique_lock<std::mutex> lock(Capture.mutex);
  // 丢弃上次留下的图像，等待后台再次接收后唤醒。
  Capture.frame.release();
  if (!Capture.ready.wait_for(lock, std::chrono::seconds(5), frameReady))
    throw std::runtime_error("Camera frame timeout");
  if (Capture.frame.empty())
    throw std::runtime_error(Capture.error.empty() ? "Camera stopped" : Capture.error);
  // Mat 引用计数接管这一帧；后台下一次接收使用另一张图，不修改已交出的像素。
  cv::Mat frame = Capture.frame;
  Capture.frame.release();
  return frame;
}

//---------关闭摄像头---------
void Camera_close()
{
  Capture.stopping = true;
  Capture.ready.notify_all();
  if (Capture.task.joinable()) Capture.task.join();
  Capture.video.release();
  Capture.frame.release();
}

//---------后台取流---------
// 持续读取视频流，图像就绪时唤醒 Camera_read。
static void receiveFrames()
{
  try
  {
    while (!Capture.stopping)
    {
      cv::Mat frame;
      if (!Capture.video.read(frame)) throw std::runtime_error("Camera stream ended");
      std::lock_guard<std::mutex> lock(Capture.mutex);
      Capture.frame = frame;
      Capture.ready.notify_one();
    }
  }
  catch (const std::exception& error)
  {
    std::lock_guard<std::mutex> lock(Capture.mutex);
    Capture.error = error.what();
    Capture.ready.notify_one();
  }
}

static bool frameReady()
{
  return !Capture.frame.empty() || !Capture.error.empty() || Capture.stopping;
}
}
