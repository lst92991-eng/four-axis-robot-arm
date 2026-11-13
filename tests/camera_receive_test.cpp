// 用可控读帧结果运行真实 capture.cpp，验证失败重试及主动关闭；不连接相机。
#include <opencv2/videoio.hpp>
#include <atomic>
#include <chrono>
#include <future>
#include <iostream>
#include <stdexcept>
#include <thread>

namespace mock {
std::atomic<int> remainingFailures = 0;
std::atomic<int> attempts = 0;
std::atomic<int> failedReads = 0;
std::atomic<int> activeReads = 0;
std::atomic<bool> alwaysFail = false;
std::atomic<bool> throwOnRead = false;
std::atomic<bool> pauseRead = false;
bool failOpen = false;
bool releasedWhileReading = false;
}

namespace cv {
class ScriptedCapture
{
public:
  bool open(const std::string&, int, const std::vector<int>&) { return !mock::failOpen; }
  bool read(Mat& frame)
  {
    int index = ++mock::attempts;
    if (mock::throwOnRead) throw std::runtime_error("Injected decoder failure");
    if (mock::alwaysFail || mock::remainingFailures>0)
    {
      if (mock::remainingFailures>0) --mock::remainingFailures;
      ++mock::failedReads;
      frame.release();
      return false;
    }
    ++mock::activeReads;
    while (mock::pauseRead) std::this_thread::sleep_for(std::chrono::milliseconds(1));
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    frame = Mat(720,1280,CV_8UC3,Scalar::all(index%255));
    --mock::activeReads;
    return true;
  }
  void release() { mock::releasedWhileReading |= mock::activeReads!=0; }
};
}

// 只替换视频源类型，接收循环、交帧、关闭均编译自生产文件。
#define VideoCapture ScriptedCapture
#include "vision/capture.cpp"
#undef VideoCapture

static void check(bool condition, const char *message)
{
  if (!condition) throw std::runtime_error(message);
}

int main()
{
  try
  {
    mock::failOpen = true;
    scara::Camera_open("unavailable");
    cv::Mat frame(2,2,CV_8UC3,cv::Scalar::all(1));
    bool unavailable = false;
    try { scara::Camera_read(frame); }
    catch (const std::runtime_error&) { unavailable = frame.empty(); }
    scara::Camera_close();
    check(unavailable && mock::attempts==0,
          "Failed open did not leave reception off and end through the reader");
    mock::failOpen = false;
    mock::remainingFailures = 3;
    scara::Camera_open("scripted");
    scara::Camera_read(frame);
    check(mock::failedReads==3 && mock::attempts>=4 && frame.size()==cv::Size(1280,720),
          "Read failure ended reception instead of retrying");
    scara::Camera_close();

    mock::alwaysFail = true;
    mock::attempts = 0;
    scara::Camera_open("scripted");
    auto waiting = std::async(std::launch::async, [] {
      cv::Mat output(2,2,CV_8UC3,cv::Scalar::all(1));
      try { scara::Camera_read(output); }
      catch (const std::runtime_error&) { return output.empty(); }
      return false;
    });
    auto start = std::chrono::steady_clock::now();
    bool stillWaiting = waiting.wait_for(std::chrono::milliseconds(5200))==std::future_status::timeout;
    auto elapsedMs = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now()-start).count();
    bool paced = mock::attempts>=2 && mock::attempts<=elapsedMs/5+5;
    auto stop = std::chrono::steady_clock::now();
    scara::Camera_stop();
    bool stopDelivered = waiting.wait_for(std::chrono::seconds(1))==std::future_status::ready;
    scara::Camera_close();
    check(stillWaiting, "No-frame wait still has an application timeout");
    check(paced, "Retry loop did not pace repeated immediate failures");
    check(stopDelivered, "Stop request did not interrupt a no-frame wait");
    check(waiting.get(), "Explicit close did not unblock the waiting reader with empty output");
    check(std::chrono::steady_clock::now()-stop<std::chrono::seconds(2),
          "Explicit close failed to stop the retry loop promptly");

    mock::alwaysFail = false;
    mock::pauseRead = true;
    scara::Camera_open("scripted");
    for (int i=0; i<1000 && mock::activeReads==0; ++i)
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    check(mock::activeReads==1, "Did not start the held read");
    auto closing = std::async(std::launch::async, [] { scara::Camera_close(); });
    bool waitedForRead = closing.wait_for(std::chrono::milliseconds(30))==std::future_status::timeout;
    mock::pauseRead = false;
    closing.get();
    check(waitedForRead && scara::latestFrame.empty(),
          "Close did not wait for the last frame and clear it at thread exit");

    mock::throwOnRead = true;
    scara::Camera_open("scripted");
    bool ended = false;
    try { scara::Camera_read(frame); }
    catch (const std::runtime_error&) { ended = frame.empty(); }
    scara::Camera_close();
    check(ended, "Decoder exception did not notify the waiting reader");
    mock::throwOnRead = false;
    scara::Camera_open("scripted");
    scara::Camera_read(frame);
    scara::Camera_close();
    check(!frame.empty() && !mock::releasedWhileReading, "Reopen failed or video closed while reading");
    std::cout << "PASS failed open, retry, wait beyond five seconds, explicit stop, final-frame cleanup, decoder exception\n";
    return 0;
  }
  catch (const std::exception& error)
  {
    mock::pauseRead = false;
    scara::Camera_close();
    std::cerr << error.what() << '\n';
    return 1;
  }
}
