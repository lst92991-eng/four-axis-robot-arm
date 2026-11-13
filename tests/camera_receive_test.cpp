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
bool releasedWhileReading = false;
}

namespace cv {
class ScriptedCapture
{
public:
  bool open(const std::string&, int, const std::vector<int>&) { return true; }
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
    mock::remainingFailures = 3;
    scara::Camera_open("scripted");
    cv::Mat frame;
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
    std::this_thread::sleep_for(std::chrono::milliseconds(120));
    check(waiting.wait_for(std::chrono::milliseconds(0))==std::future_status::timeout,
          "A temporary read failure woke the caller as a terminal error");
    auto elapsedMs = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now()-start).count();
    check(mock::attempts>=2 && mock::attempts<=elapsedMs/5+5,
          "Retry loop did not pace repeated immediate failures");
    auto stop = std::chrono::steady_clock::now();
    scara::Camera_close();
    check(waiting.get(), "Explicit close did not unblock the waiting reader with empty output");
    check(std::chrono::steady_clock::now()-stop<std::chrono::seconds(2),
          "Explicit close failed to stop the retry loop promptly");

    mock::alwaysFail = false;
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
    std::cout << "PASS transient read failures recover, retries paced, explicit close unblocks, decoder exception exits\n";
    return 0;
  }
  catch (const std::exception& error)
  {
    scara::Camera_close();
    std::cerr << error.what() << '\n';
    return 1;
  }
}
