// 替换视频源，核对每次调用只读一次，以及失败时的输出。
#include <opencv2/videoio.hpp>
#include <iostream>
#include <stdexcept>

namespace mock {
int reads = 0;
bool dropFrame = false, decoderError = false;
}

namespace cv {
class ScriptedCapture
{
    bool opened = false;
public:
    bool open(const std::string& url) { return opened = url == "scripted"; }
    void release() { opened = false; }
    bool read(Mat& frame)
    {
        ++mock::reads;
        if (!opened || mock::dropFrame) { frame.release(); return false; }
        frame.create(2, 2, CV_8UC1);
        frame.setTo(Scalar::all(mock::reads));
        if (mock::decoderError) CV_Error(Error::StsError, "Injected decode failure");
        return true;
    }
};
}

#define VideoCapture ScriptedCapture
#include "vision/capture.cpp"
#undef VideoCapture

static void check(bool ok, const char *message)
{
    if (!ok) throw std::runtime_error(message);
}

int main()
{
    using namespace scara;
    try
    {
        Camera_open("unavailable");
        check(Camera_read().empty(), "Failed open returned stale data");
        Camera_open("scripted");
        check(mock::reads == 1, "Open read ahead of the caller");
        mock::dropFrame = true;
        check(Camera_read().empty() && mock::reads == 2, "Read retried a missing frame");

        mock::dropFrame = false;
        cv::Mat previous = Camera_read();
        check(!previous.empty() && previous.at<uchar>(0, 0) == 3, "Next call failed to recover");
        cv::Mat next = Camera_read();
        check(!next.empty() && previous.at<uchar>(0, 0) == 3 && next.at<uchar>(0, 0) == 4,
              "Another read modified the previous frame");

        mock::decoderError = true;
        bool decoderErrorSeen = false;
        try { Camera_read(); }
        catch (const cv::Exception &e) { decoderErrorSeen = e.err == "Injected decode failure"; }
        check(decoderErrorSeen && mock::reads == 5, "Lost the original decoder error");
        mock::decoderError = false;
        Camera_close();
        check(Camera_read().empty(), "Closed source delivered a frame");
        Camera_open("scripted");
        check(!Camera_read().empty(), "Reopen did not deliver a frame");
        Camera_close();
        std::cout << "PASS one read per call, failure, recovery, frame lifetime, close and reopen\n";
        return 0;
    }
    catch (const std::exception& e)
    {
        Camera_close();
        std::cerr << e.what() << '\n';
        return 1;
    }
}
