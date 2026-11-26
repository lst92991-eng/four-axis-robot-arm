// 独立视觉预览入口，将相机画面识别成带框和分数的 JPEG 帧流。
#include "site.hpp"
#include "vision/capture.hpp"
#include "vision/yolo.hpp"
#include <opencv2/imgcodecs.hpp>
#include <csignal>
#ifdef _WIN32
#include <io.h>
#include <fcntl.h>
#endif
#include <iostream>
#include <stdexcept>

//---------预览停止区---------
static volatile std::sig_atomic_t running = 1;

static void requestStop(int)
{
    running = 0;
}

//---------模型与相机初始化区---------
int main(int argc, char **argv)
{
    using namespace scara;
    if (argc != 2) return 2;
    const char *modelPath = argv[1];
#ifdef _WIN32
    // Windows 上把标准输出切到二进制模式，保留 JPEG 的原始字节。
    _setmode(_fileno(stdout), _O_BINARY);
#endif
    int result = 1;
    try
    {
        const cv::Point cropOrigin = Site_loadCrop();
        std::signal(SIGINT, requestStop);
        std::signal(SIGTERM, requestStop);
        result = Yolo_open(modelPath) ? 0 : 1;
        if (result == 0 && running) Camera_open();
        else running = 0;

        //---------取帧、识别与绘制区---------
        while (running)
        {
            cv::Mat source = Camera_read(); // 摄像头解码得到的 BGR 原图。
            if (!running) break;
            if (source.size() != cv::Size(ImageWidth, ImageHeight))
                throw std::runtime_error("相机未返回 1280×720 原图");
            cv::Mat input = Vision_prepare(source, cropOrigin);
            std::vector<Detection> pieces;
            if (!Yolo_run(input, cropOrigin, pieces)) throw std::runtime_error("NPU 推理失败");
            Vision_draw(source, pieces);
            // JPEG 帧直接送给播放器的管道。
            std::vector<unsigned char> jpeg;
            if (!cv::imencode(".jpg", source, jpeg)) throw std::runtime_error("预览编码失败");
            std::cout.write(reinterpret_cast<const char *>(jpeg.data()), jpeg.size()).flush();
            if (!std::cout) running = 0;
        }
    }
    catch (const std::exception &e)
    {
        if (running)
        {
            result = 1;
            std::cerr << e.what() << '\n';
        }
    }
    //---------预览收尾区---------
    // 释放视频流和 NPU 的输入输出内存。
    Camera_close();
    Yolo_close();
    return result;
}
