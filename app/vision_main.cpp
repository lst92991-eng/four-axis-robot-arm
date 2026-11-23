// 独立视觉预览入口，将相机画面识别成带框和分数的 JPEG 帧流。
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
    scara::Camera_stop();
}

//---------模型与相机初始化区---------
int main(int argc, char **argv)
{
    using namespace scara;
    if (argc != 3) return 2;
    // 模型路径交给 RKNN，相机地址交给 OpenCV 取流。
    const char *modelPath = argv[1];
    const char *cameraUrl = argv[2];
#ifdef _WIN32
    // Windows 上把标准输出切到二进制模式，保留 JPEG 的原始字节。
    _setmode(_fileno(stdout), _O_BINARY);
#endif
    int result = 1;
    try
    {
        Vision_setup();
        std::signal(SIGINT, requestStop);
        std::signal(SIGTERM, requestStop);
        result = Yolo_open(modelPath) ? 0 : 1;
        if (result == 0 && running) Camera_open(cameraUrl);
        else running = 0;

        //---------取帧、识别与绘制区---------
        while (running)
        {
            cv::Mat source;                // 摄像头解码得到的 BGR 原图。
            cv::Mat input;                 // 裁剪、转 RGB、缩放补边后的 640×640 图。
            cv::Mat preview;               // 画好框、中文标签和分数的原尺寸预览图。
            YoloHead heads[YoloHeads];     // NPU 三路输出的地址和格式描述。
            std::vector<Detection> pieces; // 检测到的棋子：类别、分数、原图位置框。

            // 从相机取得新帧，经预处理、推理、解码后，把检测结果画回原图。
            Camera_read(source);
            Vision_prepare(source, input);
            Yolo_run(input, heads);
            Vision_decode(heads, pieces);
            Vision_draw(source, pieces, preview);
            // JPEG 帧直接送给播放器的管道。
            std::vector<unsigned char> jpeg;
            if (!cv::imencode(".jpg", preview, jpeg)) throw std::runtime_error("预览编码失败");
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
    // 先等取流线程结束，再释放 NPU 的输入输出内存。
    Camera_close();
    Yolo_close();
    return result;
}
