#include "vision/capture.hpp"
#include "vision/yolo.hpp"
#include <opencv2/imgcodecs.hpp>
#include <csignal>
#include <filesystem>
#include <iostream>
#include <stdexcept>

static volatile std::sig_atomic_t running = 1;

static void requestStop(int)
{
    running = 0;
    scara::Camera_stop();
}

// 完整写入临时图后，再替换给预览端读取的文件。
static void savePreview(const cv::Mat &image, const std::string &temporary, const char *output)
{
    if (!cv::imwrite(temporary, image)) throw std::runtime_error("Cannot write preview");
    std::filesystem::rename(std::filesystem::u8path(temporary), std::filesystem::u8path(output));
}

int main(int argc, char **argv)
{
    using namespace scara;
    if (argc != 5) return 2;           // 程序名之外，需要下面四个路径/地址参数。
    const char *modelPath = argv[1];   // 已训练好的 RKNN 模型。
    const char *cameraUrl = argv[2];   // 摄像头 RTSP 视频流地址。
    const char *labelPath = argv[3];   // 用于显示中文名称的固定字图。
    const char *previewPath = argv[4]; // 保存识别画面的 JPEG 文件。
    int result = 1;
    try
    {
        Vision_setup();
        cv::Mat labels = cv::imread(labelPath, cv::IMREAD_GRAYSCALE);
        std::string temporary = std::string(previewPath) + ".tmp.jpg";
        std::signal(SIGINT, requestStop);
        std::signal(SIGTERM, requestStop);
        result = Yolo_open(modelPath) ? 0 : 1;
        running = running && result == 0;
        Camera_open(cameraUrl);

        //---------取帧、识别与绘制区---------
        while (running)
        {
            cv::Mat source;                // 摄像头解码得到的 BGR 原图。
            cv::Mat input;                 // 裁剪、转 RGB、缩放补边后的 640×640 图。
            cv::Mat preview;               // 画好框、中文标签和分数的原尺寸预览图。
            YoloHead heads[YoloHeads];     // NPU 三路输出的地址和格式描述。
            std::vector<Detection> pieces; // 检测到的棋子：类别、分数、原图位置框。

            Camera_read(source);
            Vision_prepare(source, input);
            Yolo_run(input, heads);
            Vision_decode(heads, pieces);
            Vision_draw(source, pieces, labels, preview);
            savePreview(preview, temporary, previewPath);
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
    Camera_close();
    Yolo_close();
    return result;
}
