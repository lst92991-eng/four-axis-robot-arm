#include "vision/capture.hpp"
#include "vision/yolo.hpp"
#include <opencv2/imgcodecs.hpp>
#include <csignal>
#include <filesystem>
#include <iostream>
#include <exception>

static volatile std::sig_atomic_t stopped = 0;

static void requestStop(int)
{
    stopped = 1;
}

int main(int argc, char **argv)
{
    using namespace scara;
    if (argc != 5) return 2;
    int result = 1;
    try
    {
        Vision_setup();
        cv::Mat labels = cv::imread(argv[3], cv::IMREAD_GRAYSCALE);
        std::string temporary = std::string(argv[4]) + ".tmp.jpg";
        std::signal(SIGINT, requestStop);
        std::signal(SIGTERM, requestStop);
        bool ready = labels.cols == AtlasWidth && labels.rows == YoloClasses * LabelHeight && Yolo_open(argv[1]) &&
                     Camera_open(argv[2]);

        //---------取帧、识别与绘制区---------
        while (ready && !stopped)
        {
            cv::Mat source, input, preview;
            YoloHead heads[YoloHeads];
            std::vector<Detection> pieces;

            if (!Camera_read(source)) break;
            if (!Vision_prepare(source, input)) break;
            if (!Yolo_run(input, heads)) break;
            Vision_decode(heads, pieces);
            Vision_draw(source, pieces, labels, preview);
            if (!cv::imwrite(temporary, preview)) break;
            std::filesystem::rename(std::filesystem::u8path(temporary), std::filesystem::u8path(argv[4]));
        }
        result = ready && stopped ? 0 : 1;
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << '\n';
    }
    Camera_close();
    Yolo_close();
    return result;
}
