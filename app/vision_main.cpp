#include "vision/capture.hpp"
#include "vision/yolo.hpp"
#include <opencv2/imgcodecs.hpp>
#include <csignal>
#include <filesystem>
#include <iostream>
#include <stdexcept>

static volatile std::sig_atomic_t stopped = 0;

static void requestStop(int)
{
    stopped = 1;
}

int main(int argc, char **argv)
{
    if (argc != 5) return 2;
    int result = 1;
    try
    {
        scara::Vision_setup();
        cv::Mat labels = cv::imread(argv[3], cv::IMREAD_GRAYSCALE);
        if (labels.cols != scara::AtlasWidth || labels.rows != scara::YoloClasses * scara::LabelHeight)
            throw std::runtime_error("Cannot read the 14-class label image");
        std::string temporary = std::string(argv[4]) + ".tmp.jpg";
        std::signal(SIGINT, requestStop);
        std::signal(SIGTERM, requestStop);
        if (!scara::Yolo_open(argv[1]) || !scara::Camera_open(argv[2]))
            throw std::runtime_error("Cannot start model or camera");

        //---------取帧、识别与绘制区---------
        while (!stopped)
        {
            cv::Mat source = scara::Camera_read();
            if (source.empty()) break;
            cv::Mat input = scara::Vision_prepare(source);
            const scara::YoloHead *heads = scara::Yolo_run(input);
            if (!heads) break;
            std::vector<scara::Detection> pieces = scara::Vision_decode(heads);
            cv::Mat preview = scara::Vision_draw(source, pieces, labels);
            if (!cv::imwrite(temporary, preview)) throw std::runtime_error("Cannot write preview");
            std::filesystem::rename(std::filesystem::u8path(temporary), std::filesystem::u8path(argv[4]));
        }
        result = stopped ? 0 : 1;
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << '\n';
    }
    scara::Camera_close();
    scara::Yolo_close();
    return result;
}
