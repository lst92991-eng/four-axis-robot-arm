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
    if (argc != 5)
    {
        std::cerr << "Usage: scara_vision model.rknn camera-url labels.png preview.jpg\n";
        return 2;
    }
    int result = 0;
    try
    {
        scara::VisionConfig config;
        cv::Mat labels = cv::imread(argv[3], cv::IMREAD_GRAYSCALE);
        if (labels.cols != scara::AtlasWidth || labels.rows != scara::YoloClasses * scara::LabelHeight)
            throw std::runtime_error("Cannot read the 14-class label image");
        std::string temporary = std::string(argv[4]) + ".tmp.jpg";
        std::signal(SIGINT, requestStop);
        std::signal(SIGTERM, requestStop);
        cv::setNumThreads(1);
        scara::Yolo_open(argv[1]);
        scara::Camera_open(argv[2]);

        //---------取帧、识别与绘制区---------
        while (!stopped)
        {
            cv::Mat source = scara::Camera_read();
            scara::ImageMapping mapping;
            cv::Mat input = scara::Vision_prepare(source, config, mapping);
            const scara::YoloHead *heads = scara::Yolo_run(input);
            std::vector<scara::Detection> pieces = scara::Vision_decode(heads, mapping, config);
            cv::Mat preview = scara::Vision_draw(source, pieces, labels);
            if (!cv::imwrite(temporary, preview)) throw std::runtime_error("Cannot write preview");
            std::filesystem::rename(std::filesystem::u8path(temporary), std::filesystem::u8path(argv[4]));
        }
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << '\n';
        result = 1;
    }
    scara::Camera_close();
    scara::Yolo_close();
    return result;
}
