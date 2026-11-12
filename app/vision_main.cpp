#include "vision/capture.hpp"
#include "vision/yolo.hpp"
#include <opencv2/imgcodecs.hpp>
#include <csignal>
#include <filesystem>
#include <iostream>
#include <stdexcept>

namespace {
volatile std::sig_atomic_t stopped = 0;
void stop(int) { stopped = 1; }
}

int main(int argc, char **argv)
{
  if (argc<5 || argc>6)
  {
    std::cerr << "Usage: scara_vision model.rknn camera-url labels.png preview.jpg [frame-count]\n";
    return 2;
  }
  int result = 0;
  try
  {
    scara::VisionConfig config;
    cv::Mat labels = cv::imread(argv[3], cv::IMREAD_GRAYSCALE);
    int count = 0, limit = argc==6 ? std::stoi(argv[5]) : 0;
    if (limit<0) throw std::runtime_error("Frame count must be nonnegative");
    std::signal(SIGINT, stop);
    std::signal(SIGTERM, stop);
    cv::setNumThreads(1);
    scara::Yolo_open(argv[1]);
    scara::Camera_open(argv[2]);

    //---------取帧、识别与绘制区---------
    while (!stopped && (!limit || count<limit))
    {
      cv::Mat source = scara::Camera_read();
      scara::ImageMapping mapping;
      cv::Mat input = scara::Vision_prepare(source, config, mapping);
      auto heads = scara::Yolo_run(input);
      auto pieces = scara::Vision_decode(heads, mapping, config);
      cv::Mat preview = scara::Vision_draw(source, pieces, labels);
      std::string temporary = std::string(argv[4])+".tmp.jpg";
      if (!cv::imwrite(temporary, preview)) throw std::runtime_error("Cannot write preview");
      std::filesystem::rename(std::filesystem::u8path(temporary), std::filesystem::u8path(argv[4]));
      std::cout << "frame=" << ++count << " pieces=" << pieces.size() << '\n';
    }
  }
  catch (const std::exception& e)
  {
    std::cerr << e.what() << '\n';
    result = 1;
  }
  scara::Camera_close();
  scara::Yolo_close();
  return result;
}
