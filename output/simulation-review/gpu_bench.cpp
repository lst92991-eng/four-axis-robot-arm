#include "detector.hpp"
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <chrono>
#include <iostream>
#ifdef BENCH_CPU
Detector *detector_load(const char *, std::string &);
#endif
int main(int argc, char **argv) {
    cv::setNumThreads(4);
    std::string error;
#ifdef BENCH_CPU
    auto *detector = detector_load("models/xiangqi14-s.onnx", error);
#else
    auto *detector = detector_load(error);
#endif
    if (!detector) { std::cerr << error; return 1; }
    cv::Mat image = cv::imread(argv[1]), rgb;
    cv::cvtColor(image(cv::Rect(130,130,540,600)), rgb, cv::COLOR_BGR2RGB);
    std::vector<Detection> boxes;
    cv::FileStorage output(argv[2], cv::FileStorage::WRITE | cv::FileStorage::FORMAT_JSON);
    output << "opencv" << CV_VERSION << "ms" << "[";
    for (int i=0; i<21; ++i) {
        auto start = std::chrono::steady_clock::now();
        if (!detector_detect(detector,rgb,boxes,error)) { std::cerr << error; return 2; }
        double ms = std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();
        if (i) output << ms;
    }
    output << "]" << "boxes" << "[";
    for (auto &b:boxes) output << "[" << b.classId << b.confidence << b.x1 << b.y1 << b.x2 << b.y2 << "]";
    output << "]";
#ifndef BENCH_CPU
    output << "backend" << detector_backend(detector);
#endif
    detector_close(detector);
}
