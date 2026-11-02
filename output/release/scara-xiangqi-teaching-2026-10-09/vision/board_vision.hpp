#pragma once
#include "detector.hpp"
#include "algorithm/homography_ekf.hpp"
#include "chess/board.hpp"
#include <opencv2/objdetect/aruco_detector.hpp>

// 定位和盘面整理独立于NPU模型生命周期；EKF只估计棋盘映射。
class BoardVision {
public:
    BoardVision();
    bool locate(const cv::Mat& bgr, cv::Mat& rgbForYolo, std::string& error);
    chess::Observation interpret(const std::vector<Detection>& pieces);
    void reset();
    const cv::Mat& view() const { return annotated_; }
    const cv::Mat& boardImage() const { return rectified_; }
    int markersFound() const { return markersFound_; }
private:
    cv::aruco::ArucoDetector detector_;
    scara::HomographyEkf filter_;
    cv::Mat annotated_, rectified_;
    chess::Board previous_{};
    int sameFrames_ = 0, markersFound_ = 0;
    double lastTime_ = 0;
    bool located_ = false;
};
