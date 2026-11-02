#pragma once
#include "detector.hpp"
#include "chess/board.hpp"
#include <opencv2/objdetect/aruco_detector.hpp>
#include <array>

// 长度统一为mm。标定数值由同一份应用配置传入，不从旧实验常量读取。
struct VisionConfig {
    double boardWidthMm = 0, boardHeightMm = 0;
    double gridXmm = 0, gridYmm = 0, pitchMm = 0, markerSideMm = 0;
    std::array<cv::Point2f, 4> markerCentersMm{}; // ID 0/1/2/3，左上起顺时针。
    double pixelsPerMm = 2;
    int stableFrames = 3;
    float confidence = .5f;
    bool boardCalibrated = false;
    cv::Matx33d boardToBase = cv::Matx33d::eye();
    std::array<cv::Point2f, 4> referenceMarkerPixels{};
    double boardPoseTolerancePixels = 5;
    int tcpMarkerId = -1;
    bool tcpCalibrated = false;
    cv::Matx33d tcpImageToBase = cv::Matx33d::eye();
    double tcpPlaneZmm = 0, tcpPlaneToleranceMm = .5;
    cv::Point2d tcpMarkerOffsetMm{}; // 标记局部X/Y到TCP的偏移，随标记朝向旋转。
    double maxObservationAgeSeconds = .1;
};

struct TcpObservation {
    cv::Point2d xy{};
    double stamp = 0; // steady_clock采集时刻或经实测延迟推得的保守下界；不能直接取FIFO读完时刻。
    bool valid = false;
};

class BoardVision {
public:
    explicit BoardVision(const VisionConfig& config);
    bool locate(const cv::Mat& bgr, cv::Mat& rgbForYolo, std::string& error);
    chess::Observation interpret(const std::vector<Detection>& pieces);
    cv::Point2d squareXY(int row, int col) const;
    // 单平面标定仅在该Z范围有效；其他高度返回无效观测，不套用棋盘单应矩阵。
    TcpObservation observeTcp(const cv::Mat& bgr, double measuredAt, double actualZmm);
    void reset();
    const cv::Mat& view() const { return annotated_; }
    const cv::Mat& boardImage() const { return rectified_; }
    int markersFound() const { return markersFound_; }
private:
    VisionConfig config_;
    cv::aruco::ArucoDetector detector_;
    cv::Mat annotated_, rectified_;
    cv::Rect area_;
    chess::Board previous_{};
    int sameFrames_ = 0, markersFound_ = 0;
    bool located_ = false;
};
