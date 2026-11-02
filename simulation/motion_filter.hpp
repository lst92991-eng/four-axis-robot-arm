#pragma once
#include <opencv2/core.hpp>

// 电机侧关节角＋可选独立末端XY。只有编码器时是KF，有新鲜XY时才是EKF。
class MotionFilter {
    double l1, l2;
    cv::Mat state, covariance;

public:
    bool usedXY = false;
    MotionFilter(double firstArmMm, double secondArmMm) : l1(firstArmMm), l2(secondArmMm) {}
    void reset(cv::Vec2d angles);
    cv::Vec2d step(cv::Vec2d encoderAngles, double dt, const cv::Point2d *tcp = nullptr,
                   double tcpAgeSeconds = 0, double tcpSigmaMm = .2);
};
