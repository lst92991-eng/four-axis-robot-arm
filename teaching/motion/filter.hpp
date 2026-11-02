#pragma once
#include <opencv2/core.hpp>

namespace motion {

// 四状态=[肩角,肘角,肩角速度,肘角速度]。测量角已换成关节rad。
class Filter {
public:
    Filter(double l1, double l2, double accelerationNoise, double encoderSigma);
    void reset(const cv::Vec2d& angles);
    cv::Vec2d update(const cv::Vec2d& encoder, double dt,
                    const cv::Point2d* tcp, double age, double sigmaMm, double maximumAge);
    bool usedXY() const { return usedXY_; }
private:
    double l1_, l2_, accelerationNoise_, encoderSigma_;
    cv::Mat state_, covariance_;
    bool usedXY_ = false;
};

} // namespace motion
