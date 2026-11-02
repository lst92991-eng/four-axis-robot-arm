#include "filter.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace motion {

Filter::Filter(double l1, double l2, double accelerationNoise, double encoderSigma)
    : l1_(l1), l2_(l2), accelerationNoise_(accelerationNoise), encoderSigma_(encoderSigma) {}

void Filter::reset(const cv::Vec2d& angles)
{
    state_ = cv::Mat(cv::Vec4d(angles[0], angles[1], 0, 0));
    covariance_ = cv::Mat::eye(4, 4, CV_64F) * .001;
    usedXY_ = false;
}

cv::Vec2d Filter::update(const cv::Vec2d& encoder, double dt, const cv::Point2d* tcp,
                       double age, double sigmaMm, double maximumAge)
{
    if (state_.empty() || !std::isfinite(dt) || dt <= 0 || dt > .3 ||
        !std::isfinite(encoder[0]) || !std::isfinite(encoder[1]))
        throw std::runtime_error("运动估计未初始化或反馈时间失效");
    cv::Mat F = cv::Mat::eye(4, 4, CV_64F), G = cv::Mat::zeros(4, 2, CV_64F);
    F.at<double>(0, 2) = F.at<double>(1, 3) = dt;
    G.at<double>(0, 0) = G.at<double>(1, 1) = dt * dt / 2;
    G.at<double>(2, 0) = G.at<double>(3, 1) = dt;
    state_ = F * state_;
    covariance_ = F * covariance_ * F.t() + G * G.t() * accelerationNoise_ * accelerationNoise_;
    const double a = state_.at<double>(0), b = state_.at<double>(1), ab = a + b;
    cv::Mat H(cv::Matx<double, 2, 4>(1, 0, 0, 0, 0, 1, 0, 0));
    cv::Mat residual(cv::Vec2d(encoder[0] - a, encoder[1] - b));
    cv::Mat R = cv::Mat::eye(2, 2, CV_64F) * encoderSigma_ * encoderSigma_;
    usedXY_ = false;
    if (tcp && std::isfinite(age) && age >= 0 && age <= maximumAge &&
        std::isfinite(tcp->x) && std::isfinite(tcp->y) && std::isfinite(sigmaMm) && sigmaMm > 0) {
        cv::Mat J(cv::Matx<double, 2, 4>(-l1_ * std::sin(a) - l2_ * std::sin(ab), -l2_ * std::sin(ab), 0, 0,
                                       l1_ * std::cos(a) + l2_ * std::cos(ab), l2_ * std::cos(ab), 0, 0));
        cv::Mat delta(cv::Vec2d(tcp->x - l1_ * std::cos(a) - l2_ * std::cos(ab),
                                tcp->y - l1_ * std::sin(a) - l2_ * std::sin(ab)));
        cv::Mat xyR = cv::Mat::eye(2, 2, CV_64F) * sigmaMm * sigmaMm, solved;
        if (cv::solve(J * covariance_ * J.t() + xyR, delta, solved, cv::DECOMP_CHOLESKY) && delta.dot(solved) <= 16) {
            cv::vconcat(H, J, H); cv::vconcat(residual, delta, residual);
            cv::Mat full = cv::Mat::zeros(4, 4, CV_64F);
            R.copyTo(full(cv::Rect(0, 0, 2, 2))); xyR.copyTo(full(cv::Rect(2, 2, 2, 2)));
            R = full;
            usedXY_ = true;
        }
    }
    cv::Mat solved;
    if (!cv::solve(H * covariance_ * H.t() + R, H * covariance_, solved, cv::DECOMP_CHOLESKY))
        throw std::runtime_error("运动滤波协方差求解失败");
    cv::Mat K = solved.t(), I = cv::Mat::eye(4, 4, CV_64F) - K * H;
    state_ += K * residual;
    covariance_ = I * covariance_ * I.t() + K * R * K.t(); // Joseph更新保留协方差数值对称性。
    return {state_.at<double>(0), state_.at<double>(1)};
}

} // namespace motion
