#include "motion_filter.hpp"
#include <cmath>
#include <stdexcept>

void MotionFilter::reset(cv::Vec2d angles) {
    state = cv::Mat(cv::Vec4d(angles[0], angles[1], 0, 0));
    covariance = cv::Mat::eye(4, 4, CV_64F) * .001;
}
cv::Vec2d MotionFilter::step(cv::Vec2d encoder, double dt, const cv::Point2d *tcp, double age,
                             double sigma) {
    if (state.empty() || !std::isfinite(dt) || dt <= 0 || dt > .2 || !std::isfinite(encoder[0]) ||
        !std::isfinite(encoder[1]))
        throw std::runtime_error("运动滤波未初始化/采样失效");
    cv::Mat F = cv::Mat::eye(4, 4, CV_64F), G = cv::Mat::zeros(4, 2, CV_64F);
    F.at<double>(0, 2) = F.at<double>(1, 3) = dt;
    G.at<double>(0, 0) = G.at<double>(1, 1) = dt * dt / 2;
    G.at<double>(2, 0) = G.at<double>(3, 1) = dt;
    state = F * state;
    covariance = F * covariance * F.t() + G * G.t() * 225; // 加速度噪声15rad/s²为待辨识假设。
    double a = state.at<double>(0), b = state.at<double>(1), ab = a + b;
    cv::Mat H(cv::Matx<double, 2, 4>(1, 0, 0, 0, 0, 1, 0, 0));
    cv::Mat residual(cv::Vec2d(encoder[0] - a, encoder[1] - b));
    cv::Mat R = cv::Mat::eye(2, 2, CV_64F) * .004 * .004;
    usedXY = false;
    if (tcp && age >= 0 && age <= .1 && std::isfinite(tcp->x) && std::isfinite(tcp->y) &&
        std::isfinite(sigma) && sigma > 0) {
        cv::Mat J(cv::Matx<double, 2, 4>(-l1 * std::sin(a) - l2 * std::sin(ab), -l2 * std::sin(ab),
                                         0, 0, l1 * std::cos(a) + l2 * std::cos(ab),
                                         l2 * std::cos(ab), 0, 0));
        cv::Mat delta(cv::Vec2d(tcp->x - l1 * std::cos(a) - l2 * std::cos(ab),
                                tcp->y - l1 * std::sin(a) - l2 * std::sin(ab)));
        cv::Mat xyR = cv::Mat::eye(2, 2, CV_64F) * std::max(.01, sigma * sigma), scaled;
        bool valid = cv::solve(J * covariance * J.t() + xyR, delta, scaled, cv::DECOMP_CHOLESKY);
        // 归一化残差门限16是当前配置假设；拒绝大跳点，不凭错定位修正电机。
        if (valid && delta.dot(scaled) <= 16) {
            cv::vconcat(H, J, H);
            cv::vconcat(residual, delta, residual);
            cv::Mat full = cv::Mat::zeros(4, 4, CV_64F);
            R.copyTo(full(cv::Rect(0, 0, 2, 2)));
            xyR.copyTo(full(cv::Rect(2, 2, 2, 2)));
            R = full;
            usedXY = true;
        }
    }
    cv::Mat solved;
    if (!cv::solve(H * covariance * H.t() + R, H * covariance, solved, cv::DECOMP_CHOLESKY))
        throw std::runtime_error("运动滤波协方差求解失败");
    cv::Mat K = solved.t(), I = cv::Mat::eye(4, 4, CV_64F) - K * H;
    state += K * residual;
    covariance = I * covariance * I.t() + K * R * K.t(); // Joseph形式，保持数值稳定。
    return {state.at<double>(0), state.at<double>(1)};
}
