#include "homography_ekf.hpp"
#include <cmath>

namespace scara {
namespace {

bool validTransform(const std::array<double, 8>& h)
{
    for (double value : h) {
        if (!std::isfinite(value)) return false;
    }
    // 归一化棋盘四角的投影分母须同为正，防止无穷远线穿过棋盘。
    if (1 + h[6] <= 1e-8 || 1 + h[7] <= 1e-8 || 1 + h[6] + h[7] <= 1e-8)
        return false;
    const double determinant = h[0] * (h[4] - h[5] * h[7])
        - h[1] * (h[3] - h[5] * h[6]) + h[2] * (h[3] * h[7] - h[4] * h[6]);
    return std::isfinite(determinant) && std::abs(determinant) > 1e-12;
}

} // namespace

bool projectHomography(const std::array<double, 8>& h, double x, double y,
                       std::array<double, 2>& image, std::array<double, 16>* jacobian)
{
    if (!std::isfinite(x) || !std::isfinite(y)) return false;
    for (double value : h) {
        if (!std::isfinite(value)) return false;
    }
    const double denominator = h[6] * x + h[7] * y + 1;
    if (!std::isfinite(denominator) || std::abs(denominator) < 1e-8) return false;
    const double u = (h[0] * x + h[1] * y + h[2]) / denominator;
    const double v = (h[3] * x + h[4] * y + h[5]) / denominator;
    if (!std::isfinite(u) || !std::isfinite(v)) return false;
    const std::array<double, 16> derivative = {
        x / denominator, y / denominator, 1 / denominator, 0, 0, 0,
        -u * x / denominator, -u * y / denominator,
        0, 0, 0, x / denominator, y / denominator, 1 / denominator,
        -v * x / denominator, -v * y / denominator
    };
    for (double value : derivative) {
        if (!std::isfinite(value)) return false;
    }
    image = {u, v};
    if (jacobian) *jacobian = derivative;
    return true;
}

bool initializeHomography(HomographyEkf& filter, const std::array<double, 9>& matrix,
                          double initialVariance)
{
    if (!std::isfinite(initialVariance) || initialVariance <= 0 ||
        !std::isfinite(matrix[8]) || std::abs(matrix[8]) < 1e-12) return false;
    HomographyEkf initial;
    for (int i = 0; i < 8; ++i) {
        initial.state[i] = matrix[i] / matrix[8];
        initial.covariance[i * 8 + i] = initialVariance;
    }
    if (!validTransform(initial.state)) return false;
    initial.initialized = true;
    filter = initial;
    return true;
}

bool predictHomography(HomographyEkf& filter, double dt, double processVariance)
{
    if (!filter.initialized || !std::isfinite(dt) || dt < 0 ||
        !std::isfinite(processVariance) || processVariance < 0) return false;
    auto covariance = filter.covariance;
    for (int i = 0; i < 8; ++i) covariance[i * 8 + i] += processVariance * dt;
    for (double value : covariance) {
        if (!std::isfinite(value)) return false;
    }
    filter.covariance = covariance;
    return true;
}

bool updateHomography(HomographyEkf& filter, const HomographyObservation& point,
                      double measurementVariance, double gateSquared)
{
    if (!filter.initialized || !std::isfinite(point.u) || !std::isfinite(point.v) ||
        !std::isfinite(measurementVariance) || measurementVariance <= 0 ||
        !std::isfinite(gateSquared) || gateSquared <= 0) return false;
    std::array<double, 2> predicted;
    std::array<double, 16> jacobian;
    if (!projectHomography(filter.state, point.x, point.y, predicted, &jacobian)) return false;

    // P*J转置为8×2；S=J*P*J转置+R只需求一个2×2逆矩阵。
    std::array<double, 16> cross{};
    for (int row = 0; row < 8; ++row) {
        for (int column = 0; column < 2; ++column) {
            for (int k = 0; k < 8; ++k)
                cross[row * 2 + column] += filter.covariance[row * 8 + k] * jacobian[column * 8 + k];
        }
    }
    double s00 = measurementVariance, s01 = 0, s11 = measurementVariance;
    for (int k = 0; k < 8; ++k) {
        s00 += jacobian[k] * cross[k * 2];
        s01 += jacobian[k] * cross[k * 2 + 1];
        s11 += jacobian[8 + k] * cross[k * 2 + 1];
    }
    const double determinant = s00 * s11 - s01 * s01;
    if (!std::isfinite(determinant) || s00 <= 0 || s11 <= 0 ||
        determinant <= 1e-12 * s00 * s11) return false;
    const double innovationU = point.u - predicted[0];
    const double innovationV = point.v - predicted[1];
    const double distance = (s11 * innovationU * innovationU -
        2 * s01 * innovationU * innovationV + s00 * innovationV * innovationV) / determinant;
    if (!std::isfinite(distance) || distance < 0 || distance > gateSquared) return false;

    std::array<double, 16> gain{};
    auto state = filter.state;
    std::array<double, 64> residual{}, intermediate{}, covariance{};
    for (int row = 0; row < 8; ++row) {
        gain[row * 2] = (cross[row * 2] * s11 - cross[row * 2 + 1] * s01) / determinant;
        gain[row * 2 + 1] = (cross[row * 2 + 1] * s00 - cross[row * 2] * s01) / determinant;
        state[row] += gain[row * 2] * innovationU + gain[row * 2 + 1] * innovationV;
        for (int column = 0; column < 8; ++column) {
            residual[row * 8 + column] = (row == column ? 1.0 : 0.0)
                - gain[row * 2] * jacobian[column] - gain[row * 2 + 1] * jacobian[8 + column];
        }
    }
    if (!validTransform(state)) return false;

    // Joseph形式：(I-KJ)P(I-KJ)转置+KRK转置，比直接相减更不易破坏半正定性。
    for (int row = 0; row < 8; ++row) {
        for (int column = 0; column < 8; ++column) {
            for (int k = 0; k < 8; ++k)
                intermediate[row * 8 + column] += residual[row * 8 + k] * filter.covariance[k * 8 + column];
        }
    }
    for (int row = 0; row < 8; ++row) {
        for (int column = 0; column < 8; ++column) {
            for (int k = 0; k < 8; ++k)
                covariance[row * 8 + column] += intermediate[row * 8 + k] * residual[column * 8 + k];
            covariance[row * 8 + column] += measurementVariance *
                (gain[row * 2] * gain[column * 2] + gain[row * 2 + 1] * gain[column * 2 + 1]);
        }
    }
    for (int row = 0; row < 8; ++row) {
        for (int column = row; column < 8; ++column) {
            const double value = (covariance[row * 8 + column] + covariance[column * 8 + row]) * 0.5;
            if (!std::isfinite(value) || (row == column && value < 0)) return false;
            covariance[row * 8 + column] = covariance[column * 8 + row] = value;
        }
    }
    filter.state = state;
    filter.covariance = covariance;
    return true;
}

int updateHomography(HomographyEkf& filter, const std::vector<HomographyObservation>& points,
                     double measurementVariance, double gateSquared)
{
    int accepted = 0;
    for (const auto& point : points) {
        if (updateHomography(filter, point, measurementVariance, gateSquared)) ++accepted;
    }
    return accepted;
}

bool homographyMatrix(const HomographyEkf& filter, std::array<double, 9>& matrix)
{
    if (!filter.initialized || !validTransform(filter.state)) return false;
    for (int i = 0; i < 8; ++i) matrix[i] = filter.state[i];
    matrix[8] = 1;
    return true;
}

} // namespace scara
