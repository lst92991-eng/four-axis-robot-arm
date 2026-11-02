#pragma once
#include <array>
#include <vector>

namespace scara {

// 状态为棋盘平面→相机图像的单应矩阵前八项，最后一项固定为1。
// 棋盘和图像坐标各自除以宽高，约在[0,1]内；不是毫米或原图像素。
// 本滤波器稳定视觉标定，不估计电机角度，也不构成电机闭环。
struct HomographyEkf {
    std::array<double, 8> state{};
    std::array<double, 64> covariance{};  // 8×8，按行排列。
    bool initialized = false;
};

struct HomographyObservation {
    double x, y;  // 已知棋盘点的归一化坐标。
    double u, v;  // 相机检测得到的归一化图像坐标。
};

// 计算非线性投影及可选的2×8解析Jacobian（先u行、后v行）。
bool projectHomography(const std::array<double, 8>& state, double x, double y,
                       std::array<double, 2>& image,
                       std::array<double, 16>* jacobian = nullptr);

// 初始化亦用于重新定位后的重置；失败不修改原滤波器。
// 完全清空可直接赋值 filter = HomographyEkf{}。
bool initializeHomography(HomographyEkf& filter, const std::array<double, 9>& matrix,
                          double initialVariance);
// 随机游走：状态均值不动，P增加processVariance*dt*I。方差按秒给出。
bool predictHomography(HomographyEkf& filter, double dt, double processVariance);
// 测量噪声为measurementVariance*I；门限为二维创新的Mahalanobis距离平方。
// 9.21对应二维高斯假设下约99%的门限，不保证能识别所有错误标记。
// 拒绝非有限输入、投影退化和离群测量；拒绝时不修改状态或协方差。
bool updateHomography(HomographyEkf& filter, const HomographyObservation& point,
                      double measurementVariance, double gateSquared = 9.21);
// 顺序融合各独立角点，返回接受数量；空观测不修改滤波器。
int updateHomography(HomographyEkf& filter, const std::vector<HomographyObservation>& points,
                     double measurementVariance, double gateSquared = 9.21);
// 输出按行排列的3×3矩阵；尚未初始化时返回false，不输出伪造矩阵。
bool homographyMatrix(const HomographyEkf& filter, std::array<double, 9>& matrix);

} // namespace scara
