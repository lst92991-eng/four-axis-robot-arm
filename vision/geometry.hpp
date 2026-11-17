//固定棋盘的初始化、识别归格、目标坐标和检查图
#pragma once
#include "yolo.hpp"
#include <opencv2/core.hpp>
#include <array>
#include <string>

namespace scara
{
constexpr int Columns = 9, Rows = 10, Cells = Columns * Rows;
constexpr int ImageWidth = 1280, ImageHeight = 720;
//类别0～13对应黑方小写、红方大写；另外保留空格与未知格
constexpr char PieceSymbols[] = "kabnrcpKABNRCP";
constexpr int Empty = -1, Unknown = -2;
struct Observation
{
    //帧编号、90格类别和连续帧确认结果
    std::string frameId;
    std::array<int, Cells> pieces{};
    bool valid = false;
};
struct Geometry
{
    //像素到基座毫米、像素到逻辑行列的六系数矩阵
    cv::Matx23d pixelToBase{}, pixelToGrid{};
    //90个交点的原图像素及基座XY，下标均为行号乘9加列号
    std::array<cv::Point2d, Cells> pixels{}, targets{};
    //上一张有效观察及连续相同盘面的帧数
    Observation previous;
    int stableFrames = 0;
    //初始化完成后，90个交点可供识别与走棋使用
    bool ready = false;
};
//初始化读一次配置，之后识别和走棋只访问内存
void Geometry_init(const cv::FileNode &config, Geometry &geometry);
void Geometry_observe(Geometry &geometry, const std::string &frameId, const std::vector<Detection> &pieces,
                      bool sceneClear, Observation &result);
bool Geometry_target(const Geometry &geometry, int column, int row, cv::Point2d &baseMm);
void Geometry_draw(const cv::Mat &source, const Geometry &geometry, cv::Mat &image);
} // namespace scara
