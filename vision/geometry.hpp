// 交点用 (行,列) 索引。pixel 为原图像素，xy 为基座毫米。
#pragma once
#include "chess/board.hpp"
#include "yolo.hpp"

namespace scara
{
//---------棋盘尺寸与交点数据区---------
using chess::Rows;
using chess::Columns;
constexpr int Cells = Rows * Columns;
constexpr int ImageWidth = 1280, ImageHeight = 720;

// 同一印刷交点在相机中的位置和在机械臂基座平面中的位置成对保存。
struct BoardPoint { cv::Point2d pixel, xy; };
struct Geometry
{
    // 固定后的 90 个交点都按 cell[行][列] 查找。
    BoardPoint cell[Rows][Columns]{};
    BoardPoint origin, rowStep, columnStep; // 三点确定的原点、每行位移、每列位移。
    double assignmentTolerance = .35;      // 允许棋子中心偏离交点的格数。
    // 两个平面均能确定行列方向且整张表填好后，ready 才置为 true。
    bool ready = false;
};

//---------标定、归格与预览接口区---------
// 按红左、红右、黑左传入三个实测交点，输出整张坐标表。
void Geometry_calibrate(BoardPoint redLeft, BoardPoint redRight, BoardPoint blackLeft, Geometry &board);
// 把同一平面上的位移换算为行、列，保持小数，供归格与盘外位置核对。
void Geometry_rowCol(cv::Point2d offset, cv::Point2d rowStep, cv::Point2d columnStep, double &row, double &col);
// 把本帧的检测框归到交点，seen.valid 在此表示这一帧的归格结果可用。
void Geometry_assign(const Geometry &board, const std::vector<Detection> &pieces, chess::Observation &seen,
                     bool visible = true);
// 在 source 的副本上绘制像素交点和行列号，结果写入 image。
void Geometry_draw(const cv::Mat &source, const Geometry &board, cv::Mat &image);
} // namespace scara
