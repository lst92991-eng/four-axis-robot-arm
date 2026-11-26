// 交点用 (行,列) 索引。pixel 为原图像素，xy 为基座毫米。
#pragma once
#include "chess/board.hpp"
#include "capture.hpp"
#include "yolo.hpp"

namespace scara
{
//---------棋盘尺寸与交点数据区---------
using chess::Rows;
using chess::Columns;
constexpr int Cells = Rows * Columns;

// 同一印刷交点在相机中的位置和在机械臂基座平面中的位置成对保存。
struct BoardPoint { cv::Point2d pixel, xy; };
struct Geometry
{
    // 固定后的 90 个交点都按 cell[行][列] 查找。
    BoardPoint cell[Rows][Columns]{};
    BoardPoint origin, rowStep, columnStep; // 三点确定的原点、每行位移、每列位移。
    double assignmentTolerance = 0.35;     // 允许棋子中心偏离交点的格数。
};
// 一帧能识别出的盘面；发生重叠或偏离交点时仍可显示，暂不用于确认走法。
struct BoardObservation
{
    chess::Board cells;
    bool valid;
};

//---------标定、归格与预览接口区---------
// 课堂固定选红左、红右、黑左三个不共线交点，输出整张坐标表。
Geometry Geometry_calibrate(BoardPoint redLeft, BoardPoint redRight, BoardPoint blackLeft);
// 将平面位移换成棋盘坐标；点的 x 为列，y 为行，保留小数。
cv::Point2d Geometry_gridPoint(cv::Point2d offset, cv::Point2d rowStep, cv::Point2d columnStep);
BoardObservation Geometry_assign(const Geometry &board, const std::vector<Detection> &pieces, bool visible = true);
// 返回带交点和行列号的原图副本。
cv::Mat Geometry_draw(const cv::Mat &source, const Geometry &board);
} // namespace scara
