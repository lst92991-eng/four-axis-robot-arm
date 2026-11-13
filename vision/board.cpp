#include "geometry.hpp"
#include <opencv2/geometry.hpp>
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace scara
{
namespace
{
constexpr int RectWidth = 432, RectHeight = 480, Step = 48, Margin = 24;
void require(bool value, const char *message)
{
    if (!value) throw std::runtime_error(message);
}
} // namespace

void boardFromCorners(const std::array<cv::Point2d, 4> &raw, bool confirmed, Board &board)
{
    std::vector<cv::Point2f> pixels;                                  // 操作员确认的原图四角。
    std::vector<cv::Point2f> corners{{0, 0}, {8, 0}, {8, 9}, {0, 9}}; // 对应的棋盘列、行。
    cv::Mat matrix;
    cv::Point2d cell;
    Board prepared;

    for (cv::Point2d point : raw)
    {
        require(std::isfinite(point.x) && std::isfinite(point.y) && point.x >= 0 && point.y >= 0 &&
                    point.x < ImageWidth && point.y < ImageHeight,
                "Invalid raw corner");
        pixels.push_back(point);
    }
    require(cv::isContourConvex(pixels) && std::abs(cv::contourArea(pixels)) > 100,
            "Corners must form a nondegenerate ordered convex quad");
    matrix = cv::getPerspectiveTransform(corners, pixels);
    std::copy(matrix.ptr<double>(), matrix.ptr<double>() + 9, prepared.gridToPixel.val);
    require(std::abs(cv::determinant(matrix)) > 1e-14, "Degenerate board quadrilateral");
    for (int row = 0; row < Rows; ++row)
        for (int col = 0; col < Columns; ++col)
        {
            cell.x = col;
            cell.y = row;
            project(prepared.gridToPixel, cell, prepared.intersections[row * Columns + col]);
        }
    prepared.geometryValid = true;
    prepared.orientationValid = confirmed;
    prepared.orientationSource = confirmed ? "manual_canonical_corners" : "unconfirmed_corners";
    board = prepared;
}
void rectifiedBoard(const cv::Mat &image, const Board &board, cv::Mat &result)
{
    // 俯视预览把红方放在下边；矩阵第二行反转纵轴，不镜像棋字。
    cv::Matx33d gridToRect(Step, 0, Margin, 0, -Step, Margin + 9 * Step, 0, 0, 1);
    cv::Matx33d pixelToRect;
    cv::Size outputSize(RectWidth, RectHeight);

    require(board.geometryValid, "Grid required for rectification");
    pixelToRect = gridToRect * board.gridToPixel.inv();
    cv::warpPerspective(image, result, pixelToRect, outputSize);
}
} // namespace scara
