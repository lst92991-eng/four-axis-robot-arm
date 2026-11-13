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

void boardFromCorners(const std::string &frame, const std::array<cv::Point2d, 4> &raw, bool confirmed, Board &board)
{
    require(!frame.empty(), "Frame ID required");
    std::vector<cv::Point2f> p;
    for (auto v : raw)
    {
        require(std::isfinite(v.x) && std::isfinite(v.y) && v.x >= 0 && v.y >= 0 && v.x < ImageWidth &&
                    v.y < ImageHeight,
                "Invalid raw corner");
        p.push_back(v);
    }
    require(cv::isContourConvex(p) && std::abs(cv::contourArea(p)) > 100,
            "Corners must form a nondegenerate ordered convex quad");
    cv::Mat matrix = cv::getPerspectiveTransform(std::vector<cv::Point2f>{{0, 0}, {8, 0}, {8, 9}, {0, 9}}, p);
    Board b;
    std::copy(matrix.ptr<double>(), matrix.ptr<double>() + 9, b.gridToPixel.val);
    require(std::abs(cv::determinant(matrix)) > 1e-14, "Degenerate board quadrilateral");
    b.frameId = frame;
    for (int row = 0; row < Rows; ++row)
        for (int col = 0; col < Columns; ++col)
            b.intersections[row * Columns + col] = project(b.gridToPixel, {double(col), double(row)});
    b.geometryValid = true;
    b.orientationValid = confirmed;
    b.orientationSource = confirmed ? "manual_canonical_corners" : "unconfirmed_corners";
    board = b;
}
void rectifiedBoard(const cv::Mat &image, const Board &b, cv::Mat &result)
{
    require(b.geometryValid, "Grid required for rectification");
    // Human-facing rectification keeps red at the bottom, without mirroring Chinese glyphs.
    cv::Matx33d gridToRect(Step, 0, Margin, 0, -Step, Margin + 9 * Step, 0, 0, 1);
    cv::warpPerspective(image, result, gridToRect * b.gridToPixel.inv(), {RectWidth, RectHeight});
}
} // namespace scara
