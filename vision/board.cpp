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
cv::Matx33d homography(const std::vector<cv::Point2f> &a, const std::vector<cv::Point2f> &b)
{
    cv::Mat m = cv::getPerspectiveTransform(a, b);
    cv::Matx33d h;
    std::copy(m.ptr<double>(), m.ptr<double>() + 9, h.val);
    require(std::abs(cv::determinant(h)) > 1e-14, "Degenerate board quadrilateral");
    return h;
}
Board boardWithMatrix(const Camera &camera, const std::string &frame, const cv::Matx33d &h)
{
    require(!frame.empty(), "Frame ID required");
    Board b;
    b.cameraId = camera.id;
    b.geometryCamera = camera;
    b.frameId = frame;
    b.gridToPixel = h;
    for (int r = 0; r < Rows; ++r)
        for (int c = 0; c < Columns; ++c)
        {
            auto p = project(h, {(double)c, (double)r});
            require(p.x >= 0 && p.y >= 0 && p.x < camera.imageSize.width && p.y < camera.imageSize.height,
                    "Grid extends outside image");
            b.intersections[r * Columns + c] = p;
        }
    b.geometryValid = true;
    return b;
}
} // namespace

void boardFromCorners(const Camera &camera, const std::string &frame, const std::array<cv::Point2d, 4> &raw,
                      bool confirmed, Board &board)
{
    camera.validate();
    std::vector<cv::Point2f> p;
    for (auto v : raw)
    {
        require(std::isfinite(v.x) && std::isfinite(v.y) && v.x >= 0 && v.y >= 0 && v.x < camera.imageSize.width &&
                    v.y < camera.imageSize.height,
                "Invalid raw corner");
        p.push_back(camera.geometricPixel(v));
    }
    require(cv::isContourConvex(p) && std::abs(cv::contourArea(p)) > 100,
            "Corners must form a nondegenerate ordered convex quad");
    auto b = boardWithMatrix(camera, frame, homography({{0, 0}, {8, 0}, {8, 9}, {0, 9}}, p));
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
