#pragma once
#include "yolo.hpp"
#include <opencv2/core.hpp>
#include <array>
#include <string>
#include <vector>

// 固定机位使用已独立验误的原始图像像素；本模块不发送运动命令。
namespace scara
{
constexpr int Columns = 9, Rows = 10, Cells = Columns * Rows;
constexpr int Unknown = -2, Empty = -1; // 未确认的格子与已确认的空格分开。
constexpr char PieceSymbols[] = "kabnrcpKABNRCP";

constexpr int ImageWidth = 1280, ImageHeight = 720;
constexpr char CameraId[] = "sc3336-1280x720-fixed-focus-v1";

struct PointPair
{
    cv::Point2d pixel, baseXY;
}; // pixel为原始图像像素，baseXY为基座毫米坐标。
struct ErrorReport
{
    double rmsMm = 0, maxMm = 0;
    std::vector<double> errorsMm;
};
struct PlaneMap
{
    std::string id;
    double planeZmm = 0, allowedErrorMm = 0; // 标定平面说明与验误要求，不是J1驱动角度。
    cv::Matx33d pixelToBase = cv::Matx33d::eye();
    std::vector<cv::Point2f> validPixelHull; // 拟合点覆盖的几何像素范围。
    std::vector<PointPair> fitPoints, checkPoints;
    ErrorReport fitError, checkError;
    bool accepted = false;
};

// 拟合点和独立检查点使用原图像素、基座毫米；结果由 map 写出。
void calibratePlane(const std::string &id, double planeZmm, double allowedErrorMm, const std::vector<PointPair> &fit,
                    const std::vector<PointPair> &independentChecks, PlaneMap &map);
void savePlane(const std::string &path, const PlaneMap &);
void loadPlane(const std::string &path, PlaneMap &map); // 读取新采点或已保存标定，统一复算并验误。
void checkCamera(const cv::FileNode &node);

struct Board
{
    cv::Matx33d gridToPixel = cv::Matx33d::eye();   // (列,行)到几何像素；第0行在红方底线。
    std::array<cv::Point2d, Cells> intersections{}; // 按 行*9+列 保存90个交点。
    bool geometryValid = false, orientationValid = false;
    double gridScore = 0;
    std::string orientationSource;
};

// 四角按逻辑格点(0,0)、(8,0)、(8,9)、(0,9)给出，需对应红黑方向。
void boardFromCorners(const std::array<cv::Point2d, 4> &rawCorners, bool canonicalOrderConfirmed, Board &board);
// 固定棋盘的俯视展示。
void rectifiedBoard(const cv::Mat &geometricImage, const Board &board, cv::Mat &image);

struct Observation
{
    std::string frameId;
    std::array<int, Cells> pieces{};
    bool valid = false; // 本帧结果；提交棋局前再经过连续帧稳定确认。
};
void assignPieces(const Board &board, const std::string &detectionFrameId, const std::vector<Detection> &detections,
                  bool sceneClear, Observation &observation);
class Stabilizer
{
  public:
    void update(const Observation &input, Observation &result);
    void reset();

  private:
    int count_ = 0;
    Observation previous_;
};

// 固定交点转换成基座XY，与帧号无关；方向和覆盖范围满足条件才返回true。
// 关节逆解和Z层次由运动模块处理。
bool gridTarget(const PlaneMap &map, const Board &board, int column, int row, cv::Point2d &baseMm);
cv::Point2d project(const cv::Matx33d &, cv::Point2d);
void drawBoard(const cv::Mat &source, const Board &board, cv::Mat &image);
} // namespace scara
