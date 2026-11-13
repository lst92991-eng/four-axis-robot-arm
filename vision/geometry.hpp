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

struct Camera
{
    std::string id;     // 相机配置名称，随标定文件保存。
    cv::Size imageSize; // 原始图像尺寸，单位像素。
    void validate() const;
    bool sameGeometry(const Camera &) const;
};

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
    Camera camera;
    std::string id;
    double planeZmm = 0, allowedErrorMm = 0; // 标定平面说明与验误要求，不是J1驱动角度。
    cv::Matx33d pixelToBase = cv::Matx33d::eye();
    std::vector<cv::Point2f> validPixelHull; // 拟合点覆盖的几何像素范围。
    std::vector<PointPair> fitPoints, checkPoints;
    ErrorReport fitError, checkError;
    bool accepted = false;
};

// 拟合点和独立检查点使用原图像素、基座毫米；结果由 map 写出。
void calibratePlane(const Camera &camera, const std::string &id, double planeZmm, double allowedErrorMm,
                    const std::vector<PointPair> &fit, const std::vector<PointPair> &independentChecks, PlaneMap &map);
void savePlane(const std::string &path, const PlaneMap &);
void loadPlane(const std::string &path, PlaneMap &map); // 读取新采点或已保存标定，统一复算并验误。
void saveCamera(cv::FileStorage &, const Camera &);
void readCamera(const cv::FileNode &node, Camera &camera);
void readPairs(const cv::FileNode &node, std::vector<PointPair> &pairs);

struct Board
{
    std::string cameraId, frameId;
    Camera geometryCamera;                          // 建系时使用的相机参数。
    cv::Matx33d gridToPixel = cv::Matx33d::eye();   // (列,行)到几何像素；第0行在红方底线。
    std::array<cv::Point2d, Cells> intersections{}; // 按 行*9+列 保存90个交点。
    bool geometryValid = false, orientationValid = false;
    double gridScore = 0;
    std::string orientationSource;
};

// 四角按逻辑格点(0,0)、(8,0)、(8,9)、(0,9)给出，需对应红黑方向。
void boardFromCorners(const Camera &camera, const std::string &frameId, const std::array<cv::Point2d, 4> &rawCorners,
                      bool canonicalOrderConfirmed, Board &board);
// 固定棋盘的俯视展示。
void rectifiedBoard(const cv::Mat &geometricImage, const Board &board, cv::Mat &image);

struct Observation
{
    std::string cameraId, frameId;
    std::array<int, Cells> pieces{};
    std::array<double, Cells> confidence{};
    bool valid = false; // 本帧结果；提交棋局前再经过连续帧稳定确认。
    std::vector<std::string> issues;
};
void assignPieces(const Camera &camera, const Board &board, const std::string &detectionFrameId,
                  const std::vector<Detection> &detections, bool sceneClear, Observation &observation);
class Stabilizer
{
  public:
    explicit Stabilizer(int frames = 3, double cornerTolerancePixels = 2.0);
    void update(const Board &board, const Observation &input, Observation &result);
    void reset();

  private:
    int needed_, count_ = 0;
    double tolerance_;
    Board previousBoard_;
    Observation previous_;
};

// 交点转换成基座XY；方向、帧号和覆盖范围满足条件才返回true。
// 关节逆解和Z层次由运动模块处理。
bool gridTarget(const PlaneMap &map, const Board &board, int column, int row, const std::string &currentFrameId,
                cv::Point2d &baseMm);
cv::Point2d project(const cv::Matx33d &, cv::Point2d);
void drawBoard(const cv::Mat &source, const Board &board, cv::Mat &image);
} // namespace scara
