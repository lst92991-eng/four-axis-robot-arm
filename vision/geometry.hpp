#pragma once
#include "yolo.hpp"
#include <opencv2/core.hpp>
#include <array>
#include <string>
#include <vector>

// Geometry only. No CAN, motor, homing, or gripper calls.
namespace scara
{
constexpr int Columns = 9, Rows = 10, Cells = Columns * Rows;
constexpr int Unknown = -2, Empty = -1;
constexpr char PieceSymbols[] = "kabnrcpKABNRCP";

struct Camera
{
    std::string id;
    cv::Size imageSize;
    cv::Mat K, distortion; // Both empty = raw pixels, subject to independent error validation.
    void validate() const;
    bool sameGeometry(const Camera &) const;
    cv::Point2d geometricPixel(cv::Point2d raw) const; // 使用已验证的相机配置，输入输出单位均为像素。
    void geometricImage(const cv::Mat &raw, cv::Mat &image) const;
};

struct PointPair
{
    cv::Point2d pixel, baseXY;
}; // Pixel is in ORIGINAL camera image; XY is mm.
struct ErrorReport
{
    double rmsMm = 0, maxMm = 0;
    std::vector<double> errorsMm;
};
struct PlaneMap
{
    Camera camera;
    std::string id;
    double planeZmm = 0, allowedErrorMm = 0;
    cv::Matx33d pixelToBase = cv::Matx33d::eye();
    std::vector<cv::Point2d> validPixelHull; // In geometric pixels, conservative fit-point hull.
    std::vector<PointPair> fitPoints, checkPoints;
    ErrorReport fitError, checkError;
    bool accepted = false;
};

// 拟合点和独立检查点使用原图像素、基座毫米；结果由 map 写出。
void calibratePlane(const Camera &camera, const std::string &id, double planeZmm, double allowedErrorMm,
                    const std::vector<PointPair> &fit, const std::vector<PointPair> &independentChecks, PlaneMap &map);
void savePlane(const std::string &path, const PlaneMap &);
void loadPlane(const std::string &path, PlaneMap &map); // 从保存的采点复算映射和误差。
void saveCamera(cv::FileStorage &, const Camera &);
void readCamera(const cv::FileNode &node, Camera &camera);

struct Board
{
    std::string cameraId, frameId;
    Camera geometryCamera;                        // Numeric profile checked as well as the human-readable ID.
    cv::Matx33d gridToPixel = cv::Matx33d::eye(); // (column,row) -> geometric pixels
    std::array<cv::Point2d, Cells> intersections{};
    bool geometryValid = false, orientationValid = false;
    double gridScore = 0;
    std::string orientationSource;
};

// Corner order MUST be logical (0,0),(8,0),(8,9),(0,9), NOT screen top-left order.
void boardFromCorners(const Camera &camera, const std::string &frameId, const std::array<cv::Point2d, 4> &rawCorners,
                      bool canonicalOrderConfirmed, Board &board);
struct LocateOptions
{
    double minAreaFraction = 0.08;
    double minLineSupport = 0.65;
    double minGridScore = 0.50;
};
// Searches natural border/grid features. A successful geometry result is deliberately UNORIENTED.
void locateBoard(const cv::Mat &raw, const Camera &camera, const std::string &frameId, const LocateOptions &options,
                 Board &board);
// A screen confirmation of which end is canonical, after inspecting the 90-point overlay.
void confirmOrientation(const Board &input, bool rotate180, Board &board);

struct OrientationReference
{
    cv::Mat patch; // An asymmetric printed-board feature, e.g. river text; never piece positions.
    cv::Rect patchRect;
    std::string cameraId;
    double minCorrelation = 0.70, minMargin = 0.12;
};
void rectifiedBoard(const cv::Mat &geometricImage, const Board &board, cv::Mat &image);
void makeOrientationReference(const cv::Mat &raw, const Camera &camera, const Board &board,
                              cv::Rect patchInRectifiedImage, OrientationReference &reference);
void orientByReference(const cv::Mat &raw, const Camera &camera, const Board &input,
                       const OrientationReference &reference, Board &board);
void saveOrientationReference(const std::string &, const OrientationReference &);
void loadOrientationReference(const std::string &path, OrientationReference &reference);

enum class DetectionSpace
{
    Original,
    RotatedRoi,
    Network
};
struct ImageTransform
{
    cv::Size sourceSize;
    cv::Rect roi;
    int clockwiseDegrees = 0;
    cv::Size resizedSize; // Actual integer resize BEFORE letterbox padding (network-space input only).
    cv::Point2d padding;
    cv::Point2d toOriginal(cv::Point2d, DetectionSpace) const;
};
struct Observation
{
    std::string cameraId, frameId;
    std::array<int, Cells> pieces{};
    std::array<double, Cells> confidence{};
    bool valid = false; // Single-frame result; use Stabilizer before committing a board state.
    std::vector<std::string> issues;
};
struct AssignmentOptions
{
    double minConfidence = 0.50;   // 接受的最小识别分数。
    double maxGridDistance = 0.35; // 棋子中心距最近交点，单位为格距。
};
void assignPieces(const Camera &camera, const Board &board, const std::string &detectionFrameId,
                  const std::vector<Detection> &detections, const ImageTransform &transform, DetectionSpace space,
                  bool sceneClear, const AssignmentOptions &options, Observation &observation);
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

struct TargetXY
{
    int column = 0, row = 0;
    cv::Point2d baseMm;
    double planeZmm = 0;
    std::string calibrationId, frameId;
};
// Pure calculation: validates orientation, camera identity and calibrated coverage.
// IK reachability / Z approach / TCP compensation are responsibilities of the motion caller.
bool gridTarget(const PlaneMap &map, const Board &board, int column, int row, const std::string &currentFrameId,
                TargetXY &target);
cv::Point2d project(const cv::Matx33d &, cv::Point2d);
void drawBoard(const cv::Mat &geometricImage, const Board &board, const Observation *observation, cv::Mat &image);
} // namespace scara
