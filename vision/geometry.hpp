#pragma once
#include <opencv2/core.hpp>
#include <array>
#include <optional>
#include <string>
#include <vector>

// Geometry only. No CAN, motor, homing, or gripper calls.
namespace scara {
constexpr int Columns = 9, Rows = 10, Cells = Columns * Rows;
constexpr int Unknown = -2, Empty = -1;
constexpr char PieceSymbols[] = "kabnrcpKABNRCP";

struct Camera {
    std::string id;
    cv::Size imageSize;
    cv::Mat K, distortion; // Both empty = raw pixels, subject to independent error validation.
    void validate() const;
    bool sameGeometry(const Camera&) const;
    cv::Point2d geometricPixel(cv::Point2d raw) const;
    cv::Mat geometricImage(const cv::Mat& raw) const;
};

struct PointPair { cv::Point2d pixel, baseXY; }; // Pixel is in ORIGINAL camera image; XY is mm.
struct ErrorReport { double rmsMm = 0, maxMm = 0; std::vector<double> errorsMm; };
struct PlaneMap {
    Camera camera;
    std::string id;
    double planeZmm = 0, allowedErrorMm = 0;
    cv::Matx33d pixelToBase = cv::Matx33d::eye();
    std::vector<cv::Point2d> validPixelHull; // In geometric pixels, conservative fit-point hull.
    std::vector<PointPair> fitPoints, checkPoints;
    ErrorReport fitError, checkError;
    bool accepted = false;
};

// At least four non-collinear fit points and three different check points.
PlaneMap calibratePlane(const Camera&, const std::string& id, double planeZmm,
                        double allowedErrorMm, const std::vector<PointPair>& fit,
                        const std::vector<PointPair>& independentChecks);
void savePlane(const std::string& path, const PlaneMap&);
PlaneMap loadPlane(const std::string& path); // Recomputes and verifies, never trusts a stored valid flag.
void saveCamera(cv::FileStorage&, const Camera&);
Camera readCamera(const cv::FileNode&);

struct Board {
    std::string cameraId, frameId;
    Camera geometryCamera; // Numeric profile checked as well as the human-readable ID.
    cv::Matx33d gridToPixel = cv::Matx33d::eye(); // (column,row) -> geometric pixels
    std::array<cv::Point2d, Cells> intersections{};
    bool geometryValid = false, orientationValid = false;
    double gridScore = 0;
    std::string orientationSource;
};

// Corner order MUST be logical (0,0),(8,0),(8,9),(0,9), NOT screen top-left order.
Board boardFromCorners(const Camera&, const std::string& frameId,
                       const std::array<cv::Point2d,4>& rawCorners, bool canonicalOrderConfirmed);
struct LocateOptions {
    double minAreaFraction = 0.08;
    double minLineSupport = 0.65;
    double minGridScore = 0.50;
};
// Searches natural border/grid features. A successful geometry result is deliberately UNORIENTED.
Board locateBoard(const cv::Mat& raw, const Camera&, const std::string& frameId,
                  const LocateOptions& = {});
// A screen confirmation of which end is canonical, after inspecting the 90-point overlay.
Board confirmOrientation(const Board&, bool rotate180);

struct OrientationReference {
    cv::Mat patch; // An asymmetric printed-board feature, e.g. river text; never piece positions.
    cv::Rect patchRect;
    std::string cameraId;
    double minCorrelation = 0.70, minMargin = 0.12;
};
cv::Mat rectifiedBoard(const cv::Mat& geometricImage, const Board&);
OrientationReference makeOrientationReference(const cv::Mat& raw, const Camera&, const Board&,
                                              cv::Rect patchInRectifiedImage);
Board orientByReference(const cv::Mat& raw, const Camera&, const Board&, const OrientationReference&);
void saveOrientationReference(const std::string&, const OrientationReference&);
OrientationReference loadOrientationReference(const std::string&);

enum class DetectionSpace { Original, RotatedRoi, Network };
struct ImageTransform {
    cv::Size sourceSize;
    cv::Rect roi;
    int clockwiseDegrees = 0;
    cv::Size resizedSize; // Actual integer resize BEFORE letterbox padding (network-space input only).
    cv::Point2d padding;
    cv::Point2d toOriginal(cv::Point2d, DetectionSpace) const;
};
struct Detection { int classId = -1; double confidence = 0; cv::Rect2d bbox; };
struct Observation {
    std::string cameraId, frameId;
    std::array<int,Cells> pieces{};
    std::array<double,Cells> confidence{};
    bool valid = false; // Single-frame result; use Stabilizer before committing a board state.
    std::vector<std::string> issues;
};
Observation assignPieces(const Camera&, const Board&, const std::string& detectionFrameId,
                         const std::vector<Detection>&, const ImageTransform&, DetectionSpace,
                         bool sceneClear, double minConfidence = 0.50, double maxGridDistance = 0.35);
class Stabilizer {
public:
    explicit Stabilizer(int frames = 3, double cornerTolerancePixels = 2.0);
    Observation update(const Board&, const Observation&);
    void reset();
private:
    int needed_, count_ = 0;
    double tolerance_;
    Board previousBoard_;
    Observation previous_;
};

struct TargetXY {
    int column = 0, row = 0;
    cv::Point2d baseMm;
    double planeZmm = 0;
    std::string calibrationId, frameId;
};
// Pure calculation: validates orientation, camera identity and calibrated coverage.
// IK reachability / Z approach / TCP compensation are responsibilities of the motion caller.
std::optional<TargetXY> gridTarget(const PlaneMap&, const Board&, int column, int row,
                                  const std::string& currentFrameId);
cv::Point2d project(const cv::Matx33d&, cv::Point2d);
cv::Mat drawBoard(const cv::Mat& geometricImage, const Board&, const Observation* = nullptr);
}
