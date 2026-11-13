#include "geometry.hpp"
#include "io.hpp"
#include <opencv2/geometry.hpp>
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace scara
{
namespace
{
void require(bool ok, const char *message)
{
    if (!ok) throw std::runtime_error(message);
}
bool finite(cv::Point2d p)
{
    return std::isfinite(p.x) && std::isfinite(p.y);
}
void matrix33(const cv::Mat &input, cv::Matx33d &matrix)
{
    cv::Mat values; // 统一使用double保存3x3变换矩阵。

    require(input.rows == 3 && input.cols == 3 && input.channels() == 1, "Expected a scalar 3x3 matrix");
    input.convertTo(values, CV_64F);
    require(cv::checkRange(values), "Nonfinite matrix");
    std::copy(values.ptr<double>(), values.ptr<double>() + 9, matrix.val);
    require(std::abs(cv::determinant(matrix)) > 1e-14, "Singular matrix");
}
void hull(const std::vector<cv::Point2d> &points, std::vector<cv::Point2f> &boundary)
{
    std::vector<cv::Point2f> input(points.begin(), points.end());
    cv::convexHull(input, boundary);
    require(boundary.size() >= 3 && std::abs(cv::contourArea(boundary)) > 1.0,
            "Points are collinear or too concentrated");
}
void errorReport(const cv::Matx33d &matrix, const std::vector<PointPair> &pairs, ErrorReport &report)
{
    cv::Point2d predicted; // 由图像点算出的基座XY，毫米。
    double errorMm;

    report = {};
    for (const PointPair &pair : pairs)
    {
        project(matrix, pair.pixel, predicted);
        errorMm = cv::norm(predicted - pair.baseXY);
        report.errorsMm.push_back(errorMm);
        report.maxMm = std::max(report.maxMm, errorMm);
        report.rmsMm += errorMm * errorMm;
    }
    report.rmsMm = std::sqrt(report.rmsMm / pairs.size());
}
void writePairs(cv::FileStorage &fs, const char *key, const std::vector<PointPair> &pairs)
{
    fs << key << "[";
    for (const PointPair &pair : pairs)
    {
        fs << "{";
        fs << "pixel" << "[" << pair.pixel.x << pair.pixel.y << "]";
        fs << "base_xy" << "[" << pair.baseXY.x << pair.baseXY.y << "]";
        fs << "}";
    }
    fs << "]";
}
} // namespace

static void readPairs(const cv::FileNode &node, std::vector<PointPair> &pairs)
{
    pairs.clear();
    require(node.isSeq(), "Missing point pair sequence");
    for (const cv::FileNode &point : node)
    {
        cv::FileNode pixel = point["pixel"], base = point["base_xy"];
        PointPair pair;
        require(pixel.size() == 2 && base.size() == 2, "Each sample needs pixel and base_xy");
        for (cv::FileNode value : {pixel[0], pixel[1], base[0], base[1]})
            require(value.isReal() || value.isInt(), "Sample coordinates must be measured numbers");
        pair.pixel.x = double(pixel[0]);
        pair.pixel.y = double(pixel[1]);
        pair.baseXY.x = double(base[0]);
        pair.baseXY.y = double(base[1]);
        pairs.push_back(pair);
    }
}
void project(const cv::Matx33d &matrix, cv::Point2d point, cv::Point2d &result)
{
    cv::Vec3d homogeneous; // 齐次坐标；前两项除以第三项得到二维坐标。
    homogeneous = matrix * cv::Vec3d(point.x, point.y, 1);
    require(std::isfinite(homogeneous[2]) && std::abs(homogeneous[2]) > 1e-10, "Projection at infinity");
    result.x = homogeneous[0] / homogeneous[2];
    result.y = homogeneous[1] / homogeneous[2];
}
void calibratePlane(const std::string &id, double z, double tolerance, const std::vector<PointPair> &fit,
                    const std::vector<PointPair> &checks, PlaneMap &map)
{
    std::vector<PointPair> samples;
    std::vector<cv::Point2d> pixels, xy; // 用于拟合的像素点、实测基座坐标。
    std::vector<cv::Point2f> coverage, baseCoverage;
    cv::Mat fitted; // OpenCV拟合得到的3x3矩阵。
    PlaneMap prepared;

    require(!id.empty(), "Calibration ID required");
    require(std::isfinite(z) && std::isfinite(tolerance) && tolerance > 0, "Real plane Z and error tolerance required");
    require(fit.size() >= 4 && !checks.empty(), "Need >=4 fit pairs and an independent check pair");
    // 拟合点和检查点必须分别实测；先统一检查，后面只做计算。
    samples = fit;
    samples.insert(samples.end(), checks.begin(), checks.end());
    for (size_t i = 0; i < samples.size(); ++i)
    {
        const PointPair &point = samples[i];
        require(finite(point.pixel) && finite(point.baseXY) && point.pixel.x >= 0 && point.pixel.y >= 0 &&
                    point.pixel.x < ImageWidth && point.pixel.y < ImageHeight,
                "Invalid measured point");
        for (size_t j = 0; j < i; ++j)
            require(cv::norm(point.pixel - samples[j].pixel) > 1 && cv::norm(point.baseXY - samples[j].baseXY) > 1e-6,
                    "Calibration samples must be independent");
    }
    for (const PointPair &point : fit)
    {
        pixels.push_back(point.pixel);
        xy.push_back(point.baseXY);
    }
    hull(pixels, coverage);
    hull(xy, baseCoverage);
    fitted = cv::findHomography(pixels, xy, 0);
    matrix33(fitted, prepared.pixelToBase);
    prepared.id = id;
    prepared.planeZmm = z;
    prepared.allowedErrorMm = tolerance;
    prepared.validPixelHull = coverage;
    prepared.fitPoints = fit;
    prepared.checkPoints = checks;
    errorReport(prepared.pixelToBase, fit, prepared.fitError);
    errorReport(prepared.pixelToBase, checks, prepared.checkError);
    prepared.accepted = std::isfinite(prepared.fitError.rmsMm) && std::isfinite(prepared.checkError.rmsMm) &&
                        prepared.fitError.maxMm <= tolerance && prepared.checkError.maxMm <= tolerance;
    map = prepared;
}
// 只在配置入口核对固定机位；帧循环不重复比较多套相机参数。
void checkCamera(const cv::FileNode &node)
{
    std::string id = (std::string)node["id"];
    int width = int(node["width"]), height = int(node["height"]);
    cv::Mat intrinsic, distortion;

    require(id == CameraId && width == ImageWidth && height == ImageHeight,
            "Layout must match the fixed 1280x720 camera");
    if (!node["K"].empty()) node["K"] >> intrinsic;
    if (!node["distortion"].empty()) node["distortion"] >> distortion;
    require(intrinsic.empty() && distortion.empty(), "Fixed layout uses validated raw camera pixels");
}
void savePlane(const std::string &path, const PlaneMap &map)
{
    cv::FileStorage fs;
    cv::Mat matrix(map.pixelToBase);
    cv::Mat empty; // 固定原图像素方案未启用相机去畸变。
    std::string message;

    require(map.accepted, "Calibration failed; not saving a usable plane map");
    memoryStorage(false, fs);
    fs << "schema_version" << 1;
    fs << "calibration_id" << map.id;
    fs << "plane_z_mm" << map.planeZmm;
    fs << "allowed_error_mm" << map.allowedErrorMm;
    fs << "camera" << "{";
    fs << "id" << CameraId;
    fs << "width" << ImageWidth;
    fs << "height" << ImageHeight;
    fs << "K" << empty;
    fs << "distortion" << empty;
    fs << "}";
    fs << "pixel_to_base" << matrix;
    fs << "fit_rms_mm" << map.fitError.rmsMm;
    fs << "fit_max_mm" << map.fitError.maxMm;
    fs << "check_rms_mm" << map.checkError.rmsMm;
    fs << "check_max_mm" << map.checkError.maxMm;
    fs << "fit_errors_mm" << map.fitError.errorsMm;
    fs << "check_errors_mm" << map.checkError.errorsMm;
    writePairs(fs, "fit_points", map.fitPoints);
    writePairs(fs, "check_points", map.checkPoints);
    message = fs.releaseAndGetString();
    writeBytes(path, message);
}
void loadPlane(const std::string &path, PlaneMap &map)
{
    cv::FileStorage fs;
    PlaneMap prepared;
    std::vector<PointPair> fit, checks;
    std::string id;
    double planeZmm, toleranceMm;
    cv::Mat saved;
    cv::Matx33d matrix;
    cv::Point2d savedXY, fittedXY; // 保存矩阵和重算矩阵得到的同一个点，毫米。

    readStorage(path, fs);
    require(fs["schema_version"].empty() || (int)fs["schema_version"] == 1, "Unsupported calibration schema");
    require(fs["plane_z_mm"].isReal() || fs["plane_z_mm"].isInt(), "Plane height is missing");
    require(fs["allowed_error_mm"].isReal() || fs["allowed_error_mm"].isInt(), "Validation tolerance is missing");
    readPairs(fs["fit_points"], fit);
    readPairs(fs["check_points"], checks);
    checkCamera(fs["camera"]);
    fs["calibration_id"] >> id;
    fs["plane_z_mm"] >> planeZmm;
    fs["allowed_error_mm"] >> toleranceMm;
    calibratePlane(id, planeZmm, toleranceMm, fit, checks, prepared);
    if (!fs["pixel_to_base"].empty())
    {
        fs["pixel_to_base"] >> saved;
        matrix33(saved, matrix);
        for (const PointPair &point : prepared.fitPoints)
        {
            project(matrix, point.pixel, savedXY);
            project(prepared.pixelToBase, point.pixel, fittedXY);
            require(cv::norm(savedXY - fittedXY) < 1e-5, "Stored map differs from calibration samples");
        }
    }
    require(prepared.accepted, "Independent validation failed");
    map = prepared;
}

void assignPieces(const Board &board, const std::string &frameId, const std::vector<Detection> &detections, bool clear,
                  Observation &observation)
{
    constexpr double MinScore = .50, MaxGridDistance = .35; // 分数与格距阈值保持原值。
    std::array<bool, Cells> occupied{};
    cv::Matx33d inverse;

    observation = {};
    observation.frameId = frameId;
    observation.pieces.fill(Unknown);
    observation.valid = board.geometryValid && board.orientationValid && !frameId.empty();
    if (!observation.valid) return;
    if (clear)
        observation.pieces.fill(Empty);
    else
        observation.valid = false;
    inverse = board.gridToPixel.inv();
    for (const Detection &piece : detections)
    {
        cv::Point2d center;        // 棋子检测框中心，原图像素。
        cv::Point2d grid, nearest; // 连续格点坐标和最近的整数交点。
        cv::Vec3d homogeneous;
        int column, row, index;
        bool valid, inside;

        center.x = piece.bbox.x + piece.bbox.width / 2;
        center.y = piece.bbox.y + piece.bbox.height / 2;
        valid = piece.classId >= 0 && piece.classId < YoloClasses && std::isfinite(piece.confidence) &&
                piece.confidence >= MinScore && piece.confidence <= 1 && piece.bbox.width > 0 &&
                piece.bbox.height > 0 && finite(center) && center.x >= 0 && center.y >= 0 && center.x < ImageWidth &&
                center.y < ImageHeight;
        if (!valid)
        {
            observation.valid = false;
            continue;
        }
        homogeneous = inverse * cv::Vec3d(center.x, center.y, 1);
        grid.x = homogeneous[0] / homogeneous[2];
        grid.y = homogeneous[1] / homogeneous[2];
        inside = finite(grid) && grid.x > -.5 && grid.y > -.5 && grid.x < Columns - .5 && grid.y < Rows - .5;
        if (inside)
        {
            nearest.x = std::round(grid.x);
            nearest.y = std::round(grid.y);
            inside = cv::norm(grid - nearest) <= MaxGridDistance;
        }
        if (!inside)
        {
            observation.valid = false;
            continue;
        }
        column = int(std::lround(grid.x));
        row = int(std::lround(grid.y));
        index = row * Columns + column;
        observation.pieces[index] = occupied[index] ? Unknown : piece.classId;
        if (occupied[index]) observation.valid = false;
        occupied[index] = true;
    }
}
void Stabilizer::reset()
{
    count_ = 0;
    previous_ = Observation{};
}
void Stabilizer::update(const Observation &input, Observation &result)
{
    result = input;
    if (!input.valid)
    {
        reset();
        return;
    }
    if (input.frameId == previous_.frameId)
    {
        result.valid = false;
        return;
    }
    bool same = count_ > 0 && input.pieces == previous_.pieces;
    count_ = same ? std::min(count_ + 1, 3) : 1;
    previous_ = input;
    result.valid = count_ >= 3;
}
bool gridTarget(const PlaneMap &map, const Board &board, int column, int row, cv::Point2d &baseMm)
{
    cv::Point2d pixel;

    baseMm = {};
    if (!map.accepted || !board.geometryValid || !board.orientationValid) return false;
    if (column < 0 || column >= Columns || row < 0 || row >= Rows) return false;
    pixel = board.intersections[row * Columns + column];
    if (map.validPixelHull.size() < 3 || cv::pointPolygonTest(map.validPixelHull, pixel, true) < -1e-4) return false;
    project(map.pixelToBase, pixel, baseMm);
    return true;
}
void drawBoard(const cv::Mat &image, const Board &board, cv::Mat &out)
{
    out = image.clone();
    if (!board.geometryValid) return;
    for (int r = 0; r < Rows; ++r)
        for (int c = 0; c < Columns; ++c)
        {
            cv::Point2d pixel = board.intersections[r * Columns + c];
            cv::Point2d textOrigin = pixel + cv::Point2d(4, -4);
            std::string label = std::to_string(c) + "," + std::to_string(r);
            cv::circle(out, pixel, 3, {0, 255, 255}, -1);
            cv::putText(out, label, textOrigin, cv::FONT_HERSHEY_SIMPLEX, 0.32, {0, 0, 255}, 1, cv::LINE_AA);
        }
    cv::putText(out, board.orientationValid ? "ORIENTATION CONFIRMED" : "ORIENTATION UNKNOWN - NO ROBOT TARGETS",
                {10, 24}, cv::FONT_HERSHEY_SIMPLEX, 0.55, {0, 0, 255}, 2);
}
} // namespace scara
