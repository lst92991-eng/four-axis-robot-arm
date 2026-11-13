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
cv::Matx33d matrix33(const cv::Mat &input)
{
    require(input.rows == 3 && input.cols == 3 && input.channels() == 1, "Expected a scalar 3x3 matrix");
    cv::Mat m;
    input.convertTo(m, CV_64F);
    require(cv::checkRange(m), "Nonfinite matrix");
    cv::Matx33d h;
    std::copy(m.ptr<double>(), m.ptr<double>() + 9, h.val);
    require(std::abs(cv::determinant(h)) > 1e-14, "Singular matrix");
    return h;
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
    report = {};
    for (const auto &pair : pairs)
    {
        double errorMm = cv::norm(project(matrix, pair.pixel) - pair.baseXY);
        report.errorsMm.push_back(errorMm);
        report.maxMm = std::max(report.maxMm, errorMm);
        report.rmsMm += errorMm * errorMm;
    }
    report.rmsMm = std::sqrt(report.rmsMm / pairs.size());
}
void writePairs(cv::FileStorage &fs, const char *key, const std::vector<PointPair> &pairs)
{
    fs << key << "[";
    for (const auto &p : pairs)
        fs << "{" << "pixel" << "[" << p.pixel.x << p.pixel.y << "]"
           << "base_xy" << "[" << p.baseXY.x << p.baseXY.y << "]" << "}";
    fs << "]";
}
} // namespace

static void readPairs(const cv::FileNode &node, std::vector<PointPair> &pairs)
{
    pairs.clear();
    require(node.isSeq(), "Missing point pair sequence");
    for (const auto &point : node)
    {
        auto pixel = point["pixel"], base = point["base_xy"];
        require(pixel.size() == 2 && base.size() == 2, "Each sample needs pixel and base_xy");
        for (auto value : {pixel[0], pixel[1], base[0], base[1]})
            require(value.isReal() || value.isInt(), "Sample coordinates must be measured numbers");
        PointPair pair;
        pair.pixel = {double(pixel[0]), double(pixel[1])};
        pair.baseXY = {double(base[0]), double(base[1])};
        pairs.push_back(pair);
    }
}
cv::Point2d project(const cv::Matx33d &h, cv::Point2d p)
{
    const auto q = h * cv::Vec3d(p.x, p.y, 1);
    require(std::isfinite(q[2]) && std::abs(q[2]) > 1e-10, "Projection at infinity");
    return {q[0] / q[2], q[1] / q[2]};
}
void calibratePlane(const std::string &id, double z, double tolerance, const std::vector<PointPair> &fit,
                    const std::vector<PointPair> &checks, PlaneMap &map)
{
    require(!id.empty(), "Calibration ID required");
    require(std::isfinite(z) && std::isfinite(tolerance) && tolerance > 0, "Real plane Z and error tolerance required");
    require(fit.size() >= 4 && !checks.empty(), "Need >=4 fit pairs and an independent check pair");
    // 拟合点和检查点必须分别实测；先统一检查，后面只做计算。
    std::vector<PointPair> samples = fit;
    samples.insert(samples.end(), checks.begin(), checks.end());
    for (size_t i = 0; i < samples.size(); ++i)
    {
        const auto &point = samples[i];
        require(finite(point.pixel) && finite(point.baseXY) && point.pixel.x >= 0 && point.pixel.y >= 0 &&
                    point.pixel.x < ImageWidth && point.pixel.y < ImageHeight,
                "Invalid measured point");
        for (size_t j = 0; j < i; ++j)
            require(cv::norm(point.pixel - samples[j].pixel) > 1 && cv::norm(point.baseXY - samples[j].baseXY) > 1e-6,
                    "Calibration samples must be independent");
    }
    std::vector<cv::Point2d> pixels, xy;
    for (const auto &point : fit)
    {
        pixels.push_back(point.pixel);
        xy.push_back(point.baseXY);
    }
    std::vector<cv::Point2f> coverage, baseCoverage;
    hull(pixels, coverage);
    hull(xy, baseCoverage);
    PlaneMap m;
    m.id = id;
    m.planeZmm = z;
    m.allowedErrorMm = tolerance;
    m.pixelToBase = matrix33(cv::findHomography(pixels, xy, 0));
    m.validPixelHull = coverage;
    m.fitPoints = fit;
    m.checkPoints = checks;
    errorReport(m.pixelToBase, fit, m.fitError);
    errorReport(m.pixelToBase, checks, m.checkError);
    m.accepted = std::isfinite(m.fitError.rmsMm) && std::isfinite(m.checkError.rmsMm) &&
                 m.fitError.maxMm <= tolerance && m.checkError.maxMm <= tolerance;
    map = m;
}
// 只在配置入口核对固定机位；帧循环不重复比较多套相机参数。
void checkCamera(const cv::FileNode &node)
{
    require((std::string)node["id"] == CameraId && (int)node["width"] == ImageWidth &&
                (int)node["height"] == ImageHeight,
            "Layout must match the fixed 1280x720 camera");
    cv::Mat intrinsic, distortion;
    if (!node["K"].empty()) node["K"] >> intrinsic;
    if (!node["distortion"].empty()) node["distortion"] >> distortion;
    require(intrinsic.empty() && distortion.empty(), "Fixed layout uses validated raw camera pixels");
}
void savePlane(const std::string &path, const PlaneMap &m)
{
    require(m.accepted, "Calibration failed; not saving a usable plane map");
    cv::FileStorage fs;
    memoryStorage(false, fs);
    fs << "schema_version" << 1 << "calibration_id" << m.id << "plane_z_mm" << m.planeZmm << "allowed_error_mm"
       << m.allowedErrorMm;
    fs << "camera" << "{" << "id" << CameraId << "width" << ImageWidth << "height" << ImageHeight << "K" << cv::Mat()
       << "distortion" << cv::Mat() << "}";
    fs << "pixel_to_base" << cv::Mat(m.pixelToBase) << "fit_rms_mm" << m.fitError.rmsMm << "fit_max_mm"
       << m.fitError.maxMm << "check_rms_mm" << m.checkError.rmsMm << "check_max_mm" << m.checkError.maxMm;
    fs << "fit_errors_mm" << m.fitError.errorsMm << "check_errors_mm" << m.checkError.errorsMm;
    writePairs(fs, "fit_points", m.fitPoints);
    writePairs(fs, "check_points", m.checkPoints);
    writeBytes(path, fs.releaseAndGetString());
}
void loadPlane(const std::string &path, PlaneMap &map)
{
    cv::FileStorage fs;
    readStorage(path, fs);
    require(fs["schema_version"].empty() || (int)fs["schema_version"] == 1, "Unsupported calibration schema");
    require(fs["plane_z_mm"].isReal() || fs["plane_z_mm"].isInt(), "Plane height is missing");
    require(fs["allowed_error_mm"].isReal() || fs["allowed_error_mm"].isInt(), "Validation tolerance is missing");
    PlaneMap m;
    std::vector<PointPair> fit, checks;
    readPairs(fs["fit_points"], fit);
    readPairs(fs["check_points"], checks);
    checkCamera(fs["camera"]);
    calibratePlane((std::string)fs["calibration_id"], (double)fs["plane_z_mm"], (double)fs["allowed_error_mm"], fit,
                   checks, m);
    if (!fs["pixel_to_base"].empty())
    {
        cv::Mat saved;
        fs["pixel_to_base"] >> saved;
        auto matrix = matrix33(saved);
        for (const auto &point : m.fitPoints)
            require(cv::norm(project(matrix, point.pixel) - project(m.pixelToBase, point.pixel)) < 1e-5,
                    "Stored map differs from calibration samples");
    }
    require(m.accepted, "Independent validation failed");
    map = m;
}

void assignPieces(const Board &board, const std::string &frameId, const std::vector<Detection> &detections, bool clear,
                  Observation &observation)
{
    observation = {};
    observation.frameId = frameId;
    observation.pieces.fill(Unknown);
    observation.valid = board.geometryValid && board.orientationValid && !frameId.empty();
    if (!observation.valid) return;
    if (clear)
        observation.pieces.fill(Empty);
    else
        observation.valid = false;
    constexpr double MinScore = .50, MaxGridDistance = .35; // 分数与格距阈值保持原值。
    std::array<bool, Cells> occupied{};
    auto inverse = board.gridToPixel.inv();
    for (const auto &piece : detections)
    {
        cv::Point2d center(piece.bbox.x + piece.bbox.width / 2, piece.bbox.y + piece.bbox.height / 2);
        bool valid = piece.classId >= 0 && piece.classId < YoloClasses && std::isfinite(piece.confidence) &&
                     piece.confidence >= MinScore && piece.confidence <= 1 && piece.bbox.width > 0 &&
                     piece.bbox.height > 0 && finite(center) && center.x >= 0 && center.y >= 0 &&
                     center.x < ImageWidth && center.y < ImageHeight;
        if (!valid)
        {
            observation.valid = false;
            continue;
        }
        auto homogeneous = inverse * cv::Vec3d(center.x, center.y, 1);
        cv::Point2d grid(homogeneous[0] / homogeneous[2], homogeneous[1] / homogeneous[2]);
        bool inside = finite(grid) && grid.x > -.5 && grid.y > -.5 && grid.x < Columns - .5 && grid.y < Rows - .5;
        if (!inside || cv::norm(grid - cv::Point2d(std::round(grid.x), std::round(grid.y))) > MaxGridDistance)
        {
            observation.valid = false;
            continue;
        }
        int index = int(std::lround(grid.y)) * Columns + int(std::lround(grid.x));
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
bool gridTarget(const PlaneMap &map, const Board &board, int c, int r, cv::Point2d &baseMm)
{
    baseMm = {};
    if (!map.accepted || !board.geometryValid || !board.orientationValid || c < 0 || c >= Columns || r < 0 || r >= Rows)
        return false;
    auto pixel = board.intersections[r * Columns + c];
    if (map.validPixelHull.size() < 3 || cv::pointPolygonTest(map.validPixelHull, pixel, true) < -1e-4) return false;
    baseMm = project(map.pixelToBase, pixel);
    return true;
}
void drawBoard(const cv::Mat &image, const Board &board, cv::Mat &out)
{
    out = image.clone();
    if (!board.geometryValid) return;
    for (int r = 0; r < Rows; ++r)
        for (int c = 0; c < Columns; ++c)
        {
            auto p = board.intersections[r * Columns + c];
            cv::circle(out, p, 3, {0, 255, 255}, -1);
            std::string label = std::to_string(c) + "," + std::to_string(r);
            cv::putText(out, label, p + cv::Point2d(4, -4), cv::FONT_HERSHEY_SIMPLEX, 0.32, {0, 0, 255}, 1,
                        cv::LINE_AA);
        }
    cv::putText(out, board.orientationValid ? "ORIENTATION CONFIRMED" : "ORIENTATION UNKNOWN - NO ROBOT TARGETS",
                {10, 24}, cv::FONT_HERSHEY_SIMPLEX, 0.55, {0, 0, 255}, 2);
}
} // namespace scara
