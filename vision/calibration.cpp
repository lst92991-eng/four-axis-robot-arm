//计算并验收平面标定，保存和读取标定文件
#include "calibration.hpp"
#include "geometry.hpp"
#include "io.hpp"
#include <opencv2/geometry.hpp>
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace scara
{
static void require(bool ok, const char *message);
static void hull(const std::vector<cv::Point2d> &points, std::vector<cv::Point2f> &boundary);
static void errorReport(const cv::Matx23d &matrix, const std::vector<PointPair> &pairs, ErrorReport &report);
static void readPairs(const cv::FileNode &node, std::vector<PointPair> &pairs);
static void writePairs(cv::FileStorage &fs, const char *key, const std::vector<PointPair> &pairs);

//---------标定初始化区---------
void loadPlane(const std::string &path, PlaneMap &map)
{
    cv::FileStorage fs;
    PlaneMap prepared;
    std::vector<PointPair> fit, checks;
    std::string id;
    double planeZmm, toleranceMm;
    cv::Mat saved;
    cv::Matx23d matrix;

    readStorage(path, fs);
    //版本2保存2×3仿射矩阵，旧单应矩阵需从原采点重新标定
    require(fs["schema_version"].empty() || (int)fs["schema_version"] == 2,
            "Affine map requires schema 2; recalibrate from measured point pairs");
    require(fs["plane_z_mm"].isReal() || fs["plane_z_mm"].isInt(), "Plane height is missing");
    require(fs["allowed_error_mm"].isReal() || fs["allowed_error_mm"].isInt(), "Validation tolerance is missing");
    readPairs(fs["fit_points"], fit);
    readPairs(fs["check_points"], checks);
    checkCamera(fs["camera"]);
    fs["calibration_id"] >> id;
    fs["plane_z_mm"] >> planeZmm;
    fs["allowed_error_mm"] >> toleranceMm;
    //从保存的实测点重新计算标定，复核误差要求
    calibratePlane(id, planeZmm, toleranceMm, fit, checks, prepared);
    if (!fs["pixel_to_base"].empty())
    {
        fs["pixel_to_base"] >> saved;
        //按2×3读取六个系数，并与原采点的重新拟合结果核对
        matrix = saved;
        require(cv::checkRange(cv::Mat(matrix)) && cv::norm(matrix - prepared.pixelToBase, cv::NORM_INF) < 1e-9,
                "Stored affine map differs from calibration samples");
    }
    if (!prepared.accepted)
        throw std::runtime_error("Affine fit/check error: " + std::to_string(prepared.fitError.maxMm) + "/" +
                                 std::to_string(prepared.checkError.maxMm) + " mm; limit: " + std::to_string(toleranceMm));
    map = prepared;
}

//---------平面标定区---------
void calibratePlane(const std::string &id, double z, double tolerance, const std::vector<PointPair> &fit,
                    const std::vector<PointPair> &checks, PlaneMap &map)
{
    //合并后的采点，用于统一检查有效性和重复坐标
    std::vector<PointPair> samples;
    //原图中有效的像素范围
    cv::Rect2d frame(0, 0, ImageWidth, ImageHeight);
    //参与拟合的原图像素点与实测基座XY，基座单位为毫米
    std::vector<cv::Point2d> pixels, xy;
    //基座采点的覆盖区域，用于确认它们没有共线
    std::vector<cv::Point2f> baseCoverage;
    //局部保存计算结果，完成后交给输出参数
    PlaneMap prepared;

    //核对标定编号、平面高度和允许误差，长度单位为毫米
    require(!id.empty(), "Calibration ID required");
    require(std::isfinite(z) && std::isfinite(tolerance) && tolerance > 0, "Real plane Z and error tolerance required");
    //至少三组不共线点求六个系数，另外实测的检查点用于验误
    require(fit.size() >= 3 && !checks.empty(), "Need >=3 fit pairs and an independent check pair");
    //将两组实测点合在一起，检查坐标及重复情况
    samples = fit;
    samples.insert(samples.end(), checks.begin(), checks.end());
    for (size_t i = 0; i < samples.size(); ++i)
    {
        const PointPair &point = samples[i];
        require(frame.contains(point.pixel) && std::isfinite(point.baseXY.x) && std::isfinite(point.baseXY.y),
                "Invalid measured point");
        //与前面的采点比较，避免重复像素位置或基座位置
        for (size_t j = 0; j < i; ++j)
            require(cv::norm(point.pixel - samples[j].pixel) > 1 && cv::norm(point.baseXY - samples[j].baseXY) > 1e-6,
                    "Calibration samples must be independent");
    }
    //提取用于计算矩阵的拟合点，检查点留到验误阶段使用
    for (const PointPair &point : fit)
    {
        pixels.push_back(point.pixel);
        xy.push_back(point.baseXY);
    }
    //确认采点覆盖了二维区域，并保存其图像覆盖范围
    hull(pixels, prepared.validPixelHull);
    hull(xy, baseCoverage);
    //按全部拟合点求出像素坐标到基座XY的映射
    fitAffine(pixels, xy, prepared.pixelToBase);
    //保存标定说明和允许误差
    prepared.id = id;
    prepared.planeZmm = z;
    prepared.allowedErrorMm = tolerance;
    //保留原始实测点，供保存、回读和复核使用
    prepared.fitPoints = fit;
    prepared.checkPoints = checks;
    //分别统计拟合点与独立检查点的毫米误差
    errorReport(prepared.pixelToBase, fit, prepared.fitError);
    errorReport(prepared.pixelToBase, checks, prepared.checkError);
    //两组最大误差均满足要求时，标记这份映射可用
    prepared.accepted = std::isfinite(prepared.fitError.rmsMm) && std::isfinite(prepared.checkError.rmsMm) &&
                        prepared.fitError.maxMm <= tolerance && prepared.checkError.maxMm <= tolerance;
    //交付矩阵、误差报告和是否通过的状态
    map = prepared;
}

//---------误差计算区---------
static void errorReport(const cv::Matx23d &matrix, const std::vector<PointPair> &pairs, ErrorReport &report)
{
    cv::Point2d predicted; // 由图像点算出的基座XY，毫米。
    double errorMm;

    report = {};
    for (const PointPair &pair : pairs)
    {
        //把像素点换算到基座XY，与实测位置比较
        transformPoint(matrix, pair.pixel, predicted);
        errorMm = cv::norm(predicted - pair.baseXY);
        report.maxMm = std::max(report.maxMm, errorMm);
        report.rmsMm += errorMm * errorMm;
    }
    //平方误差求均值后开方，得到均方根误差，单位为毫米
    report.rmsMm = std::sqrt(report.rmsMm / pairs.size());
}

//---------标定文件保存区---------
void savePlane(const std::string &path, const PlaneMap &map)
{
    cv::FileStorage fs;
    cv::Mat matrix(map.pixelToBase);
    std::string message;

    require(map.accepted, "Calibration failed; not saving a usable plane map");
    memoryStorage(false, fs);
    fs << "schema_version" << 2;
    fs << "calibration_id" << map.id;
    fs << "plane_z_mm" << map.planeZmm;
    fs << "allowed_error_mm" << map.allowedErrorMm;
    fs << "camera" << "{";
    fs << "id" << CameraId;
    fs << "width" << ImageWidth;
    fs << "height" << ImageHeight;
    fs << "}";
    fs << "pixel_to_base" << matrix;
    fs << "fit_rms_mm" << map.fitError.rmsMm;
    fs << "fit_max_mm" << map.fitError.maxMm;
    fs << "check_rms_mm" << map.checkError.rmsMm;
    fs << "check_max_mm" << map.checkError.maxMm;
    writePairs(fs, "fit_points", map.fitPoints);
    writePairs(fs, "check_points", map.checkPoints);
    message = fs.releaseAndGetString();
    writeBytes(path, message);
}

//---------相机与采点检查区---------
void checkCamera(const cv::FileNode &node)
{
    std::string id = (std::string)node["id"];
    int width = int(node["width"]), height = int(node["height"]);
    cv::Mat intrinsic, distortion;

    require(id == CameraId && width == ImageWidth && height == ImageHeight,
            "Layout must match the fixed 1280x720 camera");
    node["K"] >> intrinsic;
    node["distortion"] >> distortion;
    require(intrinsic.empty() && distortion.empty(), "Fixed layout uses validated raw camera pixels");
}

static void require(bool ok, const char *message)
{
    if (!ok) throw std::runtime_error(message);
}

static void hull(const std::vector<cv::Point2d> &points, std::vector<cv::Point2f> &boundary)
{
    std::vector<cv::Point2f> input(points.begin(), points.end());
    cv::convexHull(input, boundary);
    require(boundary.size() >= 3 && std::abs(cv::contourArea(boundary)) > 1.0,
            "Points are collinear or too concentrated");
}

//---------采点文件转换区---------
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

static void writePairs(cv::FileStorage &fs, const char *key, const std::vector<PointPair> &pairs)
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

} // namespace scara
