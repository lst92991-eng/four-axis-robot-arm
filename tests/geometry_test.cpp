#include "vision/geometry.hpp"
#include "vision/io.hpp"
#include <opencv2/geometry.hpp>
#include <opencv2/imgproc.hpp>
#include <filesystem>
#include <iostream>
#include <functional>
#include <cmath>

using namespace scara;
namespace
{
int checks = 0;
void check(bool v, const char *message)
{
    ++checks;
    if (!v) throw std::runtime_error(message);
}
void rejects(const std::function<void()> &f)
{
    bool thrown = false;
    try
    {
        f();
    }
    catch (const std::exception &)
    {
        thrown = true;
    }
    check(thrown, "Invalid input was accepted");
}

const cv::Matx23d BaseAffine(0.4, 0.02, 100, -0.01, 0.5, -80);
PlaneMap plane()
{
    std::vector<PointPair> fit, validation;
    cv::Point2d base;
    for (auto p : std::vector<cv::Point2d>{{20, 20}, {620, 20}, {620, 460}, {20, 460}, {320, 70}, {180, 330}})
    {
        transformPoint(BaseAffine, p, base);
        fit.push_back({p, base});
    }
    for (auto p : std::vector<cv::Point2d>{{150, 180}, {320, 260}, {450, 300}})
    {
        transformPoint(BaseAffine, p, base);
        validation.push_back({p, base});
    }
    PlaneMap result;
    calibratePlane("synthetic-map-do-not-use-on-robot", 12, 0.001, fit, validation, result);
    return result;
}
Board board()
{
    Board result;
    boardFromCorners({{{150, 80}, {500, 100}, {480, 400}, {130, 380}}}, true, result);
    return result;
}
void calibrationTests()
{
    auto p = plane();
    check(p.accepted && p.checkError.maxMm < 0.001, "Exact affine fit failed");
    check(cv::norm(p.pixelToBase - BaseAffine, cv::NORM_INF) < 1e-10, "Six affine coefficients differ");
    // 三个不共线点可以求六个系数，检查点仍需独立
    PlaneMap minimal;
    calibratePlane(p.id, 0, .001, {p.fitPoints[0], p.fitPoints[1], p.fitPoints[2]}, p.checkPoints, minimal);
    check(minimal.accepted, "Three noncollinear pairs rejected");
    // 正方形四角有一角X误差4mm，最小二乘应让四角各分担1mm残差
    std::vector<PointPair> noisy{{{100,100},{14,10}}, {{300,100},{30,10}},
                               {{300,300},{30,30}}, {{100,300},{10,30}}};
    calibratePlane("all-points", 0, 1.01, noisy, {{{200,200},{21,20}}}, minimal);
    check(minimal.accepted && std::abs(minimal.fitError.maxMm - 1) < 1e-10,
          "Affine fit did not use every sample");
    check(cv::norm(minimal.pixelToBase - cv::Matx23d(.09,-.01,5,0,.1,0), cv::NORM_INF) < 1e-10,
          "Least-squares coefficients differ from analytic result");
    auto path = (std::filesystem::temp_directory_path() / std::filesystem::u8path("scara_几何测试.yml")).u8string();
    savePlane(path, p);
    PlaneMap loaded;
    loadPlane(path, loaded);
    check(loaded.id == p.id && loaded.accepted, "Persistence failed");
    auto bad = p.checkPoints;
    bad[0].baseXY.x += 10;
    PlaneMap attempted;
    calibratePlane(p.id, p.planeZmm, p.allowedErrorMm, p.fitPoints, bad, attempted);
    check(!attempted.accepted, "Validation error not rejected");
    rejects(
        [&]
        {
            auto v = p.checkPoints;
            v[0] = p.fitPoints[0];
            calibratePlane(p.id, 0, 1, p.fitPoints, v, attempted);
        });
    rejects(
        [&]
        {
            std::vector<PointPair> line;
            for (int i = 0; i < 4; ++i)
                line.push_back({{50. + i * 50, 100}, {double(i), 0}});
            calibratePlane(p.id, 0, 1, line, p.checkPoints, attempted);
        });
    std::string altered;
    readBytes(path, altered);
    auto at = altered.find("pixel_to_base:");
    check(at != std::string::npos, "Missing saved matrix");
    // 保存矩阵的元素数相同但形状错误时，加载仍需拒绝。
    auto wrongShape = altered;
    auto rowsAt = wrongShape.find("rows: 2", at);
    auto colsAt = wrongShape.find("cols: 3", at);
    check(rowsAt != std::string::npos && colsAt != std::string::npos, "Missing matrix shape");
    wrongShape[rowsAt + 6] = '1';
    wrongShape[colsAt + 6] = '6';
    writeBytes(path, wrongShape);
    rejects([&] { loadPlane(path, attempted); });
    auto oldVersion = altered;
    auto versionAt = oldVersion.find("schema_version: 2");
    check(versionAt != std::string::npos, "Missing affine schema");
    oldVersion[versionAt + 16] = '1';
    writeBytes(path, oldVersion);
    rejects([&] { loadPlane(path, attempted); });
    // Matrix corruption is checked against persisted independent sample pairs on load.
    auto corrupt = p;
    corrupt.pixelToBase(0, 2) += 30;
    savePlane(path, corrupt);
    rejects([&] { loadPlane(path, attempted); });
    savePlane(path, p);
    std::string wrongCamera;
    readBytes(path, wrongCamera);
    auto cameraAt = wrongCamera.find(CameraId);
    check(cameraAt != std::string::npos, "Saved camera identity missing");
    wrongCamera.replace(cameraAt, sizeof(CameraId) - 1, "different-camera");
    writeBytes(path, wrongCamera);
    rejects([&] { loadPlane(path, attempted); });
    savePlane(path, p);
}
void targetTests()
{
    auto p = plane();
    auto b = board();
    cv::Point2d t, rejected, projected, movedPixel;
    check(gridTarget(p, b, 3, 4, t), "Target missing");
    transformPoint(BaseAffine, b.intersections[39], projected);
    check(cv::norm(t - projected) < .001, "Wrong target XY");
    check(!gridTarget(p, b, 9, 0, rejected), "Out-of-range column accepted");
    auto unknown = b;
    unknown.orientationValid = false;
    check(!gridTarget(p, unknown, 3, 4, rejected), "Unoriented board generated target");
    auto limited = p;
    limited.validPixelHull = {{20, 20}, {80, 20}, {80, 80}, {20, 80}};
    rejected = t;
    check(!gridTarget(limited, b, 3, 4, rejected), "Calibration extrapolation accepted");
    check(rejected == cv::Point2d(), "Rejected output retained an old target");
    double a = .15, co = std::cos(a), si = std::sin(a);
    cv::Matx23d move(co, -si, 55, si, co, -25);
    std::array<cv::Point2d, 4> corners;
    int j = 0;
    for (int i : {0, 8, 89, 81})
        transformPoint(move, b.intersections[i], corners[j++]);
    Board moved;
    cv::Point2d mt;
    boardFromCorners(corners, true, moved);
    bool movedValid = gridTarget(p, moved, 3, 4, mt);
    transformPoint(move, b.intersections[39], movedPixel);
    transformPoint(BaseAffine, movedPixel, projected);
    check(movedValid && cv::norm(mt - projected) < .001,
          "Moved board incorrectly requires/reuses old robot calibration");
    check(cv::norm(mt - t) > 1, "Moved board target did not change");
    rejects([&] { boardFromCorners({{{100, 100}, {400, 400}, {400, 100}, {100, 400}}}, true, moved); });
    // 明显梯形不满足本轮仿射模型的角点误差要求
    rejects([&] { boardFromCorners({{{100,600},{1100,600},{650,100},{550,100}}}, true, moved); });
}
void assignmentTests()
{
    auto b = board();
    auto center = b.intersections[39];
    Detection d{12, .9, {center.x - 12, center.y - 12, 24, 24}};
    Observation o, hidden, collision, stale, offGrid, stableResult;
    assignPieces(b, "1", {d}, true, o);
    check(o.valid && o.pieces[39] == 12 && o.pieces[0] == Empty, "Grid assignment failed");
    assignPieces(b, "1", {d}, false, hidden);
    check(!hidden.valid && hidden.pieces[0] == Unknown, "Occluded cell treated as empty");
    assignPieces(b, "1", {d, d}, true, collision);
    check(!collision.valid && collision.pieces[39] == Unknown, "Collision silently resolved");
    assignPieces(b, "", {d}, true, stale);
    check(!stale.valid, "Missing observation frame identity accepted");
    cv::Point2d p;
    transformPoint(b.gridToPixel, {3.48, 4}, p);
    d.bbox = {p.x - 12, p.y - 12, 24, 24};
    assignPieces(b, "1", {d}, true, offGrid);
    check(!offGrid.valid, "Off-grid piece accepted");
    Stabilizer stable;
    stable.update(o, stableResult);
    check(!stableResult.valid, "Single frame stable");
    stable.update(o, stableResult);
    check(!stableResult.valid, "Repeated frame counted");
    o.frameId = "2";
    stable.update(o, stableResult);
    check(!stableResult.valid, "Two frames stable");
    o.frameId = "3";
    stable.update(o, stableResult);
    check(stableResult.valid, "Three frames did not stabilize");
    o.frameId = "4";
    stable.reset(); // 重新示教固定棋盘后，重新累计稳定帧。
    stable.update(o, stableResult);
    check(!stableResult.valid, "New fixed layout failed to restart stability");
}
} // namespace
int main()
{
    try
    {
        calibrationTests();
        targetTests();
        assignmentTests();
        std::cout << "PASS " << checks << " geometry checks; no hardware linked.\n";
        return 0;
    }
    catch (const std::exception &e)
    {
        std::cerr << "FAIL after " << checks << " checks: " << e.what() << '\n';
        return 1;
    }
}
