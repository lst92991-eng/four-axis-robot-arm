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
Camera camera()
{
    Camera c;
    c.id = "synthetic-test-only";
    c.imageSize = {640, 480};
    return c;
}
const cv::Matx33d BaseH(0.4, 0.02, 100, -0.01, 0.5, -80, 0.0001, -0.00005, 1);
PlaneMap plane()
{
    std::vector<PointPair> fit, validation;
    for (auto p : std::vector<cv::Point2d>{{20, 20}, {620, 20}, {620, 460}, {20, 460}, {320, 70}, {180, 330}})
        fit.push_back({p, project(BaseH, p)});
    for (auto p : std::vector<cv::Point2d>{{150, 180}, {320, 260}, {450, 300}})
        validation.push_back({p, project(BaseH, p)});
    PlaneMap result;
    calibratePlane(camera(), "synthetic-map-do-not-use-on-robot", 12, 0.001, fit, validation, result);
    return result;
}
Board board(const std::string &frame = "1")
{
    Board result;
    boardFromCorners(camera(), frame, {{{150, 80}, {500, 100}, {480, 400}, {130, 380}}}, true, result);
    return result;
}
void calibrationTests()
{
    auto p = plane();
    check(p.accepted && p.checkError.maxMm < 0.001, "Exact planar fit failed");
    auto path = (std::filesystem::temp_directory_path() / std::filesystem::u8path("scara_几何测试.yml")).u8string();
    savePlane(path, p);
    PlaneMap loaded;
    loadPlane(path, loaded);
    check(loaded.id == p.id && loaded.accepted, "Persistence failed");
    auto bad = p.checkPoints;
    bad[0].baseXY.x += 10;
    PlaneMap attempted;
    calibratePlane(p.camera, p.id, p.planeZmm, p.allowedErrorMm, p.fitPoints, bad, attempted);
    check(!attempted.accepted, "Validation error not rejected");
    rejects(
        [&]
        {
            auto v = p.checkPoints;
            v[0] = p.fitPoints[0];
            calibratePlane(p.camera, p.id, 0, 1, p.fitPoints, v, attempted);
        });
    rejects(
        [&]
        {
            std::vector<PointPair> line;
            for (int i = 0; i < 4; ++i)
                line.push_back({{50. + i * 50, 100}, {double(i), 0}});
            calibratePlane(p.camera, p.id, 0, 1, line, p.checkPoints, attempted);
        });
    auto altered = readBytes(path);
    auto at = altered.find("pixel_to_base:");
    check(at != std::string::npos, "Missing saved matrix");
    // Matrix corruption is checked against persisted independent sample pairs on load.
    auto corrupt = p;
    corrupt.pixelToBase(0, 2) += 30;
    savePlane(path, corrupt);
    rejects([&] { loadPlane(path, attempted); });
    savePlane(path, p);
}
void targetTests()
{
    auto p = plane();
    auto b = board();
    TargetXY t, rejected;
    check(gridTarget(p, b, 3, 4, "1", t), "Target missing");
    check(cv::norm(t.baseMm - project(BaseH, b.intersections[39])) < .001, "Wrong target XY");
    check(!gridTarget(p, b, 9, 0, "1", rejected) && !gridTarget(p, b, 0, 0, "old", rejected),
          "Bounds/stale frame accepted");
    auto unknown = b;
    unknown.orientationValid = false;
    check(!gridTarget(p, unknown, 3, 4, "1", rejected), "Unoriented board generated target");
    auto wrong = b;
    wrong.cameraId = "different-camera";
    check(!gridTarget(p, wrong, 3, 4, "1", rejected), "Different camera accepted");
    wrong = b;
    wrong.geometryCamera.imageSize.width += 1;
    check(!gridTarget(p, wrong, 3, 4, "1", rejected), "Changed numeric camera profile accepted under same ID");
    auto limited = p;
    limited.validPixelHull = {{20, 20}, {80, 20}, {80, 80}, {20, 80}};
    rejected = t;
    check(!gridTarget(limited, b, 3, 4, "1", rejected), "Calibration extrapolation accepted");
    check(rejected.frameId.empty() && rejected.calibrationId.empty(), "Rejected output retained an old target");
    double a = .15, co = std::cos(a), si = std::sin(a);
    cv::Matx33d move(co, -si, 55, si, co, -25, 0, 0, 1);
    std::array<cv::Point2d, 4> corners;
    int j = 0;
    for (int i : {0, 8, 89, 81})
        corners[j++] = project(move, b.intersections[i]);
    Board moved;
    TargetXY mt;
    boardFromCorners(camera(), "2", corners, true, moved);
    bool movedValid = gridTarget(p, moved, 3, 4, "2", mt);
    check(movedValid && cv::norm(mt.baseMm - project(BaseH, project(move, b.intersections[39]))) < .001,
          "Moved board incorrectly requires/reuses old robot calibration");
    check(cv::norm(mt.baseMm - t.baseMm) > 1, "Moved board target did not change");
    rejects([&] { boardFromCorners(camera(), "x", {{{100, 100}, {400, 400}, {400, 100}, {100, 400}}}, true, moved); });
}
void assignmentTests()
{
    auto b = board();
    auto center = b.intersections[39];
    Detection d{12, .9, {center.x - 12, center.y - 12, 24, 24}};
    Observation o, hidden, collision, stale, offGrid, stableResult;
    assignPieces(camera(), b, "1", {d}, true, o);
    check(o.valid && o.pieces[39] == 12 && o.pieces[0] == Empty, "Grid assignment failed");
    assignPieces(camera(), b, "1", {d}, false, hidden);
    check(!hidden.valid && hidden.pieces[0] == Unknown, "Occluded cell treated as empty");
    assignPieces(camera(), b, "1", {d, d}, true, collision);
    check(!collision.valid && collision.pieces[39] == Unknown, "Collision silently resolved");
    assignPieces(camera(), b, "wrong", {d}, true, stale);
    check(!stale.valid, "Mixed frames accepted");
    auto p = project(b.gridToPixel, {3.48, 4});
    d.bbox = {p.x - 12, p.y - 12, 24, 24};
    assignPieces(camera(), b, "1", {d}, true, offGrid);
    check(!offGrid.valid, "Off-grid piece accepted");
    Stabilizer stable(3);
    stable.update(b, o, stableResult);
    check(!stableResult.valid, "Single frame stable");
    stable.update(b, o, stableResult);
    check(!stableResult.valid, "Repeated frame counted");
    b.frameId = o.frameId = "2";
    stable.update(b, o, stableResult);
    check(!stableResult.valid, "Two frames stable");
    b.frameId = o.frameId = "3";
    stable.update(b, o, stableResult);
    check(stableResult.valid, "Three frames did not stabilize");
    b.frameId = o.frameId = "4";
    b.intersections[0].x += 10;
    stable.update(b, o, stableResult);
    check(!stableResult.valid, "Board motion failed to reset stability");
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
