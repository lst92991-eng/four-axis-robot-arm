#include "geometry.hpp"
#include "io.hpp"
#include <opencv2/geometry.hpp>
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <optional>

namespace scara
{
namespace
{
constexpr int RectWidth = 432, RectHeight = 480, Step = 48, Margin = 24;
void require(bool value, const char *message)
{
    if (!value) throw std::runtime_error(message);
}
void gray(const cv::Mat &image, cv::Mat &grayscale)
{
    if (image.channels() == 1)
        grayscale = image;
    else
        cv::cvtColor(image, grayscale, cv::COLOR_BGR2GRAY);
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
struct AxisFit
{
    double start = 0, step = 0, score = -1, support = 0;
};
AxisFit fitAxis(const cv::Mat &projection, int count)
{
    std::vector<float> v((size_t)projection.total());
    std::copy(projection.ptr<float>(), projection.ptr<float>() + v.size(), v.begin());
    const int length = (int)v.size();
    const float maximum = *std::max_element(v.begin(), v.end());
    AxisFit best;
    if (maximum <= 0) return best;
    auto peak = [&](double p)
    {
        int at = (int)std::lround(p), where = std::clamp(at, 0, length - 1);
        float value = 0;
        for (int j = std::max(0, at - 2); j <= std::min(length - 1, at + 2); ++j)
            if (v[j] > value)
            {
                value = v[j];
                where = j;
            }
        return std::pair<int, float>(where, value / maximum);
    };
    for (double step = 0.60 * (length - 1) / (count - 1); step <= double(length - 1) / (count - 1); step += 0.5)
    {
        for (double start = 0; start + (count - 1) * step <= length - 1; start += 1.0)
        {
            double score = 0, hits = 0;
            for (int i = 0; i < count; ++i)
            {
                const auto value = peak(start + i * step).second;
                score += value;
                hits += value >= 0.20f;
            }
            score = 0.5 * (score / count + hits / count);
            if (score > best.score) best = {start, step, score, hits / count};
        }
    }
    // Refine the equally spaced grid using supported line peaks, not board-border corners.
    double n = 0, si = 0, sy = 0, sii = 0, siy = 0;
    for (int i = 0; i < count; ++i)
    {
        auto p = peak(best.start + i * best.step);
        if (p.second < 0.20f) continue;
        ++n;
        si += i;
        sy += p.first;
        sii += i * i;
        siy += i * p.first;
    }
    if (n >= 3 && n * sii - si * si > 0)
    {
        best.step = (n * siy - si * sy) / (n * sii - si * si);
        best.start = (sy - best.step * si) / n;
    }
    return best;
}
std::optional<cv::Vec3d> localLine(const cv::Mat &mask, double position, bool vertical)
{
    std::vector<cv::Point2f> points;
    const int along = vertical ? mask.rows : mask.cols, across = vertical ? mask.cols : mask.rows;
    for (int a = 5; a < along - 5; ++a)
    {
        double sum = 0, count = 0;
        for (int b = std::max(0, (int)std::lround(position) - 4);
             b <= std::min(across - 1, (int)std::lround(position) + 4); ++b)
            if (mask.at<unsigned char>(vertical ? a : b, vertical ? b : a))
            {
                sum += b;
                ++count;
            }
        if (count)
            points.push_back(vertical ? cv::Point2f((float)(sum / count), (float)a)
                                      : cv::Point2f((float)a, (float)(sum / count)));
    }
    if (points.size() < along * .20 || cv::norm(points.back() - points.front()) < along * .5) return std::nullopt;
    cv::Vec4f line;
    cv::fitLine(points, line, cv::DIST_HUBER, 0, .01, .01);
    if ((vertical && std::abs(line[0]) > .15) || (!vertical && std::abs(line[1]) > .15)) return std::nullopt;
    return cv::Vec3d(-line[1], line[0], line[1] * line[2] - line[0] * line[3]);
}
cv::Matx33d refineGrid(const cv::Mat &vertical, const cv::Mat &horizontal, const AxisFit &x, const AxisFit &y)
{
    cv::Matx33d regular(x.step, 0, x.start, 0, y.step, y.start, 0, 0, 1);
    std::array<std::optional<cv::Vec3d>, Columns> cols;
    std::array<std::optional<cv::Vec3d>, Rows> rows;
    for (int c = 0; c < Columns; ++c)
        cols[c] = localLine(vertical, x.start + c * x.step, true);
    for (int r = 0; r < Rows; ++r)
        rows[r] = localLine(horizontal, y.start + r * y.step, false);
    std::vector<cv::Point2f> grid, observed;
    for (int r = 0; r < Rows; ++r)
        for (int c = 0; c < Columns; ++c)
            if (cols[c] && rows[r])
            {
                auto p = cols[c]->cross(*rows[r]);
                if (std::abs(p[2]) < 1e-8) continue;
                cv::Point2d q(p[0] / p[2], p[1] / p[2]);
                if (cv::norm(q - project(regular, {(double)c, (double)r})) > 7) continue;
                grid.emplace_back((float)c, (float)r);
                observed.push_back(q);
            }
    if (grid.size() < 20) return regular;
    cv::Mat h = cv::findHomography(grid, observed, 0);
    if (h.empty()) return regular;
    cv::Matx33d refined;
    std::copy(h.ptr<double>(), h.ptr<double>() + 9, refined.val);
    for (auto p : std::vector<cv::Point2d>{{0, 0}, {8, 0}, {8, 9}, {0, 9}})
        if (cv::norm(project(refined, p) - project(regular, p)) > 8) return regular;
    return refined;
}

// Score actual segments inside the proposed lattice, not projections over the
// whole candidate image. The river and palace diagonals disambiguate a grid
// translated by one rank, and distinguish the 9-file and 10-rank axes.
double structureScore(const cv::Mat &binary, const cv::Mat &vertical, const cv::Mat &horizontal,
                      const cv::Matx33d &gridToRect)
{
    auto hit = [&](const cv::Mat &mask, cv::Point2d grid)
    {
        auto p = project(gridToRect, grid);
        int x = (int)std::lround(p.x), y = (int)std::lround(p.y);
        if (x < 2 || y < 2 || x >= mask.cols - 2 || y >= mask.rows - 2) return 0.0;
        for (int dy = -2; dy <= 2; ++dy)
            for (int dx = -2; dx <= 2; ++dx)
                if (mask.at<unsigned char>(y + dy, x + dx)) return 1.0;
        return 0.0;
    };
    double support = 0, edges = 0, river = 0, palace = 0;
    int n = 0, ne = 0, nr = 0, np = 0;
    for (int r = 0; r < Rows; ++r)
        for (int c = 0; c < Columns - 1; ++c)
            for (double t : {.35, .5, .65})
            {
                double s = hit(horizontal, {c + t, (double)r});
                support += s;
                ++n;
                if (r == 0 || r == Rows - 1)
                {
                    edges += s;
                    ++ne;
                }
            }
    for (int c = 0; c < Columns; ++c)
        for (int r = 0; r < Rows - 1; ++r)
            for (double t : {.35, .5, .65})
            {
                double s = hit(vertical, {(double)c, r + t});
                if (r == 4 && c > 0 && c < Columns - 1)
                {
                    river += s;
                    ++nr;
                    continue;
                }
                support += s;
                ++n;
                if (c == 0 || c == Columns - 1)
                {
                    edges += s;
                    ++ne;
                }
            }
    for (int base : {0, 7})
        for (double t : {.35, .5, .65, 1.35, 1.5, 1.65})
        {
            palace += hit(binary, {3 + t, base + t}) + hit(binary, {5 - t, base + t});
            np += 2;
        }
    return .55 * support / n + .15 * edges / ne + .20 * palace / np + .10 * (1 - river / nr);
}
// 格线的主方向，角度用弧度；余弦/正弦用于投影边界点。
struct LineDirection
{
    double radians = 0;
    double cosine = 1;
    double sine = 0;
    bool found = false;
};

void addQuad(std::vector<cv::Point2f> q, std::vector<std::vector<cv::Point2f>> &quads)
{
    cv::Point2f center;
    for (auto v : q)
        center += v;
    center *= .25f;
    std::sort(q.begin(), q.end(), [&](auto a, auto b)
              { return std::atan2(a.y - center.y, a.x - center.x) < std::atan2(b.y - center.y, b.x - center.x); });
    auto first = std::min_element(q.begin(), q.end(), [](auto a, auto b) { return a.x + a.y < b.x + b.y; });
    std::rotate(q.begin(), first, q.end());
    std::reverse(q.begin() + 1, q.end());
    quads.push_back(q);
}

void lineDirection(const std::vector<cv::Vec4i> &lines, LineDirection &axis)
{
    // 按线段长度投票；方向按90度折叠，横竖格线落在同一组。
    std::array<double, 90> votes{};
    std::vector<std::pair<double, double>> angles;
    for (auto l : lines)
    {
        double dx = l[2] - l[0], dy = l[3] - l[1], a = std::fmod(std::atan2(dy, dx) * 180 / CV_PI + 180, 90),
               weight = std::hypot(dx, dy);
        angles.emplace_back(a, weight);
        int bin = (int)std::lround(a) % 90;
        for (int d = -2; d <= 2; ++d)
            votes[(bin + d + 90) % 90] += weight;
    }
    int peak = (int)std::distance(votes.begin(), std::max_element(votes.begin(), votes.end()));
    double sum = 0, weight = 0;
    for (auto a : angles)
    {
        double delta = std::remainder(a.first - peak, 90);
        if (std::abs(delta) <= 4)
        {
            sum += (peak + delta) * a.second;
            weight += a.second;
        }
    }
    axis.radians = weight ? sum / weight * CV_PI / 180 : 0;
    axis.cosine = std::cos(axis.radians);
    axis.sine = std::sin(axis.radians);
    axis.found = weight > 0;
}

void lineQuads(const std::vector<cv::Vec4i> &lines, const LineDirection &axis,
               std::vector<std::vector<cv::Point2f>> &quads)
{
    if (!axis.found) return;
    double theta = axis.radians, co = axis.cosine, si = axis.sine;
    // 把长线投到主方向上，用两端位置组成候选四边形。
    std::vector<double> u, v;
    for (auto l : lines)
    {
        double a = std::fmod(std::atan2(double(l[3] - l[1]), double(l[2] - l[0])) * 180 / CV_PI + 180, 90);
        if (std::abs(std::remainder(a - theta * 180 / CV_PI, 90)) > 5) continue;
        for (int k : {0, 2})
        {
            u.push_back(co * l[k] + si * l[k + 1]);
            v.push_back(-si * l[k] + co * l[k + 1]);
        }
    }
    std::sort(u.begin(), u.end());
    std::sort(v.begin(), v.end());
    if (u.size() >= 16)
        for (double trim : {.05, .10, .15})
        {
            size_t lo = (size_t)(trim * (u.size() - 1)), hi = (size_t)((1 - trim) * (u.size() - 1));
            double padU = (u[hi] - u[lo]) * .04, padV = (v[hi] - v[lo]) * .04;
            auto point = [&](double x, double y)
            { return cv::Point2f((float)(co * x - si * y), (float)(si * x + co * y)); };
            addQuad({point(u[lo] - padU, v[lo] - padV), point(u[hi] + padU, v[lo] - padV),
                     point(u[hi] + padU, v[hi] + padV), point(u[lo] - padU, v[hi] + padV)},
                    quads);
        }
}

void contourQuads(const std::vector<std::vector<cv::Point>> &contours, size_t pixelCount, const LocateOptions &options,
                  const LineDirection &axis, std::vector<std::vector<cv::Point2f>> &quads)
{
    // 闭合轮廓直接提供四角；不规则轮廓用外接框提出候选。
    double co = axis.cosine, si = axis.sine;
    for (const auto &c : contours)
    {
        double area = std::abs(cv::contourArea(c));
        if (area < pixelCount * options.minAreaFraction || area > pixelCount * 0.99) continue;
        std::vector<cv::Point> p;
        cv::approxPolyDP(c, p, 0.02 * cv::arcLength(c, true), true);
        std::vector<cv::Point2f> q;
        if (p.size() == 4 && cv::isContourConvex(p))
            q.assign(p.begin(), p.end());
        else
        {
            // Cables/occlusions can join or break the outer contour. This is only a proposal;
            // acceptance still requires the internal 9-by-10 lattice and line refinement.
            auto box = cv::minAreaRect(c);
            if (std::min(box.size.width, box.size.height) < 80) continue;
            cv::Point2f corners[4];
            box.points(corners);
            q.assign(corners, corners + 4);
        }
        // Red-view column-right / rank-away is counterclockwise in a normal image's down-positive Y.
        // This fixes chirality before resolving the remaining 180-degree end ambiguity.
        addQuad(q, quads);
        if (axis.found && p.size() != 4)
        {
            double u0 = 1e9, u1 = -1e9, v0 = 1e9, v1 = -1e9;
            for (auto point : c)
            {
                double u = co * point.x + si * point.y, v = -si * point.x + co * point.y;
                u0 = std::min(u0, u);
                u1 = std::max(u1, u);
                v0 = std::min(v0, v);
                v1 = std::max(v1, v);
            }
            auto point = [&](double u, double v)
            { return cv::Point2f((float)(co * u - si * v), (float)(si * u + co * v)); };
            addQuad({point(u0, v0), point(u1, v0), point(u1, v1), point(u0, v1)}, quads);
        }
    }
}

void boardEdges(const cv::Mat &grayscale, cv::Mat &edges)
{
    cv::Mat blur;
    cv::GaussianBlur(grayscale, blur, {5, 5}, 1);
    cv::Canny(blur, edges, 40, 120);
    cv::morphologyEx(edges, edges, cv::MORPH_CLOSE, cv::getStructuringElement(cv::MORPH_RECT, {3, 3}));
}

void borderCandidates(const cv::Mat &edges, const LocateOptions &options, std::vector<std::vector<cv::Point2f>> &quads)
{
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> lines;
    LineDirection axis;
    quads.clear();
    cv::findContours(edges, contours, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);
    cv::HoughLinesP(edges, lines, 1, CV_PI / 180, 80, std::min(edges.cols, edges.rows) * .28, 35);
    lineDirection(lines, axis);
    lineQuads(lines, axis, quads);
    contourQuads(contours, edges.total(), options, axis, quads);
    std::sort(quads.begin(), quads.end(),
              [](const auto &a, const auto &b) { return cv::contourArea(a) > cv::contourArea(b); });
    if (quads.size() > 24) quads.resize(24);
}

void chooseGrid(const cv::Mat &g, const Camera &camera, const std::string &frame,
                const std::vector<std::vector<cv::Point2f>> &quads, const LocateOptions &options, Board &board)
{
    // 每个候选旋到四种方向，依据格线、河界和九宫选择得分最高的网格。
    Board best;
    best.cameraId = camera.id;
    best.frameId = frame;
    best.orientationSource = "natural_grid_unoriented";
    const cv::Size size(480, 528);
    const std::vector<cv::Point2f> rect{{0, 0}, {479, 0}, {479, 527}, {0, 527}};
    for (const auto &quad : quads)
        for (int rotation = 0; rotation < 4; ++rotation)
        {
            std::vector<cv::Point2f> q;
            for (int i = 0; i < 4; ++i)
                q.push_back(quad[(i + rotation) % 4]);
            const auto pixelToRect = homography(q, rect);
            cv::Mat warped, binary, vertical, horizontal, px, py;
            cv::warpPerspective(g, warped, pixelToRect, size);
            cv::adaptiveThreshold(warped, binary, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY_INV, 31, 8);
            cv::morphologyEx(binary, vertical, cv::MORPH_OPEN, cv::getStructuringElement(cv::MORPH_RECT, {1, 25}));
            cv::morphologyEx(binary, horizontal, cv::MORPH_OPEN, cv::getStructuringElement(cv::MORPH_RECT, {25, 1}));
            cv::reduce(vertical, px, 0, cv::REDUCE_SUM, CV_32F);
            cv::reduce(horizontal, py, 1, cv::REDUCE_SUM, CV_32F);
            const auto x = fitAxis(px, Columns), y = fitAxis(py, Rows);
            const double score = (x.score + y.score) * 0.5;
            if (x.support < options.minLineSupport || y.support < options.minLineSupport ||
                score < options.minGridScore)
                continue;
            for (int dx : {-1, 0, 1})
                for (int dy : {-1, 0, 1})
                    try
                    {
                        auto sx = x, sy = y;
                        sx.start += dx * x.step;
                        sy.start += dy * y.step;
                        auto gridToRect = refineGrid(vertical, horizontal, sx, sy);
                        const double structural = structureScore(binary, vertical, horizontal, gridToRect);
                        if (structural < options.minGridScore || structural <= best.gridScore) continue;
                        auto candidate = boardWithMatrix(camera, frame, pixelToRect.inv() * gridToRect);
                        candidate.gridScore = structural;
                        candidate.orientationSource = "natural_grid_unoriented";
                        best = candidate;
                    }
                    catch (const std::exception &)
                    { /* Reject bad extrapolation; try next natural-border candidate. */
                    }
        }
    board = best;
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
void locateBoard(const cv::Mat &raw, const Camera &camera, const std::string &frame, const LocateOptions &options,
                 Board &board)
{
    require(options.minAreaFraction > 0 && options.minAreaFraction < 1 && options.minLineSupport > 0 &&
                options.minLineSupport <= 1,
            "Invalid localization options");
    cv::Mat image;     // 相机几何校正后的原尺寸图。
    cv::Mat grayscale; // 灰度图，用来寻找边框和格线。
    cv::Mat edges;     // 边缘图，用来产生棋盘四边形候选。
    std::vector<std::vector<cv::Point2f>> quads;

    camera.geometricImage(raw, image);
    gray(image, grayscale);
    boardEdges(grayscale, edges);
    borderCandidates(edges, options, quads);
    chooseGrid(grayscale, camera, frame, quads, options, board);
}
void confirmOrientation(const Board &input, bool flip, Board &board)
{
    require(input.geometryValid, "Cannot orient invalid grid");
    auto b = input;
    if (flip) b.gridToPixel = input.gridToPixel * cv::Matx33d(-1, 0, 8, 0, -1, 9, 0, 0, 1);
    for (int r = 0; r < Rows; ++r)
        for (int c = 0; c < Columns; ++c)
            b.intersections[r * Columns + c] = project(b.gridToPixel, {(double)c, (double)r});
    b.orientationValid = true;
    b.orientationSource = "manual_end_confirmation";
    board = b;
}
void rectifiedBoard(const cv::Mat &image, const Board &b, cv::Mat &result)
{
    require(b.geometryValid, "Grid required for rectification");
    // Human-facing rectification keeps red at the bottom, without mirroring Chinese glyphs.
    cv::Matx33d gridToRect(Step, 0, Margin, 0, -Step, Margin + 9 * Step, 0, 0, 1);
    cv::warpPerspective(image, result, gridToRect * b.gridToPixel.inv(), {RectWidth, RectHeight});
}
void makeOrientationReference(const cv::Mat &raw, const Camera &camera, const Board &b, cv::Rect patch,
                              OrientationReference &reference)
{
    require(b.orientationValid && b.cameraId == camera.id && camera.sameGeometry(b.geometryCamera),
            "Confirm canonical board orientation with matching camera profile first");
    require(patch.width >= 12 && patch.height >= 12 && (patch & cv::Rect(0, 0, RectWidth, RectHeight)) == patch,
            "Invalid reference patch");
    cv::Mat image;
    camera.geometricImage(raw, image);
    cv::Mat rectified;
    rectifiedBoard(image, b, rectified);
    cv::Mat canonical;
    gray(rectified, canonical);
    cv::Scalar mean, stddev;
    cv::meanStdDev(canonical(patch), mean, stddev);
    require(stddev[0] >= 8, "Reference patch lacks distinctive texture");
    OrientationReference ref;
    ref.cameraId = camera.id;
    ref.patchRect = patch;
    ref.patch = canonical(patch).clone();
    reference = ref;
}
void orientByReference(const cv::Mat &raw, const Camera &camera, const Board &input, const OrientationReference &ref,
                       Board &board)
{
    require(input.geometryValid && input.cameraId == camera.id && camera.sameGeometry(input.geometryCamera) &&
                ref.cameraId == camera.id,
            "Orientation reference camera mismatch");
    require(ref.minCorrelation > 0 && ref.minCorrelation <= 1 && ref.minMargin > 0 && ref.minMargin <= 2,
            "Invalid orientation thresholds");
    require(!ref.patch.empty() && ref.patch.size() == ref.patchRect.size() &&
                (ref.patchRect & cv::Rect(0, 0, RectWidth, RectHeight)) == ref.patchRect,
            "Invalid orientation reference");
    cv::Mat image;
    camera.geometricImage(raw, image);
    std::array<double, 2> scores{};
    for (int flip = 0; flip < 2; ++flip)
    {
        Board b;
        cv::Mat rectified;
        confirmOrientation(input, flip != 0, b);
        rectifiedBoard(image, b, rectified);
        cv::Mat rect;
        gray(rectified, rect);
        cv::Rect search(ref.patchRect.x - 4, ref.patchRect.y - 4, ref.patchRect.width + 8, ref.patchRect.height + 8);
        search &= cv::Rect(0, 0, RectWidth, RectHeight);
        cv::Mat score;
        cv::matchTemplate(rect(search), ref.patch, score, cv::TM_CCOEFF_NORMED);
        cv::minMaxLoc(score, nullptr, &scores[flip]);
    }
    int winner = scores[1] > scores[0] ? 1 : 0;
    if (!std::isfinite(scores[winner]) || scores[winner] < ref.minCorrelation ||
        scores[winner] - scores[1 - winner] < ref.minMargin)
    {
        auto b = input;
        b.orientationValid = false;
        b.orientationSource = "ambiguous_printed_feature";
        board = b;
        return;
    }
    Board b;
    confirmOrientation(input, winner != 0, b);
    b.orientationSource = "printed_board_feature";
    board = b;
}
void saveOrientationReference(const std::string &path, const OrientationReference &r)
{
    auto fs = memoryStorage();
    fs << "camera_id" << r.cameraId << "rect" << "[" << r.patchRect.x << r.patchRect.y << r.patchRect.width
       << r.patchRect.height << "]"
       << "patch" << r.patch << "min_correlation" << r.minCorrelation << "min_margin" << r.minMargin;
    writeBytes(path, fs.releaseAndGetString());
}
void loadOrientationReference(const std::string &path, OrientationReference &reference)
{
    auto fs = readStorage(path);
    OrientationReference r;
    fs["camera_id"] >> r.cameraId;
    fs["patch"] >> r.patch;
    auto p = fs["rect"];
    require(p.isSeq() && p.size() == 4, "Invalid patch rectangle");
    r.patchRect = {(int)p[0], (int)p[1], (int)p[2], (int)p[3]};
    r.minCorrelation = (double)fs["min_correlation"];
    r.minMargin = (double)fs["min_margin"];
    require(r.minCorrelation > 0 && r.minCorrelation <= 1 && r.minMargin > 0 && r.minMargin <= 2,
            "Invalid orientation thresholds");
    reference = r;
}
} // namespace scara
