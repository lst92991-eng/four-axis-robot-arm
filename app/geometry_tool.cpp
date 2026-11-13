#include "vision/geometry.hpp"
#include "vision/io.hpp"
#include <opencv2/imgcodecs.hpp>
#include <algorithm>
#include <filesystem>
#include <iostream>
#include <sstream>
#include <cmath>

namespace
{
using namespace scara;
void require(bool v, const char *s)
{
    if (!v) throw std::runtime_error(s);
}
double number(const cv::FileNode &n)
{
    require(n.isReal() || n.isInt(), "A real measured numeric value is required");
    double v = (double)n;
    require(std::isfinite(v), "Fill all measured values; NaN placeholders are not calibration");
    return v;
}
std::string relativePath(const std::string &config, const std::string &value)
{
    require(!value.empty(), "Missing file path");
    auto p = std::filesystem::u8path(value);
    return (p.is_absolute() ? p : std::filesystem::u8path(config).parent_path() / p).lexically_normal().u8string();
}
std::vector<PointPair> pairs(const cv::FileNode &node)
{
    require(node.isSeq(), "fit_points/check_points must be sequences");
    std::vector<PointPair> out;
    for (const auto &n : node)
    {
        auto p = n["pixel"], q = n["base_xy"];
        require(p.size() == 2 && q.size() == 2, "Each pair needs pixel and base_xy");
        out.push_back({{number(p[0]), number(p[1])}, {number(q[0]), number(q[1])}});
    }
    return out;
}
struct ArmTouch
{
    cv::Point2d board;
    double j2Deg, j3MotorDeg;
};

std::vector<ArmTouch> armTouches(const cv::FileNode &node)
{
    std::vector<ArmTouch> out;
    if (node.empty()) return out;
    require(node.isSeq(), "Arm touches must be a list");
    for (const auto &n : node)
    {
        const auto p = n["board_xy_mm"];
        require(p.isSeq() && p.size() == 2, "Arm touch requires board_xy_mm");
        out.push_back({{number(p[0]), number(p[1])}, number(n["j2_deg"]), number(n["j3_motor_deg"])});
    }
    return out;
}

// 标定输入：位置用毫米，读回角用度；待求参数是原点XY和两段臂的参考弧度。
struct ArmSurvey
{
    std::vector<ArmTouch> fit, checks;
    ArmTouch reference;
    double upperMm = 0, forearmMm = 0, j3Ratio = 1, j2Scale = 1;
    int j2Sign = 1, j3Sign = 1;
};

void readArmSurvey(const std::string &path, ArmSurvey &data)
{
    auto file = readStorage(path);
    data.fit = armTouches(file["fit_touches"]);
    data.checks = armTouches(file["check_touches"]);
    require(data.fit.size() >= 3, "Record at least three distributed arm touches");
    auto lengths = file["arm_lengths_mm"];
    require(lengths.isSeq() && lengths.size() == 2, "Two arm lengths are required");
    data.upperMm = number(lengths[0]);
    data.forearmMm = number(lengths[1]);
    data.j3Ratio = number(file["j3_ratio"]);
    data.j2Scale = file["j2_scale"].empty() ? 1 : number(file["j2_scale"]);
    require(data.upperMm > 0 && data.forearmMm > 0 && data.j3Ratio > 0 && data.j2Scale > 0,
            "Arm lengths and transmission ratios must be positive");
    data.reference = data.fit.front();
}

cv::Vec2d armPhase(const ArmSurvey &data, const ArmTouch &point)
{
    double shoulder = data.j2Sign * data.j2Scale * (point.j2Deg - data.reference.j2Deg) * CV_PI / 180;
    double forearm =
        shoulder + data.j3Sign * (point.j3MotorDeg - data.reference.j3MotorDeg) * CV_PI / (180 * data.j3Ratio);
    return {shoulder, forearm};
}

cv::Point2d armPoint(const ArmSurvey &data, const ArmTouch &point, const cv::Vec4d &parameters)
{
    auto phase = armPhase(data, point);
    double shoulder = phase[0] + parameters[2];
    double forearm = phase[1] + parameters[3];
    return {parameters[0] + data.upperMm * std::cos(shoulder) + data.forearmMm * std::cos(forearm),
            parameters[1] + data.upperMm * std::sin(shoulder) + data.forearmMm * std::sin(forearm)};
}

double armCost(const ArmSurvey &data, const cv::Vec4d &parameters)
{
    double squaredMm = 0;
    for (const auto &point : data.fit)
    {
        auto error = armPoint(data, point, parameters) - point.board;
        squaredMm += error.dot(error);
    }
    return squaredMm;
}

bool linearArm(const ArmSurvey &data, cv::Vec4d &parameters)
{
    int rows = 2 * static_cast<int>(data.fit.size());
    cv::Mat design(rows, 6, CV_64F), observation(rows, 1, CV_64F);
    for (int row = 0; row < rows; row += 2)
    {
        const auto &point = data.fit[row / 2];
        auto phase = armPhase(data, point);
        double c1 = std::cos(phase[0]), s1 = std::sin(phase[0]);
        double c2 = std::cos(phase[1]), s2 = std::sin(phase[1]);
        const std::array<double, 6> x{1, 0, c1, -s1, c2, -s2}, y{0, 1, s1, c1, s2, c2};
        std::copy(x.begin(), x.end(), design.ptr<double>(row));
        std::copy(y.begin(), y.end(), design.ptr<double>(row + 1));
        observation.at<double>(row) = point.board.x;
        observation.at<double>(row + 1) = point.board.y;
    }
    cv::Mat singular, linear;
    cv::SVD::compute(design, singular);
    bool independent = singular.at<double>(5) > singular.at<double>(0) * 1e-9;
    bool solved = independent && cv::solve(design, observation, linear, cv::DECOMP_SVD);
    if (solved)
        parameters = {linear.at<double>(0), linear.at<double>(1),
                      std::atan2(linear.at<double>(3), linear.at<double>(2)),
                      std::atan2(linear.at<double>(5), linear.at<double>(4))};
    return solved;
}

void refineArm(const ArmSurvey &data, cv::Vec4d &parameters)
{
    int rows = 2 * static_cast<int>(data.fit.size());
    cv::Mat jacobian(rows, 4, CV_64F), residual(rows, 1, CV_64F);
    int iteration = 0;
    bool refining = true;
    while (refining && iteration < 20)
    {
        for (int row = 0; row < rows; row += 2)
        {
            const auto &point = data.fit[row / 2];
            auto phase = armPhase(data, point);
            double shoulder = phase[0] + parameters[2], forearm = phase[1] + parameters[3];
            const std::array<double, 4> x{1, 0, -data.upperMm * std::sin(shoulder),
                                          -data.forearmMm * std::sin(forearm)};
            const std::array<double, 4> y{0, 1, data.upperMm * std::cos(shoulder), data.forearmMm * std::cos(forearm)};
            std::copy(x.begin(), x.end(), jacobian.ptr<double>(row));
            std::copy(y.begin(), y.end(), jacobian.ptr<double>(row + 1));
            auto error = point.board - armPoint(data, point, parameters);
            residual.at<double>(row) = error.x;
            residual.at<double>(row + 1) = error.y;
        }
        cv::Mat delta;
        refining = cv::solve(jacobian, residual, delta, cv::DECOMP_SVD);
        if (refining)
        {
            cv::Vec4d step(delta.at<double>(0), delta.at<double>(1), delta.at<double>(2), delta.at<double>(3));
            auto next = parameters + step;
            double previousCost = armCost(data, parameters);
            for (int attempt = 0; attempt < 6 && armCost(data, next) > previousCost; ++attempt)
            {
                step *= .5;
                next = parameters + step;
            }
            bool noImprovement = armCost(data, next) >= previousCost;
            refining = noImprovement == false;
            if (refining) parameters = next;
        }
        ++iteration;
    }
}

void writeArmSolution(cv::FileStorage &out, const ArmSurvey &data, const cv::Vec4d &parameters)
{
    out << "{" << "j2_sign" << data.j2Sign << "j3_sign" << data.j3Sign << "j2_origin_board_mm" << "[" << parameters[0]
        << parameters[1] << "]" << "shoulder_ref_rad" << parameters[2] << "elbow_ref_rad"
        << std::remainder(parameters[3] - parameters[2], 2 * CV_PI) << "fit_rms_mm"
        << std::sqrt(armCost(data, parameters) / data.fit.size()) << "fit_errors_mm" << "[";
    for (const auto &point : data.fit)
        out << cv::norm(armPoint(data, point, parameters) - point.board);
    out << "]" << "check_errors_mm" << "[";
    for (const auto &point : data.checks)
        out << cv::norm(armPoint(data, point, parameters) - point.board);
    out << "]" << "}";
}

void registerArm(const std::string &config, const std::string &output)
{
    ArmSurvey data;
    readArmSurvey(config, data);
    auto out = memoryStorage(true);
    out << "j2_reference_deg" << data.reference.j2Deg << "j3_reference_motor_deg" << data.reference.j3MotorDeg
        << "arm_lengths_mm" << "[" << data.upperMm << data.forearmMm << "]" << "j2_scale" << data.j2Scale << "j3_ratio"
        << data.j3Ratio << "solutions" << "[";
    int solutions = 0;
    for (int shoulderSign : {-1, 1})
        for (int elbowSign : {-1, 1})
        {
            data.j2Sign = shoulderSign;
            data.j3Sign = elbowSign;
            cv::Vec4d parameters;
            bool solved = linearArm(data, parameters);
            if (solved)
            {
                refineArm(data, parameters);
                writeArmSolution(out, data, parameters);
                ++solutions;
            }
        }
    require(solutions > 0, "Arm touches do not span enough different poses");
    out << "]";
    writeBytes(output, out.releaseAndGetString());
}
void imageFile(const std::string &path, cv::Mat &image)
{
    auto bytes = readBytes(path);
    std::vector<unsigned char> data(bytes.begin(), bytes.end());
    auto im = cv::imdecode(data, cv::IMREAD_COLOR);
    require(!im.empty(), "Cannot decode image");
    image = im;
}
void saveImage(const std::string &path, const cv::Mat &image)
{
    std::vector<unsigned char> b;
    require(cv::imencode(".png", image, b), "Cannot encode overlay");
    writeBytes(path, {b.begin(), b.end()});
}
void getBoard(const cv::FileStorage &fs, const cv::Mat &image, const Camera &camera, const std::string &frame,
              const std::string &config, Board &board)
{
    board = {};
    std::string mode = (std::string)fs["mode"];
    if (mode == "manual")
    {
        auto n = fs["corners"];
        require(n.isSeq() && n.size() == 4, "Four original-image corners required");
        std::array<cv::Point2d, 4> corners;
        for (int i = 0; i < 4; ++i)
        {
            require(n[i].size() == 2, "Corner must have two coordinates");
            corners[i] = {number(n[i][0]), number(n[i][1])};
        }
        boardFromCorners(camera, frame, corners, (int)fs["canonical_order_confirmed"] == 1, board);
    }
    else
    {
        require(mode == "auto", "mode must be auto or manual");
        LocateOptions options;
        locateBoard(image, camera, frame, options, board);
        std::string orientation = (std::string)fs["orientation"];
        if (board.geometryValid && (orientation == "manual_keep" || orientation == "manual_flip"))
            confirmOrientation(board, orientation == "manual_flip", board);
        else if (board.geometryValid && orientation == "reference")
        {
            OrientationReference reference;
            loadOrientationReference(relativePath(config, (std::string)fs["orientation_reference"]), reference);
            orientByReference(image, camera, board, reference, board);
        }
        else
            require(orientation.empty() || orientation == "unconfirmed" || !board.geometryValid,
                    "Unknown orientation mode");
    }
}
void detectionsFile(const std::string &path, const Camera &camera, const Board &board, bool clear,
                    Observation &observation)
{
    // Accept one JSON object or the last complete JSONL record from the current preview program.
    auto bytes = readBytes(path);
    if (std::filesystem::u8path(path).extension() == ".jsonl")
    {
        std::istringstream s(bytes);
        std::string line, last;
        while (std::getline(s, line))
            if (!line.empty()) last = line;
        bytes = last;
    }
    cv::FileStorage fs(bytes, cv::FileStorage::READ | cv::FileStorage::MEMORY);
    std::string frame = (std::string)fs["frame_id"], space = (std::string)fs["coordinates"];
    ImageTransform t;
    t.sourceSize = camera.imageSize;
    t.roi = {0, 0, camera.imageSize.width, camera.imageSize.height};
    auto source = fs["source_size"];
    if (source.isSeq() && source.size() == 2) t.sourceSize = {(int)source[0], (int)source[1]};
    auto roi = fs["roi"];
    if (roi.isSeq() && roi.size() == 4) t.roi = {(int)roi[0], (int)roi[1], (int)roi[2], (int)roi[3]};
    t.clockwiseDegrees = (int)fs["rotate_clockwise"];
    DetectionSpace kind = DetectionSpace::Original;
    if (space == "network")
    {
        kind = DetectionSpace::Network;
        auto s = fs["resized_size"], p = fs["padding"];
        require(s.size() == 2 && p.size() == 2, "Network detections need resized_size/padding");
        t.resizedSize = {(int)s[0], (int)s[1]};
        t.padding = {number(p[0]), number(p[1])};
    }
    else if (space == "roi" || (space.empty() && roi.isSeq()))
        kind = DetectionSpace::RotatedRoi;
    else
        require(space == "original" || space.empty(), "Unknown detection coordinate system");
    std::vector<Detection> ds;
    auto input = fs["detections"];
    require(input.isSeq(), "detections must be an array");
    for (const auto &d : input)
    {
        auto b = d["bbox"];
        require(b.size() == 4, "bbox must be x1,y1,x2,y2");
        double x = number(b[0]), y = number(b[1]);
        ds.push_back({(int)d["class_id"], number(d["score"]), {x, y, number(b[2]) - x, number(b[3]) - y}});
    }
    AssignmentOptions options;
    assignPieces(camera, board, frame, ds, t, kind, clear, options, observation);
}
void inspect(const std::string &config, const std::string &output)
{
    auto fs = readStorage(config);
    Camera camera;
    readCamera(fs["camera"], camera);
    std::string frame = (std::string)fs["frame_id"];
    cv::Mat image;
    imageFile(relativePath(config, (std::string)fs["image"]), image);
    require(image.size() == camera.imageSize, "Image size differs from camera profile");
    Board board;
    PlaneMap plane;
    Observation observation;
    getBoard(fs, image, camera, frame, config, board);
    if (!fs["plane_file"].empty() && !((std::string)fs["plane_file"]).empty())
        loadPlane(relativePath(config, (std::string)fs["plane_file"]), plane);
    bool hasObservation = !fs["detections_file"].empty() && !((std::string)fs["detections_file"]).empty();
    if (hasObservation)
        detectionsFile(relativePath(config, (std::string)fs["detections_file"]), camera, board,
                       (int)fs["scene_clear"] == 1, observation);
    auto result = memoryStorage(true);
    result << "motion_enabled" << 0 << "single_frame_only" << 1 << "camera_id" << camera.id << "frame_id" << frame
           << "geometry_valid" << (int)board.geometryValid << "orientation_valid" << (int)board.orientationValid
           << "orientation_source" << board.orientationSource << "grid_score" << board.gridScore << "points" << "[";
    if (board.geometryValid)
        for (int r = 0; r < Rows; ++r)
            for (int c = 0; c < Columns; ++c)
            {
                auto p = board.intersections[r * Columns + c];
                result << "{" << "column" << c << "row" << r << "pixel_geometric" << "[" << p.x << p.y << "]";
                TargetXY target;
                bool targetValid = gridTarget(plane, board, c, r, frame, target);
                result << "base_xy_valid" << (int)targetValid;
                if (targetValid)
                    result << "base_xy_mm" << "[" << target.baseMm.x << target.baseMm.y << "]" << "plane_z_mm"
                           << target.planeZmm;
                if (hasObservation) result << "piece_id" << observation.pieces[r * Columns + c];
                result << "}";
            }
    result << "]";
    if (hasObservation) result << "observation_valid" << (int)observation.valid << "issues" << observation.issues;
    writeBytes(output, result.releaseAndGetString());
    cv::Mat geometric, overlay, rectified;
    camera.geometricImage(image, geometric);
    drawBoard(geometric, board, hasObservation ? &observation : nullptr, overlay);
    saveImage(output + ".png", overlay);
    if (board.geometryValid)
    {
        rectifiedBoard(geometric, board, rectified);
        saveImage(output + ".rectified.png", rectified);
    }
    std::cout << "geometry=" << board.geometryValid << " orientation=" << board.orientationValid
              << " score=" << board.gridScore << "\nWrote " << output << " and overlay. No motion.\n";
}
} // namespace
int runGeometryTool(int argc, char **argv)
{
    try
    {
        if (argc != 4)
        {
            std::cout << "Geometry only (C++17 / OpenCV " << CV_VERSION << ")\n"
                      << "  scara_geometry_tool calibrate measured-pairs.yml plane-map.yml\n"
                      << "  scara_geometry_tool inspect inspect.yml result.json\n"
                      << "  scara_geometry_tool register-arm touches.json result.json\n"
                      << "  scara_geometry_tool reference manual-inspect.yml orientation.yml\n";
            return argc == 1 ? 0 : 1;
        }
        const std::string command = argv[1], input = argv[2], output = argv[3];
        if (command == "calibrate")
        {
            auto fs = scara::readStorage(input);
            scara::Camera camera;
            scara::PlaneMap map;
            std::string id = (std::string)fs["calibration_id"];
            double planeZmm = number(fs["plane_z_mm"]);
            double allowedErrorMm = number(fs["allowed_error_mm"]);
            std::vector<scara::PointPair> fit = pairs(fs["fit_points"]);
            std::vector<scara::PointPair> checks = pairs(fs["check_points"]);
            scara::readCamera(fs["camera"], camera);
            scara::calibratePlane(camera, id, planeZmm, allowedErrorMm, fit, checks, map);
            std::cout << "fit RMS/max mm: " << map.fitError.rmsMm << "/" << map.fitError.maxMm
                      << "\ncheck RMS/max mm: " << map.checkError.rmsMm << "/" << map.checkError.maxMm << '\n';
            for (size_t i = 0; i < map.checkError.errorsMm.size(); ++i)
                std::cout << "check[" << i << "] error mm: " << map.checkError.errorsMm[i] << '\n';
            if (!map.accepted)
            {
                std::cerr << "REJECTED: errors exceed tolerance; no usable map saved.\n";
                return 2;
            }
            scara::savePlane(output, map);
        }
        else if (command == "inspect")
            inspect(input, output);
        else if (command == "register-arm")
            registerArm(input, output);
        else if (command == "reference")
        {
            auto fs = scara::readStorage(input);
            scara::Camera camera;
            scara::readCamera(fs["camera"], camera);
            cv::Mat image;
            Board board;
            imageFile(relativePath(input, (std::string)fs["image"]), image);
            getBoard(fs, image, camera, (std::string)fs["frame_id"], input, board);
            auto n = fs["reference_patch"];
            require(n.isSeq() && n.size() == 4, "reference_patch [x,y,w,h] required in 432x480 rectified board");
            scara::OrientationReference ref;
            cv::Rect patch((int)n[0], (int)n[1], (int)n[2], (int)n[3]);
            scara::makeOrientationReference(image, camera, board, patch, ref);
            scara::saveOrientationReference(output, ref);
        }
        else
            throw std::runtime_error("Unknown command");
        return 0;
    }
    catch (const std::exception &e)
    {
        std::cerr << "ERROR: " << e.what() << '\n';
        return 1;
    }
}

#ifdef _WIN32
// Windows supplies UTF-16 arguments. Convert them explicitly; /utf-8 alone does not change argv.
int wmain(int argc, wchar_t **argv)
{
    std::vector<std::string> utf8;
    utf8.reserve(argc);
    for (int i = 0; i < argc; ++i)
        utf8.push_back(std::filesystem::path(argv[i]).u8string());
    std::vector<char *> args;
    for (auto &value : utf8)
        args.push_back(value.data());
    return runGeometryTool(argc, args.data());
}
#else
int main(int argc, char **argv)
{
    return runGeometryTool(argc, argv);
}
#endif
