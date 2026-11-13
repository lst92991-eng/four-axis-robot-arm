#include "vision/geometry.hpp"
#include "vision/io.hpp"
#include <opencv2/imgcodecs.hpp>
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
// 四角依次为红方左下、右下、黑方右上、左上；方向由首次示教确认。
void getBoard(const cv::FileStorage &file, const Camera &camera, const std::string &frame, Board &board)
{
    require((std::string)file["mode"] == "manual", "Fixed board requires saved manual corners");
    auto points = file["corners"];
    require(points.isSeq() && points.size() == 4, "Four original-image corners required");
    std::array<cv::Point2d, 4> corners;
    for (int i = 0; i < 4; ++i)
    {
        require(points[i].size() == 2, "Corner must have two coordinates");
        corners[i] = {number(points[i][0]), number(points[i][1])};
    }
    boardFromCorners(camera, frame, corners, (int)file["canonical_order_confirmed"] == 1, board);
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
    require(space == "original", "Detection boxes must use original camera pixels");
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
    assignPieces(camera, board, frame, ds, clear, observation);
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
    getBoard(fs, camera, frame, board);
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
    cv::Mat overlay, rectified;
    drawBoard(image, board, hasObservation ? &observation : nullptr, overlay);
    saveImage(output + ".png", overlay);
    if (board.geometryValid)
    {
        rectifiedBoard(image, board, rectified);
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
                      << "  scara_geometry_tool inspect fixed-board.yml result.json\n";
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
            std::vector<scara::PointPair> fit, checks;
            scara::readPairs(fs["fit_points"], fit);
            scara::readPairs(fs["check_points"], checks);
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
