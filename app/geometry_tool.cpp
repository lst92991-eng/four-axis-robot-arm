#include "vision/geometry.hpp"
#include "vision/io.hpp"
#include <opencv2/imgcodecs.hpp>
#include <filesystem>
#include <iostream>

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
    return (double)n; // 四角的有限值和范围由boardFromCorners核对。
}
std::string relativePath(const std::string &config, const std::string &value)
{
    std::filesystem::path path;

    require(!value.empty(), "Missing file path");
    path = std::filesystem::u8path(config).parent_path();
    path /= std::filesystem::u8path(value);
    return path.lexically_normal().u8string();
}
// 四角依次为红方左下、右下、黑方右上、左上；方向由首次示教确认。
void getBoard(const cv::FileStorage &file, Board &board)
{
    cv::FileNode points = file["corners"];
    std::array<cv::Point2d, 4> corners;
    bool confirmed;

    require((std::string)file["mode"] == "manual", "Fixed board requires saved manual corners");
    require(points.isSeq() && points.size() == 4, "Four original-image corners required");
    for (int i = 0; i < 4; ++i)
    {
        require(points[i].size() == 2, "Corner must have two coordinates");
        corners[i].x = number(points[i][0]);
        corners[i].y = number(points[i][1]);
    }
    confirmed = int(file["canonical_order_confirmed"]) == 1;
    boardFromCorners(corners, confirmed, board);
}
void inspect(const std::string &config, const std::string &output)
{
    cv::FileStorage fs, result;
    std::string frame, path, planeFile, message;
    cv::Mat image, overlay, rectified;
    Board board;
    PlaneMap plane;

    readStorage(config, fs);
    checkCamera(fs["camera"]);
    fs["frame_id"] >> frame;
    path = relativePath(config, (std::string)fs["image"]);
    image = cv::imread(path, cv::IMREAD_COLOR);
    require(!image.empty(), "Cannot decode image");
    require(image.size() == cv::Size(ImageWidth, ImageHeight), "Image size differs from camera profile");
    getBoard(fs, board);
    fs["plane_file"] >> planeFile;
    if (!planeFile.empty())
    {
        path = relativePath(config, planeFile);
        loadPlane(path, plane);
    }
    memoryStorage(true, result);
    result << "motion_enabled" << 0;
    result << "single_frame_only" << 1;
    result << "camera_id" << CameraId;
    result << "mapping_model" << "affine";
    result << "calibration_id" << plane.id;
    result << "frame_id" << frame;
    result << "geometry_valid" << int(board.geometryValid);
    result << "orientation_valid" << int(board.orientationValid);
    result << "points" << "[";
    //四角建系成功后，依次导出90个交点的像素与基座坐标
    for (int r = 0; r < Rows; ++r)
        for (int c = 0; c < Columns; ++c)
        {
            cv::Point2d pixel = board.intersections[r * Columns + c];
            cv::Point2d baseMm;
            bool targetValid = gridTarget(plane, board, c, r, baseMm);
            result << "{";
            result << "column" << c;
            result << "row" << r;
            result << "pixel_geometric" << "[" << pixel.x << pixel.y << "]";
            result << "base_xy_valid" << int(targetValid);
            if (targetValid)
            {
                result << "base_xy_mm" << "[" << baseMm.x << baseMm.y << "]";
                result << "plane_z_mm" << plane.planeZmm;
            }
            result << "}";
        }
    result << "]";
    message = result.releaseAndGetString();
    writeBytes(output, message);
    drawBoard(image, board, overlay);
    require(cv::imwrite(output + ".png", overlay), "Cannot write overlay");
    rectifiedBoard(image, board, rectified);
    require(cv::imwrite(output + ".rectified.png", rectified), "Cannot write rectified board");
    std::cout << "geometry=" << board.geometryValid << " orientation=" << board.orientationValid
              << "\nWrote " << output << " and overlay. No motion.\n";
}
} // namespace
int main(int argc, char **argv)
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
            scara::PlaneMap map;
            scara::loadPlane(input, map);
            std::cout << "fit/check maximum error (mm): " << map.fitError.maxMm << "/" << map.checkError.maxMm << '\n';
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
