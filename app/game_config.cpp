#include "game.hpp"
#include "vision/io.hpp"
#include <cmath>
#include <filesystem>
#include <stdexcept>

namespace scara
{
static std::string filePath(const std::string &owner, const cv::FileNode &node)
{
    std::string name = (std::string)node;
    if (name.empty()) return {};
    auto path = std::filesystem::u8path(owner).parent_path() / std::filesystem::u8path(name);
    return path.lexically_normal().u8string();
}

static void readPair(const cv::FileNode &node, double &first, double &second)
{
    if (node.size() != 2) throw std::runtime_error("需要两个已示教数值");
    for (auto value : node)
        if ((!value.isInt() && !value.isReal()) || !std::isfinite(double(value)))
            throw std::runtime_error("示教值不能为空或占位符");
    first = double(node[0]);
    second = double(node[1]);
}

void Match_load(const std::string &path, GameConfig &config, Session &session)
{
    for (auto &row : session.observed.board)
        row.fill('?');
    auto file = readStorage(path);
    std::string layoutPath = filePath(path, file["layout_file"]);
    auto layout = readStorage(layoutPath);
    checkCamera(layout["camera"]);
    auto cornersNode = layout["corners"];
    if (cornersNode.size() != 4) throw std::runtime_error("固定布局需要四角");
    std::array<cv::Point2d, 4> corners;
    for (int i = 0; i < 4; ++i)
        readPair(cornersNode[i], corners[i].x, corners[i].y);
    Board grid;
    PlaneMap plane;
    boardFromCorners(corners, int(layout["canonical_order_confirmed"]) == 1, grid);
    loadPlane(filePath(layoutPath, layout["plane_file"]), plane);
    if (!FixedBoard_setup(plane, grid, session.layout)) throw std::runtime_error("固定坐标尚未确认");
    if (!Arm_load(filePath(path, file["arm_file"]), session.arm) ||
        !WorkHeights_load(filePath(path, file["heights_file"]), session.heights))
        throw std::runtime_error("机械参考或工作高度尚未示教");
    readPair(file["park_joints_deg"], config.park.j2Deg, config.park.j3MotorDeg);
    readPair(file["capture_place_mm"], session.capturePlaceMm.x, session.capturePlaceMm.y);
    JointTarget target;
    session.capturePlaceTaught = Arm_inverse(session.arm, session.capturePlaceMm, target);
    if (!session.capturePlaceTaught) throw std::runtime_error("吃子区不在可达范围内");
    for (int i = 0; i < Cells; ++i)
        if (!session.layout.mapped[i] || !Arm_inverse(session.arm, session.layout.targets[i], target))
            throw std::runtime_error("棋盘存在未标定或不可达的交点：" + std::to_string(i));
    config.cameraUrl = (std::string)file["camera_url"];
    config.labelPath = filePath(path, file["label_image"]);
    config.engineDir = filePath(path, file["engine_directory"]);
    config.workDir = filePath(path, file["work_directory"]);
    config.scenarioPath = filePath(path, file["simulation_file"]);
    if (config.workDir.empty()) throw std::runtime_error("需要对局输出目录");
    if (!file["human_red"].empty()) config.humanRed = int(file["human_red"]) != 0;
    if (!file["red_to_move"].empty()) config.redToMove = int(file["red_to_move"]) != 0;
    if (!file["think_ms"].empty()) config.thinkMs = int(file["think_ms"]);
}
} // namespace scara
