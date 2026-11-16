#include "game.hpp"
#include "vision/io.hpp"
#include <cmath>
#include <filesystem>
#include <stdexcept>

namespace scara
{
// 配置中引用的相对路径，从对应配置文件所在目录解析。
static std::string filePath(const std::filesystem::path &directory, const cv::FileNode &node)
{
    std::string name = (std::string)node;
    if (name.empty()) return {};
    std::filesystem::path path = directory / std::filesystem::u8path(name);
    return path.lexically_normal().u8string();
}

static void readPair(const cv::FileNode &node, double &first, double &second)
{
    if (node.size() != 2) throw std::runtime_error("需要两个已示教数值");
    for (cv::FileNode value : node)
        if ((!value.isInt() && !value.isReal()) || !std::isfinite(double(value)))
            throw std::runtime_error("示教值不能为空或占位符");
    first = double(node[0]);
    second = double(node[1]);
}

void Match_load(const std::string &path, GameConfig &config, Session &session)
{
    cv::FileStorage file, layout; // 对局设置、固定棋盘设置。
    std::filesystem::path configDirectory, layoutDirectory;
    cv::FileNode cornersNode;
    std::string layoutPath, armPath, heightsPath, planePath;
    std::array<cv::Point2d, 4> corners; // 原图四角，像素。
    Board grid;                         // 棋盘90个交点，像素。
    PlaneMap plane;                     // 像素到基座毫米的标定映射。
    JointTarget target;                 // 用格点的逆解结果检查可达性。
    bool confirmed, ready;

    for (auto &row : session.observed.board)
        row.fill('?');
    readStorage(path, file);
    configDirectory = std::filesystem::u8path(path).parent_path();
    layoutPath = filePath(configDirectory, file["layout_file"]);
    readStorage(layoutPath, layout);
    layoutDirectory = std::filesystem::u8path(layoutPath).parent_path();
    checkCamera(layout["camera"]);
    cornersNode = layout["corners"];
    if (cornersNode.size() != 4) throw std::runtime_error("固定布局需要四角");
    for (int i = 0; i < 4; ++i)
        readPair(cornersNode[i], corners[i].x, corners[i].y);
    confirmed = int(layout["canonical_order_confirmed"]) == 1;
    planePath = filePath(layoutDirectory, layout["plane_file"]);
    //采点标定先得到像素到毫米的系数，再由四角建立棋盘行列
    loadPlane(planePath, plane);
    boardFromCorners(corners, confirmed, grid);
    //初始化缓存90个交点，走棋时直接读取目标XY
    ready = FixedBoard_setup(plane, grid, session.layout);
    if (!ready) throw std::runtime_error("固定坐标尚未确认");

    armPath = filePath(configDirectory, file["arm_file"]);
    heightsPath = filePath(configDirectory, file["heights_file"]);
    ready = Arm_load(armPath, session.arm);
    if (ready) ready = WorkHeights_load(heightsPath, session.heights);
    if (!ready) throw std::runtime_error("机械参考或工作高度尚未示教");
    readPair(file["park_joints_deg"], config.park.j2Deg, config.park.j3MotorDeg);
    readPair(file["capture_place_mm"], session.capturePlaceMm.x, session.capturePlaceMm.y);
    session.capturePlaceTaught = Arm_inverse(session.arm, session.capturePlaceMm, target);
    if (!session.capturePlaceTaught) throw std::runtime_error("吃子区不在可达范围内");
    for (int i = 0; i < Cells; ++i)
        if (!session.layout.mapped[i] || !Arm_inverse(session.arm, session.layout.targets[i], target))
            throw std::runtime_error("棋盘存在未标定或不可达的交点：" + std::to_string(i));
    file["camera_url"] >> config.cameraUrl;
    config.labelPath = filePath(configDirectory, file["label_image"]);
    config.engineDir = filePath(configDirectory, file["engine_directory"]);
    config.workDir = filePath(configDirectory, file["work_directory"]);
    config.scenarioPath = filePath(configDirectory, file["simulation_file"]);
    if (config.workDir.empty()) throw std::runtime_error("需要对局输出目录");
    if (!file["human_red"].empty()) config.humanRed = int(file["human_red"]) != 0;
    if (!file["red_to_move"].empty()) config.redToMove = int(file["red_to_move"]) != 0;
    if (!file["think_ms"].empty()) config.thinkMs = int(file["think_ms"]);
}
} // namespace scara
