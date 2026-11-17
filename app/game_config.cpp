#include "game.hpp"
#include "vision/io.hpp"
#include <cmath>
#include <filesystem>
#include <stdexcept>

namespace scara
{
static std::string filePath(const std::filesystem::path &directory, const cv::FileNode &node);
static void readPair(const cv::FileNode &node, double &first, double &second);

//---------初始化区---------
//读取一次布局配置，Geometry_init完成标定和90点缓存
//随后核对机械参考和可达性，交给对局流程
void Match_load(const std::string &path, GameConfig &config, Session &session)
{
    //对局设置、固定棋盘设置
    cv::FileStorage file, layout;
    std::filesystem::path configDirectory;
    std::string layoutPath, armPath, heightsPath;

    for (auto &row : session.observed.board)
        row.fill('?');
    readStorage(path, file);
    configDirectory = std::filesystem::u8path(path).parent_path();
    layoutPath = filePath(configDirectory, file["layout_file"]);
    readStorage(layoutPath, layout);
    //读取课堂约定的三点，建立棋盘坐标和90个目标缓存
    Geometry_init(layout.root(), session.layout);

    armPath = filePath(configDirectory, file["arm_file"]);
    heightsPath = filePath(configDirectory, file["heights_file"]);
    if (!Arm_load(armPath, session.arm) || !WorkHeights_load(heightsPath, session.heights))
        throw std::runtime_error("机械参考或工作高度尚未示教");
    //限幅顺序固定为J1/J2/J3，单位对应各驱动的角度口径
    cv::FileNode speed = file["joint_velocity_deg_s"], acceleration = file["joint_acceleration_deg_s2"];
    if (speed.size() != 3 || acceleration.size() != 3) throw std::runtime_error("需要三轴速度和加速度上限");
    for (int axis = 0; axis < 3; ++axis)
    {
        for (cv::FileNode value : {speed[axis], acceleration[axis]})
            if ((!value.isReal() && !value.isInt()) || !std::isfinite(double(value)) || double(value) <= 0)
                throw std::runtime_error("联动限幅需要正的实测配置数值");
        config.motion.velocity[axis] = double(speed[axis]);
        config.motion.acceleration[axis] = double(acceleration[axis]);
    }
    readPair(file["park_joints_deg"], config.park.j2Deg, config.park.j3MotorDeg);
    readPair(file["capture_place_mm"], session.capturePlaceMm.x, session.capturePlaceMm.y);
    //具体取放目标在Pick_plan中逆解，不可达时终止本次规划
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
//---------配置字段读取区---------
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

} // namespace scara
