// 把历史台架记录转换成课堂程序使用的 Geometry、ArmGeometry 和 WorkHeights。
#include "legacy_site.hpp"
#include <cmath>
#include <filesystem>
#include <stdexcept>

namespace scara
{
//---------三点标定区---------
void Bench_geometry(const cv::FileNode &config, Geometry &geometry, cv::Matx23d *pixelToBase)
{
    geometry = {};
    cv::Mat rows;
    // points 的每一行保存同一交点的 u、v 像素和基座 X、Y 毫米。
    config["points"] >> rows;
    if (rows.size() != cv::Size(4,3) || rows.type() != CV_64F || !cv::checkRange(rows))
        throw std::runtime_error("旧布局需要三行 [u,v,X,Y]");
    BoardPoint points[3];
    for (int i = 0; i < 3; ++i)
    {
        const double *p = rows.ptr<double>(i);
        points[i] = {{p[0],p[1]}, {p[2],p[3]}};
    }
    if (pixelToBase)
    {
        // 矩阵的每行是 [u,v,1]。用三点同时解 X 和 Y 的系数，再转成 2×3。
        cv::Mat a(3,3,CV_64F,cv::Scalar(1)), coefficients;
        rows.colRange(0,2).copyTo(a.colRange(0,2));
        if (!cv::solve(a, rows.colRange(2,4), coefficients)) throw std::runtime_error("三个标定点不能共线");
        *pixelToBase = cv::Mat(coefficients.t());
    }
    geometry = Geometry_calibrate(points[0], points[1], points[2]);
}

//---------参数读取区---------
bool Arm_load(const std::string &path, ArmGeometry &arm)
{
    arm = {};
    cv::FileStorage file(path, cv::FileStorage::READ);
    if (!file.isOpened()) return false;
    // 臂长、带轮比和方向采用这台机械臂的固定值；旧装配文件仍可提供自己的数值。
    ArmGeometry measured;
    measured.upperMm = measured.forearmMm = 275;
    auto lengths = file["arm_lengths_mm"];
    if (!lengths.empty())
    {
        if (lengths.size() != 2) return false;
        measured.upperMm = double(lengths[0]);
        measured.forearmMm = double(lengths[1]);
    }
    // J2 的比例和 J3 的传动比分别用于肩关节和肘关节的角度换算。
    double j3Ratio, j2Sign, j3Sign;
    cv::read(file["j3_ratio"], j3Ratio, 3.0);
    cv::read(file["j2_sign"], j2Sign, 1.0);
    cv::read(file["j3_sign"], j3Sign, -1.0);
    for (const char *key : {"j2_reference_deg", "j3_reference_motor_deg", "j2_scale", "shoulder_ref_rad", "elbow_ref_rad"})
        if ((!file[key].isReal() && !file[key].isInt()) || !std::isfinite(double(file[key]))) return false;
    measured.shoulder.driveRefDeg = double(file["j2_reference_deg"]);
    measured.elbow.driveRefDeg = double(file["j3_reference_motor_deg"]);
    double j2Scale = double(file["j2_scale"]);
    measured.shoulder.jointRefRad = double(file["shoulder_ref_rad"]);
    measured.elbow.jointRefRad = double(file["elbow_ref_rad"]);
    if (!(measured.upperMm > 0 && measured.forearmMm > 0 && j2Scale > 0 && j3Ratio > 0) ||
        !std::isfinite(measured.upperMm + measured.forearmMm + j3Ratio) ||
        std::abs(j2Sign) != 1 || std::abs(j3Sign) != 1) return false;
    measured.shoulder.jointPerDrive = j2Scale * j2Sign;
    measured.elbow.jointPerDrive = j3Sign / j3Ratio;
    // 臂长、传动比和方向通过核对后，整组参数一次写回。
    arm = measured;
    return true;
}

//---------旧记录的 Z 工作高度区---------
bool WorkHeights_load(const std::string &path, WorkHeights &heights)
{
    heights = {};
    cv::FileStorage file(path, cv::FileStorage::READ);
    auto z = file["z"];
    for (const char *key : {"pickup_deg", "travel_deg"})
        if (!z[key].isReal() && !z[key].isInt()) return false;
    WorkHeights measured;
    measured.pickupDeg = double(z["pickup_deg"]);
    measured.travelDeg = double(z["travel_deg"]);
    // 旧记录省略净空层时采用搬运层，吸合停留时间的单位为毫秒。
    cv::read(z["clearance_deg"], measured.clearanceDeg, measured.travelDeg);
    cv::read(file["pickup_at_rest_ms"], measured.gripDwellMs, 420);
    // 释放高度沿示教的竖直行程插值。未指定偏移时就在吸取高度放子。
    double riseMm, releaseOffsetMm;
    cv::read(z["operator_estimated_rise_mm"], riseMm, 20.0);
    cv::read(z["release_offset_mm"], releaseOffsetMm, 0.0);
    double travel = measured.travelDeg - measured.pickupDeg;
    if (travel == 0 || !std::isfinite(travel) || !(riseMm > 0) || !std::isfinite(riseMm) || measured.gripDwellMs < 0)
        return false;
    // 把净空和释放位置都表示为吸取到搬运这段行程的比例。
    double clearance = (measured.clearanceDeg - measured.pickupDeg) / travel;
    double release = releaseOffsetMm / riseMm;
    if (!(0 < clearance && clearance <= 1 && 0 <= release && release <= clearance)) return false;
    measured.releaseDeg = measured.pickupDeg + travel * release;
    // 最后用吸取间隙毫米数调整取子角，释放角仍以原始示教取子面为基准。
    double gapMm;
    cv::read(z["pickup_gap_mm"], gapMm, 0.0);
    measured.pickupDeg += travel * gapMm / riseMm;
    if (!(gapMm >= 0 && gapMm < riseMm)) return false;
    heights = measured;
    return true;
}

static std::string filePath(const std::filesystem::path &directory, const cv::FileNode &node);
static void readPair(const cv::FileNode &node, double &first, double &second);

//---------初始化区---------
//读取一次布局配置，Geometry_calibrate完成标定和90点缓存
// 继续读取关节参考、工作层和三轴限幅，具体格点在规划时做逆解。
void Bench_load(const std::string &path, GameConfig &config, Session &session)
{
    //对局设置、固定棋盘设置
    cv::FileStorage file, layout;
    std::filesystem::path configDirectory;
    std::string layoutPath, armPath, heightsPath;

    for (auto &row : session.observed)
        row.fill('?');
    if (!file.open(path, cv::FileStorage::READ)) throw std::runtime_error("无法打开现场配置 " + path);
    configDirectory = std::filesystem::u8path(path).parent_path();
    // 一台机器可把三点、机械参考和高度写在同一份现场记录里。
    if (file["layout_file"].empty())
        Bench_geometry(file.root(), session.layout);
    else
    {
        layoutPath = filePath(configDirectory, file["layout_file"]);
        if (!layout.open(layoutPath, cv::FileStorage::READ)) throw std::runtime_error("无法打开棋盘标定 " + layoutPath);
        Bench_geometry(layout.root(), session.layout);
    }
    // 机械和高度记录可以在主配置中，也可以通过相对路径分别提供。
    armPath = filePath(configDirectory, file["arm_file"]);
    heightsPath = filePath(configDirectory, file["heights_file"]);
    if (armPath.empty()) armPath = path;
    if (heightsPath.empty()) heightsPath = path;
    if (!Arm_load(armPath, session.arm) || !WorkHeights_load(heightsPath, session.heights))
        throw std::runtime_error("机械参考或工作高度尚未示教");
    cv::read(file["pickup_at_rest_ms"], session.heights.gripDwellMs, session.heights.gripDwellMs);
    if (session.heights.gripDwellMs < 0) throw std::runtime_error("吸合时间无效");
    //限幅顺序固定为J1/J2/J3，单位对应各驱动的角度口径
    cv::FileNode speed = file["joint_velocity_deg_s"], acceleration = file["joint_acceleration_deg_s2"];
    if (speed.size() != 3 || acceleration.size() != 3) throw std::runtime_error("需要三轴速度和加速度上限");
    for (int axis = 0; axis < 3; ++axis)
    {
        for (cv::FileNode value : {speed[axis], acceleration[axis]})
            if (!value.isReal() && !value.isInt()) throw std::runtime_error("联动限幅需要数值");
        config.motion.velocity[axis] = double(speed[axis]);
        config.motion.acceleration[axis] = double(acceleration[axis]);
    }
    if (!config.motion.valid()) throw std::runtime_error("联动限幅需要正的实测配置数值");
    readPair(file["park_joints_deg"], session.park.j2Deg, session.park.j3MotorDeg);
    readPair(file["capture_place_mm"], session.capturePlaceMm.x, session.capturePlaceMm.y);
    // 相机与引擎取自同一站点配置，执子方和思考时间可覆盖默认值。
    file["camera_url"] >> config.cameraUrl;
    config.engineDir = filePath(configDirectory, file["engine_directory"]);
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

// 观察位用两个驱动角，弃子点用两个基座坐标，两处共用这个数对读取函数。
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
