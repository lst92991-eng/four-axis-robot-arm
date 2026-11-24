#include "site.hpp"
#include <toml++/toml.hpp>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <stdexcept>

namespace scara
{
static void readSite(const toml::table &site, GameConfig &config, Session &session);
static void readArm(const toml::table &site, ArmGeometry &arm);

std::string Site_path()
{
    if (const char *path = std::getenv("SCARA_SITE"); path && *path) return path;
    const std::string root = SCARA_CONFIG_DIR;
    const std::string measured = root + "/site/installation.toml";
    if (std::filesystem::exists(std::filesystem::u8path(measured))) return measured;
    if (std::filesystem::exists(std::filesystem::u8path(root + "/site/installation.site")) ||
        std::filesystem::exists(std::filesystem::u8path(root + "/site/installation.hpp")))
        throw std::runtime_error("请先用 15-site 将原示教迁移为 installation.toml");
    return root + "/installation.toml";
}

void Site_loadFile(const std::string &path, GameConfig &config, Session &session)
{
    readSite(toml::parse_file(path), config, session);
}

void Site_parse(std::string_view text, GameConfig &config, Session &session)
{
    readSite(toml::parse(text), config, session);
}

void Site_loadArm(const std::string &path, ArmGeometry &arm)
{
    readArm(toml::parse_file(path), arm);
}

//---------TOML 数值读取区---------
static double number(const toml::node *node, const char *name)
{
    if (node && node->is_number())
    {
        double value = node->value<double>().value_or(NAN);
        if (std::isfinite(value)) return value;
    }
    throw std::runtime_error(std::string("需要有限数值 ") + name);
}

static void readArray(const toml::node *node, const char *name, double *values, size_t count)
{
    const auto *array = node ? node->as_array() : nullptr;
    if (!array || array->size() != count) throw std::runtime_error(std::string("数组长度错误 ") + name);
    for (size_t i = 0; i < count; ++i) values[i] = number(array->get(i), name);
}

static void readArm(const toml::table &site, ArmGeometry &arm)
{
    const auto section = site["arm"];
    double lengths[2], drive[2], joint[2], units[2];
    readArray(section["lengths_mm"].node(), "arm.lengths_mm", lengths, 2);
    readArray(section["reference_drive_deg"].node(), "arm.reference_drive_deg", drive, 2);
    readArray(section["reference_joint_rad"].node(), "arm.reference_joint_rad", joint, 2);
    readArray(section["joint_per_drive"].node(), "arm.joint_per_drive", units, 2);
    if (lengths[0] <= 0 || lengths[1] <= 0 || units[0] == 0 || units[1] == 0)
        throw std::runtime_error("臂长须为正，驱动换算系数须非零");
    arm.upperMm = lengths[0];
    arm.forearmMm = lengths[1];
    arm.j2ReferenceDeg = drive[0];
    arm.j3ReferenceMotorDeg = drive[1];
    arm.j2JointPerDrive = units[0];
    arm.j3JointPerDrive = units[1];
    arm.shoulderReferenceRad = joint[0];
    arm.elbowReferenceRad = joint[1];
}

//---------安装、标定和对局参数区---------
static void readSite(const toml::table &site, GameConfig &config, Session &session)
{
    readArm(site, session.arm);
    const auto z = site["z"];
    WorkHeights &heights = session.heights;
    heights.pickupDeg = number(z["pickup_deg"].node(), "z.pickup_deg");
    heights.travelDeg = number(z["travel_deg"].node(), "z.travel_deg");
    heights.releaseDeg = z["release_deg"] ? number(z["release_deg"].node(), "z.release_deg") : heights.pickupDeg;
    heights.clearanceDeg = z["clearance_deg"] ? number(z["clearance_deg"].node(), "z.clearance_deg") : heights.travelDeg;
    // 用实际驱动角检查层高顺序，兼容丝杆读数递增或递减。
    double travel = heights.travelDeg - heights.pickupDeg;
    double clearance = (heights.clearanceDeg - heights.pickupDeg) / travel;
    double release = (heights.releaseDeg - heights.pickupDeg) / travel;
    if (!std::isfinite(travel) || !(clearance > 0 && clearance <= 1 && release >= 0 && release <= clearance))
        throw std::runtime_error("Z 轴吸取、释放、净空与搬运层顺序错误");
    if (z["grip_dwell_ms"])
    {
        auto dwell = z["grip_dwell_ms"].value_exact<int64_t>().value_or(-1);
        if (dwell < 0 || dwell > INT32_MAX) throw std::runtime_error("z.grip_dwell_ms 无效");
        heights.gripDwellMs = static_cast<int>(dwell);
    }

    const auto motion = site["motion"];
    double park[2], discard[2];
    readArray(motion["park_deg"].node(), "motion.park_deg", park, 2);
    readArray(motion["discard_xy_mm"].node(), "motion.discard_xy_mm", discard, 2);
    session.park = {park[0], park[1]};
    session.capturePlaceMm = {discard[0], discard[1]};
    readArray(motion["velocity_deg_s"].node(), "motion.velocity_deg_s", config.motion.velocity.data(), 3);
    readArray(motion["acceleration_deg_s2"].node(), "motion.acceleration_deg_s2", config.motion.acceleration.data(), 3);
    for (int axis = 0; axis < 3; ++axis)
        if (config.motion.velocity[axis] <= 0 || config.motion.acceleration[axis] <= 0)
            throw std::runtime_error("速度和加速度限制须为正");

    const auto vision = site["vision"];
    const auto *points = vision["points"].as_array();
    if (!points || points->size() != 3) throw std::runtime_error("vision.points 需要红左、红右、黑左三点");
    BoardPoint mark[3];
    for (int i = 0; i < 3; ++i)
    {
        double point[4];
        readArray(points->get(i), "vision.points", point, 4);
        if (point[0] < 0 || point[0] >= ImageWidth || point[1] < 0 || point[1] >= ImageHeight)
            throw std::runtime_error("vision.points 像素超出原图");
        mark[i] = {{point[0], point[1]}, {point[2], point[3]}};
    }
    Geometry_calibrate(mark[0], mark[1], mark[2], session.layout);
    if (vision["target_offsets_mm"])
    {
        const auto *offsets = vision["target_offsets_mm"].as_array();
        if (!offsets) throw std::runtime_error("vision.target_offsets_mm 须为数组");
        for (const auto &node : *offsets)
        {
            double offset[4];
            readArray(&node, "vision.target_offsets_mm", offset, 4);
            if (offset[0] < 0 || offset[0] >= Rows || offset[1] < 0 || offset[1] >= Columns ||
                std::floor(offset[0]) != offset[0] || std::floor(offset[1]) != offset[1])
                throw std::runtime_error("格点修正的行列无效");
            auto &cell = session.layout.cell[int(offset[0])][int(offset[1])];
            cell.xy = session.layout.origin.xy + offset[0] * session.layout.rowStep.xy +
                      offset[1] * session.layout.columnStep.xy + cv::Point2d(offset[2], offset[3]);
        }
    }
    if (vision["roi"])
    {
        double roi[4];
        readArray(vision["roi"].node(), "vision.roi", roi, 4);
        for (double value : roi)
            if (value < 0 || value > ImageWidth || std::floor(value) != value)
                throw std::runtime_error("vision.roi 须为原图内的整数像素区域");
        if (roi[2] <= 0 || roi[3] <= 0 || roi[0] + roi[2] > ImageWidth || roi[1] + roi[3] > ImageHeight)
            throw std::runtime_error("vision.roi 超出原图");
        config.visionRoi = {int(roi[0]), int(roi[1]), int(roi[2]), int(roi[3])};
    }
    config.cameraUrl = vision["camera_url"].value_or(config.cameraUrl);
    const auto game = site["game"];
    config.engineDir = game["engine_directory"].value_or(config.engineDir);
    config.humanRed = game["human_red"].value_or(config.humanRed);
    config.redToMove = game["red_to_move"].value_or(config.redToMove);
    if (game["think_ms"])
    {
        auto think = game["think_ms"].value_exact<int64_t>().value_or(0);
        if (think < 1 || think > 60000) throw std::runtime_error("game.think_ms 须在 1 到 60000");
        config.thinkMs = static_cast<int>(think);
    }
}
}
