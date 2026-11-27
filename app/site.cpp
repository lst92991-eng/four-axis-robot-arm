// 从本机安装表取出一次对局需要的数值。旧文件由 15-site 在安装时转换。
#include "site.hpp"
#include <toml++/toml.hpp>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <stdexcept>

namespace scara
{
using View = toml::node_view<const toml::node>;

//---------输入数值区---------
static double number(View value, const char *name)
{
    double n = value.value<double>().value_or(NAN);
    if (value.is_number() && std::isfinite(n)) return n;
    throw std::runtime_error(std::string("需要有限数值：") + name);
}

template<size_t N>
static std::array<double, N> numbers(View array, const char *name)
{
    if (!array.is_array() || array.as_array()->size() != N) throw std::runtime_error(std::string("数组长度错误：") + name);
    std::array<double, N> result;
    for (size_t i = 0; i < N; ++i) result[i] = number(array[i], name);
    return result;
}

static int integer(double n, int min, int max, const char *name)
{
    if (n >= min && n <= max && n == std::floor(n)) return static_cast<int>(n);
    throw std::runtime_error(std::string("需要范围内的整数：") + name);
}

//---------J2、J3 机械参考区---------
static JointCalibration readJoint(View axis)
{
    double drive = number(axis["drive_deg"], "驱动读数");
    double angle = number(axis["joint_deg"], "机械角");
    return {drive, angle * (CV_PI / 180.0), number(axis["ratio"], "转角比例")};
}

static ArmGeometry readArm(const toml::table &site)
{
    ArmGeometry arm;
    arm.upperMm = number(site["j2"]["link_mm"], "大臂长度");
    arm.forearmMm = number(site["j3"]["link_mm"], "小臂长度");
    arm.shoulder = readJoint(site["j2"]);
    arm.elbow = readJoint(site["j3"]);
    if (arm.upperMm <= 0 || arm.forearmMm <= 0) throw std::runtime_error("两段臂长须为正");
    if (arm.shoulder.jointPerDrive == 0 || arm.elbow.jointPerDrive == 0) throw std::runtime_error("转角比例须非零");
    return arm;
}

//---------J1 工作高度区---------
static WorkHeights readHeights(View z)
{
    double touch = number(z["touch_deg"], "贴子角");
    WorkHeights h = WorkHeights_fromContact(touch);
    if (z["carry_deg"]) h.travelDeg = number(z["carry_deg"], "搬运角");
    h.clearanceDeg = z["clearance_deg"] ? number(z["clearance_deg"], "净空角") : h.travelDeg;
    if (z["release_deg"]) h.releaseDeg = number(z["release_deg"], "释放角");
    if (auto ms = z["grip_dwell_ms"]) h.gripDwellMs = integer(ms.value_exact<int64_t>().value_or(-1), 0, INT32_MAX, "吸合毫秒");
    // 本机 J1 驱动角增大是抬升。三个工作层须位于贴子角的上方或贴子层。
    if (!(touch < h.clearanceDeg && h.clearanceDeg <= h.travelDeg))
        throw std::runtime_error("净空高度须在贴子与搬运高度之间");
    if (!(touch <= h.releaseDeg && h.releaseDeg <= h.clearanceDeg))
        throw std::runtime_error("释放高度超出净空范围");
    return h;
}

//---------棋盘标定区---------
static BoardPoint corner(View board, const char *name)
{
    auto [u, v, x, y] = numbers<4>(board[name], name);
    if (u < 0 || u >= ImageWidth || v < 0 || v >= ImageHeight) throw std::runtime_error(std::string("角点超出原图：") + name);
    return {{u, v}, {x, y}};
}

//---------相机裁剪区---------
static cv::Point cropPoint(View crop)
{
    auto [left, top] = numbers<2>(crop, "camera.crop_px");
    int x = integer(left, 0, ImageWidth - YoloSize, "取景 X");
    int y = integer(top, 0, ImageHeight - YoloSize, "取景 Y");
    return {x, y};
}

static cv::Point readCrop(const toml::table &site)
{
    if (auto crop = site["camera"]["crop_px"]) return cropPoint(crop);

    // 旧安装表省略裁剪坐标时，从随项目安装的 TOML 读固定机位值。
    const auto defaults = toml::parse_file(SCARA_CONFIG_DIR "/installation.toml");
    return cropPoint(defaults["camera"]["crop_px"]);
}

//---------完整安装表区---------
static void readSite(const toml::table &site, StaticParams &staticParams)
{
    // 旧整表先经 15-site 迁移，尤其是现场示教的搬运高度。
    for (const char *old : {"arm","vision","z.pickup_deg","z.travel_deg","motion.park_deg","motion.discard_xy_mm",
                            "motion.velocity_deg_s","motion.acceleration_deg_s2","camera.roi","board.points"})
        if (site.at_path(old)) throw std::runtime_error(std::string("请用 15-site 导入旧字段 ") + old);
    staticParams.arm = readArm(site);
    staticParams.heights = readHeights(site["z"]);
    auto board = site["board"];
    BoardPoint redLeft = corner(board, "red_left");
    BoardPoint redRight = corner(board, "red_right");
    BoardPoint blackLeft = corner(board, "black_left");
    // 三点在像素和基座两个平面中都必须展开成棋盘。
    double pixelArea = (redRight.pixel - redLeft.pixel).cross(blackLeft.pixel - redLeft.pixel);
    double baseArea = (redRight.xy - redLeft.xy).cross(blackLeft.xy - redLeft.xy);
    if (pixelArea == 0 || baseArea == 0 || !std::isfinite(baseArea))
        throw std::runtime_error("三点标定退化，请重新示教红左、红右、黑左");
    staticParams.layout = Geometry_calibrate(redLeft, redRight, blackLeft);

    auto motion = site["motion"];
    auto [j2, j3] = numbers<2>(motion["observe_deg"], "观察位");
    auto [x, y] = numbers<2>(motion["discard_mm"], "弃子点");
    staticParams.park = {j2, j3};
    staticParams.capturePlaceMm = {x, y};
    staticParams.motion.velocity = numbers<3>(motion["speed_deg_s"], "三轴限速");
    staticParams.motion.acceleration = numbers<3>(motion["accel_deg_s2"], "三轴加速度");
    if (!staticParams.motion.valid()) throw std::runtime_error("速度和加速度限制须为正");

    staticParams.visionCropOrigin = readCrop(site);
    auto game = site["game"];
    staticParams.humanRed = game["human_red"].value_or(staticParams.humanRed);
    staticParams.redToMove = game["red_to_move"].value_or(staticParams.redToMove);
    staticParams.engineDir = game["engine_directory"].value_or(staticParams.engineDir);
    if (auto level = game["skill_level"])
        staticParams.skillLevel = integer(level.value_exact<int64_t>().value_or(-1), 0, 20, "Pikafish 棋力档");
    if (auto ms = game["think_ms"]) staticParams.thinkMs = integer(ms.value_exact<int64_t>().value_or(0), 1, 60000, "思考毫秒");
}

static std::string defaultPath()
{
    if (const char *path = std::getenv("SCARA_SITE"); path && *path) return path;
    const std::string root = SCARA_CONFIG_DIR;
    const std::string measured = root + "/site/installation.toml";
    if (std::filesystem::exists(std::filesystem::u8path(measured))) return measured;
    if (std::filesystem::exists(std::filesystem::u8path(root + "/site/installation.site")) ||
        std::filesystem::exists(std::filesystem::u8path(root + "/site/installation.hpp")))
        throw std::runtime_error("请先用 15-site 将原示教迁移为 installation.toml");
    throw std::runtime_error("缺少本机安装表，请用 15-site --install 保存三点和 Z 示教");
}

void Site_loadFile(StaticParams &staticParams, const std::string &path)
{
    readSite(toml::parse_file(path.empty() ? defaultPath() : path), staticParams);
}

void Site_parse(std::string_view text, StaticParams &staticParams)
{
    readSite(toml::parse(text), staticParams);
}

ArmGeometry Site_loadArm(const std::string &path)
{
    return readArm(toml::parse_file(path));
}

} // namespace scara
