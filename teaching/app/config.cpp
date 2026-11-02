#include "config.hpp"
#include <opencv2/core.hpp>
#include <cmath>
#include <filesystem>
#include <stdexcept>

namespace app {
namespace {
bool flag(const cv::FileNode& node, const char* name) { return int(node[name]) == 1; }
motion::Pose pose(const cv::FileNode& node) {
    std::vector<double> values; node >> values;
    if (values.size() != 4) throw std::runtime_error("点位必须是[x,y,z,yaw]四个数");
    for (double v : values) if (!std::isfinite(v)) throw std::runtime_error("点位存在非有限数");
    return {values[0], values[1], values[2], values[3]};
}
cv::Matx33d matrix(const cv::FileNode& node) {
    std::vector<double> values; node >> values;
    if (values.size() != 9) throw std::runtime_error("标定矩阵必须是9个数");
    cv::Matx33d out;
    std::copy(values.begin(), values.end(), out.val);
    return out;
}
void points(const cv::FileNode& node, std::array<cv::Point2f, 4>& out) {
    if (!node.isSeq() || node.size() != 4) throw std::runtime_error("需要四个标记点");
    int i = 0;
    for (const auto& item : node) {
        std::vector<float> xy; item >> xy;
        if (xy.size() != 2) throw std::runtime_error("标记点必须是[x,y]");
        out[i++] = {xy[0], xy[1]};
    }
}
}
Config loadConfig(const std::string& path) {
    cv::FileStorage file(path, cv::FileStorage::READ);
    if (!file.isOpened()) throw std::runtime_error("不能读取配置文件");
    Config c;
    const auto root = file.root();
    root["can_interface"] >> c.canInterface;
    root["gpio_chip"] >> c.gpioChip;
    const int gpio = int(root["gpio_line"]);
    if (gpio < 0) throw std::runtime_error("GPIO线号不能为负");
    c.gpioLine = unsigned(gpio);
    c.gpioActiveHigh = flag(root, "gpio_active_high");
    c.hardwareVerified = flag(root, "hardware_verified");
    c.faultReleaseSafe = flag(root, "fault_release_safe");
    std::string color; root["robot_color"] >> color;
    if (color != "red" && color != "black") throw std::runtime_error("robot_color必须是red或black");
    c.robotRed = color == "red";
    const auto directory = std::filesystem::absolute(path).parent_path();
    const auto resource = [&](const char* key) {
        std::string value; root[key] >> value;
        return value.empty() ? value : (directory / value).lexically_normal().string();
    };
    c.engineDirectory = resource("engine_directory"); c.model = resource("model");
    c.cameraScript = resource("camera_script"); c.journal = resource("journal");
    root["think_ms"] >> c.thinkMs;
    root["camera_width"] >> c.cameraWidth; root["camera_height"] >> c.cameraHeight;
    root["piece_contact_z"] >> c.pieceContactZ; root["piece_yaw"] >> c.pieceYaw;
    c.requireTcp = flag(root, "require_tcp"); c.latencyCalibrated = flag(root, "latency_calibrated");
    root["capture_latency_bound_seconds"] >> c.captureLatencyBoundSeconds;
    const auto m = root["motion"];
    m["l1"] >> c.motion.l1; m["l2"] >> c.motion.l2;
    c.motion.calibrated = flag(m, "calibrated");
    m["x_min"] >> c.motion.xMinimum; m["x_max"] >> c.motion.xMaximum;
    m["y_min"] >> c.motion.yMinimum; m["y_max"] >> c.motion.yMaximum;
    m["link_radius"] >> c.motion.linkRadius; m["tool_radius"] >> c.motion.toolRadius;
    m["minimum_sin_elbow"] >> c.motion.minimumSinElbow;
    m["sample_angle"] >> c.motion.sampleAngle;
    m["timeout_seconds"] >> c.motion.moveTimeoutSeconds;
    m["settle_seconds"] >> c.motion.settleSeconds; m["feedback_seconds"] >> c.motion.feedbackSeconds;
    m["estimate_error_limit"] >> c.motion.estimateErrorLimit;
    m["acceleration_noise"] >> c.motion.accelerationNoise; m["encoder_sigma"] >> c.motion.encoderSigma;
    m["tcp_sigma_mm"] >> c.motion.tcpSigmaMm; m["maximum_observation_age"] >> c.motion.maximumObservationAge;
    c.motion.acceleration = unsigned(int(m["acceleration"]));
    const auto axes = m["axes"];
    if (!axes.isSeq() || axes.size() != 4) throw std::runtime_error("轴配置必须按Z/肩/肘/腕给出四项");
    int index = 0;
    for (const auto& n : axes) {
        auto& a = c.motion.axes[index++];
        a.node = unsigned(int(n["node"])); a.direction = int(n["direction"]);
        n["motor_deg_per_unit"] >> a.motorDegPerUnit; n["zero_motor_deg"] >> a.zeroMotorDeg;
        n["min"] >> a.minimum; n["max"] >> a.maximum; n["tolerance"] >> a.tolerance;
        a.maxRpm = unsigned(int(n["max_rpm"])); a.pulsesPerRev = unsigned(int(n["pulses_per_rev"]));
        a.currentLimitMa = unsigned(int(n["current_limit_ma"]));
    }
    for (const auto& n : m["forbidden"]) {
        std::vector<double> v; n >> v;
        if (v.size() != 5) throw std::runtime_error("障碍物格式为[x,y,radius,zMin,zMax]");
        c.motion.forbidden.push_back({v[0],v[1],v[2],v[3],v[4]});
    }
    const auto t = root["task"];
    t["travel_z"] >> c.task.travelZ; t["pickup_wait"] >> c.task.pickupWait;
    t["release_wait"] >> c.task.releaseWait; t["maximum_hold_seconds"] >> c.task.maximumHoldSeconds;
    c.task.park = pose(t["park"]);
    for (int side = 0; side < 2; ++side)
        for (const auto& n : t[side == 0 ? "red_capture_slots" : "black_capture_slots"])
            c.captureSlots[side].push_back(pose(n));
    const auto v = root["vision"];
    v["board_width_mm"] >> c.vision.boardWidthMm; v["board_height_mm"] >> c.vision.boardHeightMm;
    v["grid_x_mm"] >> c.vision.gridXmm; v["grid_y_mm"] >> c.vision.gridYmm;
    v["pitch_mm"] >> c.vision.pitchMm; v["marker_side_mm"] >> c.vision.markerSideMm;
    v["pixels_per_mm"] >> c.vision.pixelsPerMm; v["stable_frames"] >> c.vision.stableFrames;
    v["confidence"] >> c.vision.confidence;
    points(v["marker_centers_mm"], c.vision.markerCentersMm);
    points(v["reference_marker_pixels"], c.vision.referenceMarkerPixels);
    v["board_pose_tolerance_pixels"] >> c.vision.boardPoseTolerancePixels;
    c.vision.boardToBase = matrix(v["board_to_base"]);
    c.vision.boardCalibrated = flag(v, "board_calibrated");
    c.vision.tcpMarkerId = int(v["tcp_marker_id"]); c.vision.tcpCalibrated = flag(v, "tcp_calibrated");
    c.vision.tcpImageToBase = matrix(v["tcp_image_to_base"]);
    v["tcp_plane_z_mm"] >> c.vision.tcpPlaneZmm; v["tcp_plane_tolerance_mm"] >> c.vision.tcpPlaneToleranceMm;
    std::vector<double> offset; v["tcp_marker_offset_mm"] >> offset;
    if (offset.size() != 2) throw std::runtime_error("末端标记偏移需要[x,y]");
    c.vision.tcpMarkerOffsetMm = {offset[0],offset[1]};
    c.vision.maxObservationAgeSeconds = c.motion.maximumObservationAge;
    return c;
}
void validateConfig(const Config& c) {
    if (!c.hardwareVerified || !c.motion.calibrated || !c.vision.boardCalibrated)
        throw std::runtime_error("硬件/机械零点/棋盘标定尚未确认；示例参数不能启动实机");
    motion::validate(c.motion);
    BoardVision check(c.vision);
    if (c.canInterface.empty() || c.gpioChip.empty() || c.engineDirectory.empty() || c.model.empty() ||
        c.cameraScript.empty() || c.journal.empty() || c.thinkMs < 50 || c.thinkMs > 10000 ||
        c.cameraWidth <= 0 || c.cameraHeight <= 0 || c.cameraWidth > 4096 || c.cameraHeight > 4096 ||
        c.cameraWidth % 2 || c.cameraHeight % 2 || !std::isfinite(c.pieceContactZ + c.pieceYaw))
        throw std::runtime_error("程序路径、图像尺寸或思考参数无效");
    if (!c.faultReleaseSafe || !std::isfinite(c.task.maximumHoldSeconds) || c.task.maximumHoldSeconds <= 0)
        throw std::runtime_error("需先确认故障限时断磁的现场处置方式和线圈允许通电时间");
    if (!std::isfinite(c.captureLatencyBoundSeconds) || c.captureLatencyBoundSeconds < 0 ||
        (c.latencyCalibrated && (c.captureLatencyBoundSeconds <= 0 ||
          c.captureLatencyBoundSeconds >= c.motion.maximumObservationAge)))
        throw std::runtime_error("采集延迟上界无效");
    if (c.requireTcp && (!c.latencyCalibrated || !c.vision.tcpCalibrated))
        throw std::runtime_error("要求运动EKF，但真实末端平面/采集延迟尚未标定");
    for (const auto& slots : c.captureSlots)
        if (slots.size() != 15) throw std::runtime_error("每方需要15个已标定吃子收纳点");
}
} // namespace app
