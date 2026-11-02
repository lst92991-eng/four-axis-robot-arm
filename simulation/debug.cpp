// 详细日志和真值对照只放这里。关闭SIM_DEBUG时不参与链接。
#include "debug.hpp"
#include <cmath>
#include <fstream>
#include <iomanip>
#include <filesystem>
#include <algorithm>
#include <chrono>
#include <vector>

namespace {
std::ofstream canLog, motionLog, eventLog;
double raw2 = 0, ekf2 = 0;
int samples = 0, frames = 0;
double maxPlacementError = 0;
int placements = 0;
std::vector<double> tickGaps;
std::chrono::steady_clock::time_point lastWall;
std::array<double, 4> lastCommand{};
std::string directory;
void onCan(bool tx, uint32_t id, const uint8_t *p, int n) {
    canLog << (tx ? "TX " : "RX ") << std::hex << id;
    for (int i = 0; i < n; ++i)
        canLog << ' ' << std::setw(2) << std::setfill('0') << int(p[i]);
    canLog << std::dec << '\n';
    ++frames;
}
std::array<double, 2> xy(const std::array<double, 4> &q) {
    return {195 * std::cos(q[1]) + 275 * std::cos(q[1] + q[2]),
            195 * std::sin(q[1]) + 275 * std::sin(q[1] + q[2])};
}
void onMotion(double t, const std::array<double, 4> &cmd, const std::array<double, 4> &truth,
              const std::array<double, 4> &encoder, const std::array<double, 4> &estimate) {
    motionLog << t;
    for (const auto *q : {&cmd, &truth, &encoder, &estimate})
        for (double a : *q)
            motionLog << ',' << a;
    motionLog << '\n';
    motionLog.flush(); // 交互暂停时也能读到最后一个已完成的虚拟采样。
    auto a = xy(truth), b = xy(encoder), c = xy(estimate);
    raw2 += std::pow(a[0] - b[0], 2) + std::pow(a[1] - b[1], 2);
    ekf2 += std::pow(a[0] - c[0], 2) + std::pow(a[1] - c[1], 2);
    ++samples;
    auto now = std::chrono::steady_clock::now();
    if (samples > 1 && cmd == lastCommand)
        tickGaps.push_back(std::chrono::duration<double, std::milli>(now - lastWall).count());
    lastWall = now;
    lastCommand = cmd;
}
void onEvent(const char *point, const std::string &text) {
    eventLog << point << '\t' << text << '\n';
}
void onPlacement(const std::array<double, 2> &goal, const std::array<double, 2> &actual) {
    maxPlacementError =
        std::max(maxPlacementError, std::hypot(goal[0] - actual[0], goal[1] - actual[1]));
    ++placements;
}
} // namespace
void setupDebug(const std::string &path) {
    directory = path;
    std::filesystem::create_directories(path);
    canLog.open(path + "/can.log");
    motionLog.open(path + "/motion.csv");
    eventLog.open(path + "/events.log");
    motionLog << "time,z_cmd,q1_cmd,q2_cmd,r_cmd,z_true,q1_true,q2_true,r_true,z_encoder,q1_"
                 "encoder,q2_encoder,r_encoder,z_est,q1_est,q2_est,r_est\n";
    debug = {onCan, onMotion, onEvent, onPlacement};
}
void finishDebug() {
    if (directory.empty())
        return;
    std::ofstream out(directory + "/motion-metrics.json");
    std::sort(tickGaps.begin(), tickGaps.end());
    double p95 = tickGaps.empty() ? 0 : tickGaps[size_t((tickGaps.size() - 1) * .95)];
    out << "{\"samples\":" << samples << ",\"can_frames\":" << frames
        << ",\"raw_xy_rmse_mm\":" << std::sqrt(raw2 / std::max(1, samples))
        << ",\"ekf_xy_rmse_mm\":" << std::sqrt(ekf2 / std::max(1, samples))
        << ",\"placements\":" << placements << ",\"max_placement_error_mm\":" << maxPlacementError
        << ",\"same_command_tick_p95_wall_ms\":" << p95 << "}\n";
    debug = {};
}
