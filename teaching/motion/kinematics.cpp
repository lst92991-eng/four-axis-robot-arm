#include "kinematics.hpp"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>

namespace motion {
namespace {
bool finite(double value) { return std::isfinite(value); }
double segmentDistance(double x, double y, double ax, double ay, double bx, double by)
{
    const double dx = bx - ax, dy = by - ay, length = dx * dx + dy * dy;
    const double t = length > 0 ? std::clamp(((x - ax) * dx + (y - ay) * dy) / length, 0., 1.) : 0.;
    return std::hypot(x - ax - t * dx, y - ay - t * dy);
}
void limits(const Joints& q, const Config& c)
{
    for (std::size_t i = 0; i < q.size(); ++i) {
        if (!finite(q[i]) || q[i] < c.axes[i].minimum || q[i] > c.axes[i].maximum)
            throw std::runtime_error("关节位置超出已配置的机械限位");
        const double pulses = std::abs(motorAngle(q[i], c.axes[i])) * c.axes[i].pulsesPerRev / 360.;
        if (!finite(pulses) || std::round(pulses) > std::numeric_limits<uint32_t>::max())
            throw std::runtime_error("关节目标换算后超出驱动绝对位置范围");
    }
}
} // namespace

void validate(const Config& c)
{
    if (!finite(c.l1) || !finite(c.l2) || c.l1 <= 0 || c.l2 <= 0)
        throw std::runtime_error("缺少有效臂长");
    for (std::size_t i = 0; i < c.axes.size(); ++i) {
        const auto& a = c.axes[i];
        if (a.node < 1 || a.node > 255 || (a.direction != 1 && a.direction != -1) ||
            !finite(a.motorDegPerUnit) || a.motorDegPerUnit <= 0 || !finite(a.zeroMotorDeg) ||
            !finite(a.minimum) || !finite(a.maximum) || a.minimum >= a.maximum ||
            !finite(a.tolerance) || a.tolerance <= 0 || a.tolerance >= (a.maximum - a.minimum) / 2 ||
            a.maxRpm < 1 || a.maxRpm > 3000 || a.pulsesPerRev < 200 || a.pulsesPerRev > 102400 || !a.currentLimitMa)
            throw std::runtime_error("轴地址、转换比例、限位、速度或误差阈值未配置");
        for (std::size_t j = 0; j < i; ++j)
            if (a.node == c.axes[j].node) throw std::runtime_error("四轴CAN地址不能重复");
    }
    for (double value : {c.xMinimum, c.xMaximum, c.yMinimum, c.yMaximum})
        if (!finite(value)) throw std::runtime_error("工作区边界不是有限数");
    if (c.xMinimum >= c.xMaximum || c.yMinimum >= c.yMaximum)
        throw std::runtime_error("缺少已核对的TCP工作区边界");
    for (double value : {c.linkRadius, c.toolRadius, c.minimumSinElbow, c.sampleAngle,
                        c.moveTimeoutSeconds, c.settleSeconds, c.feedbackSeconds, c.estimateErrorLimit,
                        c.accelerationNoise, c.encoderSigma, c.tcpSigmaMm, c.maximumObservationAge})
        if (!finite(value) || value <= 0) throw std::runtime_error("运动几何、时间或滤波参数必须为有限正数");
    if (c.sampleAngle > .1 || c.minimumSinElbow >= 1 || c.feedbackSeconds > .1 ||
        c.maximumObservationAge > .2 || c.moveTimeoutSeconds > 120 ||
        c.settleSeconds >= c.moveTimeoutSeconds || c.acceleration < 1 || c.acceleration > 255)
        throw std::runtime_error("运动采样、超时或加速度参数超出允许范围");
    for (const auto& obstacle : c.forbidden)
        if (!finite(obstacle.x) || !finite(obstacle.y) || !finite(obstacle.radius) || obstacle.radius <= 0 ||
            !finite(obstacle.zMinimum) || !finite(obstacle.zMaximum) || obstacle.zMinimum > obstacle.zMaximum)
            throw std::runtime_error("禁入圆柱参数无效");
}

double motorAngle(double joint, const Axis& a)
{
    return a.zeroMotorDeg + a.direction * joint * a.motorDegPerUnit;
}
double jointPosition(double motorDeg, const Axis& a)
{
    return a.direction * (motorDeg - a.zeroMotorDeg) / a.motorDegPerUnit;
}

Timing timing(const Joints& from, const Joints& to, const Config& c)
{
    Timing result;
    std::array<double, 4> distance{};
    const double acceleration = 20000. / (256 - c.acceleration);
    auto seconds = [acceleration](double d, double speed) {
        return d >= speed * speed / acceleration
            ? d / speed + speed / acceleration : 2 * std::sqrt(d / acceleration);
    };
    double common = 0;
    for (std::size_t i = 0; i < to.size(); ++i) {
        distance[i] = std::abs(to[i] - from[i]) * c.axes[i].motorDegPerUnit / 6.;
        common = std::max(common, seconds(distance[i], c.axes[i].maxRpm));
    }
    for (std::size_t i = 0; i < to.size(); ++i) {
        const double root = std::sqrt(std::max(0., common * common - 4 * distance[i] / acceleration));
        const double speed = distance[i] > 0 ? 2 * distance[i] / (common + root) : 1;
        result.rpm[i] = unsigned(std::clamp(std::round(speed), 1., double(c.axes[i].maxRpm)));
        result.seconds = std::max(result.seconds, seconds(distance[i], result.rpm[i]));
    }
    return result;
}

Pose forward(const Joints& q, const Config& c)
{
    const double ab = q[1] + q[2];
    return {c.l1 * std::cos(q[1]) + c.l2 * std::cos(ab),
            c.l1 * std::sin(q[1]) + c.l2 * std::sin(ab), q[0], std::remainder(ab + q[3], 2 * Pi)};
}
Joints inverse(const Pose& p, const Joints& nearby, const Config& c)
{
    if (!finite(p.x) || !finite(p.y) || !finite(p.z) || !finite(p.yaw))
        throw std::runtime_error("目标位姿不是有限数");
    const double cosine = (p.x * p.x + p.y * p.y - c.l1 * c.l1 - c.l2 * c.l2) / (2 * c.l1 * c.l2);
    if (!finite(cosine) || cosine < -1 - 1e-12 || cosine > 1 + 1e-12)
        throw std::runtime_error("目标超出几何可达范围");
    Joints best{};
    double cost = std::numeric_limits<double>::infinity();
    for (double sign : {1., -1.}) {
        const double b = sign * std::acos(std::clamp(cosine, -1., 1.));
        const double a = std::atan2(p.y, p.x) - std::atan2(c.l2 * std::sin(b), c.l1 + c.l2 * std::cos(b));
        Joints q{p.z, a, b, p.yaw - a - b};
        bool allowed = true;
        double candidate = 0;
        for (std::size_t i = 1; i < q.size(); ++i) {
            const double low = std::ceil((c.axes[i].minimum - q[i]) / (2 * Pi));
            const double high = std::floor((c.axes[i].maximum - q[i]) / (2 * Pi));
            if (low > high) { allowed = false; break; }
            q[i] += 2 * Pi * std::clamp(std::round((nearby[i] - q[i]) / (2 * Pi)), low, high);
            candidate += (q[i] - nearby[i]) * (q[i] - nearby[i]);
        }
        // 不在一次搬运中跳到另一肘型；通过伸直/折叠奇异点需要另行规划。
        if (allowed && q[0] >= c.axes[0].minimum && q[0] <= c.axes[0].maximum &&
            std::sin(q[2]) * std::sin(nearby[2]) > 0 && std::abs(std::sin(q[2])) >= c.minimumSinElbow &&
            candidate < cost) { best = q; cost = candidate; }
    }
    if (!finite(cost)) throw std::runtime_error("没有满足限位与当前肘型的逆解");
    return best;
}

void checkPath(const Joints& from, const Joints& to, const Config& c)
{
    validate(c);
    limits(from, c); limits(to, c);
    if (timing(from, to, c).seconds + c.settleSeconds + .25 >= c.moveTimeoutSeconds)
        throw std::runtime_error("运动预计耗时超过配置上限，请调整已验证参数或分段");
    const double a0 = std::min(from[1], to[1]), b0 = std::min(from[2], to[2]);
    const double da = std::abs(from[1] - to[1]), db = std::abs(from[2] - to[2]);
    const double samplesA = std::max(1., std::ceil(da / c.sampleAngle));
    const double samplesB = std::max(1., std::ceil(db / c.sampleAngle));
    if (!finite(samplesA * samplesB) || samplesA * samplesB > 25000)
        throw std::runtime_error("运动关节包络过大，请用经验证的中间点分段");
    const int na = int(samplesA), nb = int(samplesB);
    // 原生FD各轴加速度量化后不保证同一进度，检查其笛卡尔积包络而非仅理想直线。
    const double ha = da / (2 * na) + c.axes[1].tolerance;
    const double hb = db / (2 * nb) + c.axes[2].tolerance;
    const double firstMargin = c.l1 * ha, secondMargin = firstMargin + c.l2 * (ha + hb);
    const double tcpMargin = secondMargin + c.toolRadius;
    const double zLow = std::min(from[0], to[0]) - c.axes[0].tolerance;
    const double zHigh = std::max(from[0], to[0]) + c.axes[0].tolerance;
    for (int ia = 0; ia < na; ++ia) {
        for (int ib = 0; ib < nb; ++ib) {
            const double a = a0 + (ia + .5) * da / na, b = b0 + (ib + .5) * db / nb;
            if (std::abs(std::sin(b)) - hb < c.minimumSinElbow)
                throw std::runtime_error("路径经过伸直/折叠奇异区或配置的自碰限制");
            const double ex = c.l1 * std::cos(a), ey = c.l1 * std::sin(a);
            const double x = ex + c.l2 * std::cos(a + b), y = ey + c.l2 * std::sin(a + b);
            if (x - tcpMargin < c.xMinimum || x + tcpMargin > c.xMaximum ||
                y - tcpMargin < c.yMinimum || y + tcpMargin > c.yMaximum)
                throw std::runtime_error("运动包络超出已配置的工作区");
            for (const auto& obstacle : c.forbidden) {
                // 连杆采用全高度平面包络，偏保守；TCP另使用实际Z区间。
                if (segmentDistance(obstacle.x, obstacle.y, 0, 0, ex, ey) <= obstacle.radius + c.linkRadius + firstMargin ||
                    segmentDistance(obstacle.x, obstacle.y, ex, ey, x, y) <= obstacle.radius + c.linkRadius + secondMargin ||
                    (zLow <= obstacle.zMaximum && zHigh >= obstacle.zMinimum &&
                     std::hypot(x - obstacle.x, y - obstacle.y) <= obstacle.radius + tcpMargin))
                    throw std::runtime_error("运动包络进入配置的固定障碍区域");
            }
        }
    }
}

} // namespace motion
