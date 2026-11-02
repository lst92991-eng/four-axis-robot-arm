#pragma once

namespace scara {

// 装配实测的转轴中心距，单位 mm。
inline constexpr double L1 = 195.0;
inline constexpr double L2 = 275.0;
inline constexpr double Pi = 3.14159265358979323846;

// 角度用弧度：从 +Z 看向 XY 平面，逆时针为正。
// shoulder 相对 +X；elbow 相对第一臂；wrist 相对第二臂。
// z 为末端相对约定零平面的高度，向上为正，不是丝杆电机角度。
struct Joints {
    double shoulder, elbow, z, wrist;
};

// 原点的 XY 位于肩轴；工具中心须在末端旋转轴上。
struct Pose {
    double x, y, z, yaw;
};

// 输入有限的关节量。仅计算理想几何，不判断限位或碰撞。
Pose forward(const Joints& joints);

} // namespace scara
