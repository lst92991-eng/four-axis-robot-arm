#pragma once
#include "fk.hpp"

namespace scara {

// positiveElbow=true 取 [0, pi] 的肘角，false 取 [-pi, 0]。
// 两种分支分别调用即可；伸直或完全折叠时两分支几何上重合。
// 非有限输入或 XY 不可达返回 false，且不修改 joints。
// 输出肩角、腕角归一化到 [-pi, pi]；不处理电机圈数、限位和碰撞。
bool inverse(const Pose& target, bool positiveElbow, Joints& joints);

} // namespace scara
