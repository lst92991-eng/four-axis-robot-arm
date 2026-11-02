#include "fk.hpp"
#include <cmath>

namespace scara {

Pose forward(const Joints& joints)
{
    const double a = joints.shoulder;
    const double b = a + joints.elbow;
    return {
        L1 * std::cos(a) + L2 * std::cos(b),
        L1 * std::sin(a) + L2 * std::sin(b),
        joints.z,
        std::remainder(b + joints.wrist, 2.0 * Pi)
    };
}

} // namespace scara
