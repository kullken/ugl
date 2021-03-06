#include "ugl/math/quaternion.h"

#include <cassert>
#include <cmath>

#include "ugl/math/vector.h"

namespace ugl::math
{

UnitQuaternion exp(const Quaternion& q)
{
    [[maybe_unused]] constexpr double tolerance = 1e-6;
    assert(std::abs(q.w()) < tolerance); // Imaginary quaternion required.
    Vector3 u = q.vec();
    double u_norm = u.norm();
    Vector3 v = u.normalized() * std::sin(u_norm);
    return Quaternion(std::cos(u_norm), v.x(), v.y(), v.z());
}

Quaternion log(const UnitQuaternion& q)
{
    [[maybe_unused]] constexpr double tolerance = 1e-6;
    assert(std::abs(q.norm() - 1) < tolerance); // Unit-norm quaternion required.
    Vector3 u = q.vec();
    Vector3 v = u.normalized() * std::acos(q.w());
    return Quaternion(0, v.x(), v.y(), v.z());
}

}