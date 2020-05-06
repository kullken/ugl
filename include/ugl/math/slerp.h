#pragma once

#include <cassert>

#include <unsupported/Eigen/MatrixFunctions>

#include "ugl/math/quaternion.h"

namespace ugl::math
{

/// Calculates the spherical linear interpolation between two quaternions at time t.
inline
UnitQuaternion slerp(const UnitQuaternion& q0, const UnitQuaternion& q1, double t)
{
    assert(0.0 <= t && t <= 1.0);
    return q0.slerp(t, q1);
}

}
