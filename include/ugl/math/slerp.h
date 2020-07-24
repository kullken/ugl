#pragma once

#include <cassert>

#include <unsupported/Eigen/MatrixFunctions>

#include "ugl/math/quaternion.h"

namespace ugl::math
{

/// Calculates the spherical linear interpolation between two quaternions at time t.
UnitQuaternion slerp(const UnitQuaternion& q0, const UnitQuaternion& q1, double t);

} // namespace ugl::math
