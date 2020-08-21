#ifndef UGL_MATH_SLERP_H
#define UGL_MATH_SLERP_H

#include <cassert>

#include <unsupported/Eigen/MatrixFunctions>

#include "ugl/math/quaternion.h"

namespace ugl::math
{

/// Calculates the spherical linear interpolation between two quaternions at time t.
UnitQuaternion slerp(const UnitQuaternion& q0, const UnitQuaternion& q1, double t);

} // namespace ugl::math

#endif // UGL_MATH_SLERP_H
