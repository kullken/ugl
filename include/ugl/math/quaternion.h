#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "ugl/math/vector.h"

namespace ugl
{
inline namespace math
{

using Quaternion = Eigen::Quaternion<double>;
using UnitQuaternion = Eigen::Quaternion<double>;

} // namespace math

namespace math
{

/// Calculates the exponential of an imaginary-only quaternion.
UnitQuaternion exp(const Quaternion& q);

/// Calculates the natural logarithm of a unit-norm quaternion.
Quaternion log(const UnitQuaternion& q);

// Creates a unit-norm quaternion from a angle-axis pair.
//  angle is in radians.
//  axis is of unit length.
UnitQuaternion to_quat(double angle, Vector3 axis)
{
    return UnitQuaternion(Eigen::AngleAxisd(angle, axis));
}

} // namespace math
} // namespace ugl
