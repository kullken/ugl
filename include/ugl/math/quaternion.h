#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

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

} // namespace math
} // namespace ugl
