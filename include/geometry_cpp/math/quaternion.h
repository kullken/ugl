#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace geometry::math
{

using Quaternion = Eigen::Quaternion<double>;
using UnitQuaternion = Eigen::Quaternion<double>;

/// Calculates the exponential of an imaginary-only quaternion.
UnitQuaternion exp(const Quaternion& q);

/// Calculates the natural logarithm of a unit-norm quaternion.
Quaternion log(const UnitQuaternion& q);

}
