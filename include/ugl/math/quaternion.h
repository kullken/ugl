#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace ugl::math
{

using Quaternion = Eigen::Quaternion<double>;
using UnitQuaternion = Eigen::Quaternion<double>;

/// Calculates the exponential of an imaginary-only quaternion.
UnitQuaternion exp(const Quaternion& q);

/// Calculates the natural logarithm of a unit-norm quaternion.
Quaternion log(const UnitQuaternion& q);

}

// Expose basic types in the ugl namespace.
namespace ugl
{

using Quaternion = math::Quaternion;
using UnitQuaternion = math::UnitQuaternion;

}
