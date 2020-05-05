#ifndef _GEOMETRY_MATH_TYPES_H
#define _GEOMETRY_MATH_TYPES_H

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace geometry::math
{

template<int rows>
using Vector = Eigen::Matrix<double, rows, 1>;

template<int rows, int cols>
using Matrix = Eigen::Matrix<double, rows, cols>;

using Vector3 = Eigen::Matrix<double, 3, 1>;
using Matrix3 = Eigen::Matrix<double, 3, 3>;

using Rotation = Eigen::Matrix<double, 3, 3>;
using UnitQuaternion = Eigen::Quaternion<double>;

}

#endif