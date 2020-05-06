#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace ugl::math
{

template<int rows, int cols>
using Matrix = Eigen::Matrix<double, rows, cols>;

using Matrix3 = Matrix<3, 3>;
using Rotation = Matrix<3, 3>;

}

// Expose basic types in the ugl namespace.
namespace ugl
{

template<int rows, int cols>
using Matrix = math::Matrix<rows, cols>;

using Matrix3 = math::Matrix3;
using Rotation = math::Rotation;

}
