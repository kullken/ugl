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
