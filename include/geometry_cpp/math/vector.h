#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace geometry::math
{

template<int rows>
using Vector = Eigen::Matrix<double, rows, 1>;

using Vector3 = Vector<3>;

}
