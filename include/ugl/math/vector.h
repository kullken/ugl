#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace ugl
{
inline namespace math
{

template <int rows>
using Vector = Eigen::Matrix<double, rows, 1>;

using Vector3 = Vector<3>;

} // namespace math
} // namespace ugl
