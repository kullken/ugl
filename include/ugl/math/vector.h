#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace ugl::math
{

template<int rows>
using Vector = Eigen::Matrix<double, rows, 1>;

using Vector3 = Vector<3>;

}

// Expose basic types in the ugl namespace.
namespace ugl
{

template<int rows>
using Vector = math::Vector<rows>;

using Vector3 = math::Vector3;

}
