#ifndef UGL_MATH_VECTOR_H
#define UGL_MATH_VECTOR_H

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace ugl
{
namespace math
{

template <int rows>
using Vector = Eigen::Matrix<double, rows, 1, Eigen::DontAlign>;

using Vector3 = Vector<3>;

} // namespace math

template <int rows>
using Vector = math::Vector<rows>;

using Vector3 = math::Vector3;

} // namespace ugl

#endif // UGL_MATH_VECTOR_H
