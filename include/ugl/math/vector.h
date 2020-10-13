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

using VectorD = Vector<Eigen::Dynamic>;

template <int cols>
using RowVector = Eigen::Matrix<double, 1, cols, Eigen::DontAlign|Eigen::RowMajor>;

using RowVectorD = RowVector<Eigen::Dynamic>;

using Vector3 = Vector<3>;

} // namespace math

template <int rows>
using Vector = math::Vector<rows>;

using VectorD = math::VectorD;

template <int cols>
using RowVector = math::RowVector<cols>;

using RowVectorD = math::RowVectorD;

using Vector3 = math::Vector3;

} // namespace ugl

#endif // UGL_MATH_VECTOR_H
