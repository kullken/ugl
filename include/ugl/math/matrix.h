#ifndef UGL_MATH_MATRIX_H
#define UGL_MATH_MATRIX_H

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace ugl
{
namespace math
{

template <int rows, int cols>
using Matrix = Eigen::Matrix<double, rows, cols, Eigen::DontAlign | (rows==1 ? Eigen::RowMajor : 0)>;

using MatrixD = Matrix<Eigen::Dynamic, Eigen::Dynamic>;

using Matrix3 = Matrix<3, 3>;

/// @brief Compute the matrix exponential.
/// @param m matrix whose exponential is to be computed
/// @return The matrix exponential of m.
MatrixD exp(const MatrixD& m);

/// @brief Compute the matrix logarithm.
/// @param m matrix whose logarithm is to be computed
/// @return The matrix logarithm of m.
MatrixD log(const MatrixD& m);

} // namespace math

template <int rows, int cols>
using Matrix = math::Matrix<rows, cols>;

using MatrixD = math::MatrixD;

using Matrix3 = math::Matrix3;

} // namespace ugl

#endif // UGL_MATH_MATRIX_H
