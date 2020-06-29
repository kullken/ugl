#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace ugl
{
inline namespace math
{

template <int rows, int cols>
using Matrix = Eigen::Matrix<double, rows, cols>;

using Matrix3 = Matrix<3, 3>;
using Rotation = Matrix<3, 3>;

} // namespace math

namespace math
{

Matrix<3, 3> exp(const Matrix<3, 3> &m);
Matrix<4, 4> exp(const Matrix<4, 4> &m);
Matrix<5, 5> exp(const Matrix<5, 5> &m);
Matrix<9, 9> exp(const Matrix<9, 9> &m);

Matrix<3, 3> log(const Matrix<3, 3> &m);
Matrix<4, 4> log(const Matrix<4, 4> &m);
Matrix<5, 5> log(const Matrix<5, 5> &m);
Matrix<9, 9> log(const Matrix<9, 9> &m);

} // namespace math
} // namespace ugl
