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

Matrix<3, 3> exp(const Matrix<3, 3> &m);
Matrix<4, 4> exp(const Matrix<4, 4> &m);
Matrix<5, 5> exp(const Matrix<5, 5> &m);
Matrix<9, 9> exp(const Matrix<9, 9> &m);

Matrix<3, 3> log(const Matrix<3, 3> &m);
Matrix<4, 4> log(const Matrix<4, 4> &m);
Matrix<5, 5> log(const Matrix<5, 5> &m);
Matrix<9, 9> log(const Matrix<9, 9> &m);

} // namespace math

template <int rows, int cols>
using Matrix = math::Matrix<rows, cols>;

using MatrixD = math::MatrixD;

using Matrix3 = math::Matrix3;

} // namespace ugl

#endif // UGL_MATH_MATRIX_H
