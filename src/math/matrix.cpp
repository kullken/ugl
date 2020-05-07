#include "ugl/math/matrix.h"

#include <unsupported/Eigen/MatrixFunctions>

namespace ugl::math
{

Matrix<3,3> exp(const Matrix<3,3>& m)
{
    return m.exp();
}

Matrix<4,4> exp(const Matrix<4,4>& m)
{
    return m.exp();
}

Matrix<5,5> exp(const Matrix<5,5>& m)
{
    return m.exp();
}

Matrix<9,9> exp(const Matrix<9,9>& m)
{
    return m.exp();
}


Matrix<3,3> log(const Matrix<3,3>& m)
{
    return m.log();
}

Matrix<4,4> log(const Matrix<4,4>& m)
{
    return m.log();
}

Matrix<5,5> log(const Matrix<5,5>& m)
{
    return m.log();
}

Matrix<9,9> log(const Matrix<9,9>& m)
{
    return m.log();
}

}
