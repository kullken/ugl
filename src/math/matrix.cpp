#include "ugl/math/matrix.h"

#include <unsupported/Eigen/MatrixFunctions>

namespace ugl::math
{

MatrixD exp(const MatrixD& m)
{
    return m.exp();
}

MatrixD log(const MatrixD& m)
{
    return m.log();
}

} // namespace ugl::math
