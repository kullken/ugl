#ifndef UGL_LIE_JACOBIAN_HELPER_H
#define UGL_LIE_JACOBIAN_HELPER_H

#include "ugl/math/vector.h"
#include "ugl/math/matrix.h"

namespace ugl::lie::internal
{

/// @brief The matrix-block Q used for calculating the Jacobian
/// @param phi linearised rotation
/// @param rho linearised position
/// @return The 3-by-3 Q matrix
Matrix3 jac_Q_block(const Vector3& phi, const Vector3& rho);

} // namespace ugl::lie::internal

#endif // UGL_LIE_JACOBIAN_HELPER_H
