#include "jacobian_helper.h"

#include <cmath>

#include "ugl/math/vector.h"
#include "ugl/math/matrix.h"

#include "ugl/lie_group/rotation.h"

namespace ugl::lie::internal
{

Matrix3 jac_Q_block(const Vector3& phi, const Vector3& rho)
{
    constexpr double kTolerance = 1e-10;
    const double phi_norm = phi.norm();
    if (phi_norm < kTolerance) {
        return 0.5 * SO3::hat(rho);
    }

    const double sin_phi = std::sin(phi_norm);
    const double cos_phi = std::cos(phi_norm);
    const Matrix3 phi_hat = SO3::hat(phi);
    const Matrix3 rho_hat = SO3::hat(rho);

    const double scalar1 = 0.5;
    const double scalar2 = (phi_norm - sin_phi) / (phi_norm*phi_norm*phi_norm);
    const double scalar3 = (0.5*phi_norm*phi_norm + cos_phi - 1) / (phi_norm*phi_norm*phi_norm*phi_norm);
    const double scalar4 = (phi_norm - 1.5*sin_phi + 0.5*phi_norm*cos_phi) / (phi_norm*phi_norm*phi_norm*phi_norm*phi_norm);

    const Matrix3 term1 = rho_hat;
    const Matrix3 term2 = phi_hat*rho_hat + rho_hat*phi_hat + phi_hat*rho_hat*phi_hat;
    const Matrix3 term3 = phi_hat*phi_hat*rho_hat + rho_hat*phi_hat*phi_hat + 3*phi_hat*rho_hat*phi_hat;
    const Matrix3 term4 = phi_hat*phi_hat*rho_hat*phi_hat + phi_hat*rho_hat*phi_hat*phi_hat;

    return scalar1*term1 + scalar2*term2 + scalar3*term3 + scalar4*term4;
}

} // namespace ugl::lie::internal
