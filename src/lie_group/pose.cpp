#include "ugl/lie_group/pose.h"

#include "ugl/math/vector.h"
#include "ugl/math/matrix.h"

#include "ugl/lie_group/rotation.h"

namespace ugl::lie
{

Pose Pose::exp(const Vector<6>& u)
{
    const Vector3 phi = u.segment<3>(0);
    const Vector3 rho = u.segment<3>(3);
    
    constexpr double kTolerance = 1e-10;
    const double phi_norm = phi.norm();
    if (phi_norm < kTolerance) {
        return Pose{SO3::Identity(), rho};
    }

    const Matrix3 J = SO3::left_jacobian(phi);
    return Pose{SO3::exp(phi), J * rho};
}

Vector<6> Pose::log(const Pose& T)
{
    const Vector3 phi = SO3::log(T.R_);
    const Vector3& p = T.pos_;
    
    constexpr double kTolerance = 1e-10;
    const double phi_norm = phi.norm();
    // TODO: Pre-allocate result variable since it is used in both branches?
    if (phi_norm < kTolerance)
    {
        Vector<6> result;
        result << phi, p;
        return result;
    }

    const Matrix3 Jinv = SO3::left_jacobian_inv(phi);
    // TODO: Skip temp. variable rho?
    const Vector3 rho = Jinv * p;
    Vector<6> result;
    result << phi, rho;
    return result;
}

se_3 Pose::hat(const Vector<6>& u)
{
    se_3 U = se_3::Zero();
    U.block<3,3>(0,0) = SO3::hat(u.segment<3>(0));
    U.block<3,1>(0,3) = u.segment<3>(3);
    return U;
}

Vector<6> Pose::vee(const se_3& U)
{
    Vector<6> u;
    u << SO3::vee(U.block<3,3>(0,0)), U.block<3,1>(0,3);
    return u;
}

} // namespace ugl::lie
