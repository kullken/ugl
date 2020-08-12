#include "ugl/lie_group/extended_pose.h"

#include "ugl/math/vector.h"
#include "ugl/math/matrix.h"

#include "ugl/lie_group/rotation.h"

namespace ugl::lie
{

ExtendedPose ExtendedPose::exp(const Vector<9>& u)
{
    const Vector3 phi = u.segment<3>(0);
    const Vector3 nu  = u.segment<3>(3);
    const Vector3 rho = u.segment<3>(6);
    
    constexpr double kTolerance = 1e-10;
    const double phi_norm = phi.norm();
    if (phi_norm < kTolerance) {
        return ExtendedPose{SO3::Identity(), nu, rho};
    }

    const Matrix3 A = SO3::left_jacobian(phi);
    return ExtendedPose{SO3::exp(phi), A * nu, A * rho};
}

Vector<9> ExtendedPose::log(const ExtendedPose& T)
{
    // TODO: Analytical solution
    return vee(math::log(T.matrix()));
}

se2_3 ExtendedPose::hat(const Vector<9>& u)
{
    se2_3 U = se2_3::Zero();
    U.block<3,3>(0,0) = SO3::hat(u.segment<3>(0));
    U.block<3,1>(0,3) = u.segment<3>(3);
    U.block<3,1>(0,4) = u.segment<3>(6);
    return U;
}

Vector<9> ExtendedPose::vee(const se2_3& U)
{
    Vector<9> u;
    u << SO3::vee(U.block<3,3>(0,0)), U.block<3,1>(0,3), U.block<3,1>(0,4);
    return u;
}

Matrix<9,9> ExtendedPose::adjoint(const ExtendedPose& T)
{
    const Matrix3& R = T.R_.matrix();
    const Vector3&  v = T.vel_;
    const Vector3&  p = T.pos_;

    Matrix<9,9> Adj = Matrix<9,9>::Zero();
    Adj.block<3,3>(0,0) = R;
    Adj.block<3,3>(3,3) = R;
    Adj.block<3,3>(6,6) = R;
    Adj.block<3,3>(3,0) = SO3::hat(v) * R;
    Adj.block<3,3>(6,0) = SO3::hat(p) * R;

    return Adj;
}

} // namespace ugl::lie
