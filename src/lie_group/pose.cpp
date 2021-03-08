#include "ugl/lie_group/pose.h"

#include "ugl/math/vector.h"
#include "ugl/math/matrix.h"

#include "ugl/lie_group/rotation.h"
#include "jacobian_helper.h"

namespace ugl::lie
{

Vector3 Pose::transform(const Vector3& vec) const
{
    return R_*vec + pos_;
}

Pose Pose::exp(const Vector<6>& u)
{
    const Vector3& phi = u.segment<3>(0);
    const Vector3& rho = u.segment<3>(3);
    const Matrix3 J = SO3::left_jacobian(phi);
    return Pose{SO3::exp(phi), J * rho};
}

Vector<6> Pose::log(const Pose& T)
{
    const Vector3 phi = SO3::log(T.R_);
    const Vector3& p = T.pos_;
    const Matrix3 Jinv = SO3::left_jacobian_inv(phi);
    Vector<6> result;
    result << phi, Jinv * p;
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

Matrix<6,6> Pose::adjoint(const Pose& T)
{
    const Matrix3& R = T.R_.matrix();
    const Vector3& p = T.pos_;

    Matrix<6,6> Adj = Matrix<6,6>::Zero();
    Adj.block<3,3>(0,0) = R;
    Adj.block<3,3>(3,3) = R;
    Adj.block<3,3>(3,0) = SO3::hat(p) * R;

    return Adj;
}

Matrix<6,6> Pose::left_jacobian(const Vector<6>& tau)
{
    const Vector3 phi = tau.segment<3>(0);
    const Vector3 rho = tau.segment<3>(3);
    const Matrix3 Q_rho = internal::jac_Q_block(phi, rho);
    const Matrix3 J_SO3 = SO3::left_jacobian(phi);

    Matrix<6,6> J = Matrix<6,6>::Zero();
    J.block<3,3>(0,0) = J_SO3;
    J.block<3,3>(3,3) = J_SO3;
    J.block<3,3>(3,0) = Q_rho;

    return J;
}

Matrix<6,6> Pose::left_jacobian_inv(const Vector<6>& tau)
{
    const Vector3 phi = tau.segment<3>(0);
    const Vector3 rho = tau.segment<3>(3);
    const Matrix3 Q_rho = internal::jac_Q_block(phi, rho);
    const Matrix3 J_SO3_inv = SO3::left_jacobian_inv(phi);

    Matrix<6,6> J_inv = Matrix<6,6>::Zero();
    J_inv.block<3,3>(0,0) = J_SO3_inv;
    J_inv.block<3,3>(3,3) = J_SO3_inv;
    J_inv.block<3,3>(3,0) = -J_SO3_inv * Q_rho * J_SO3_inv;

    return J_inv;
}

Matrix<6,6> Pose::right_jacobian(const Vector<6>& tau)
{
    return left_jacobian(-tau);
}

Matrix<6,6> Pose::right_jacobian_inv(const Vector<6>& tau)
{
    return left_jacobian_inv(-tau);
}

} // namespace ugl::lie
