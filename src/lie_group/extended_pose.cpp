#include "ugl/lie_group/extended_pose.h"

#include "ugl/math/vector.h"
#include "ugl/math/matrix.h"

#include "ugl/lie_group/rotation.h"
#include "jacobian_helper.h"

namespace ugl::lie
{

ExtendedPose ExtendedPose::exp(const Vector<9>& u)
{
    const Vector3& phi = u.segment<3>(0);
    const Vector3& nu  = u.segment<3>(3);
    const Vector3& rho = u.segment<3>(6);
    const Matrix3 J = SO3::left_jacobian(phi);
    return ExtendedPose{SO3::exp(phi), J * nu, J * rho};
}

Vector<9> ExtendedPose::log(const ExtendedPose& T)
{
    const Vector3 phi = SO3::log(T.R_);
    const Vector3& v = T.vel_;
    const Vector3& p = T.pos_;
    const Matrix3 Jinv = SO3::left_jacobian_inv(phi);
    Vector<9> result;
    result << phi, Jinv * v, Jinv * p;
    return result;
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
    const Vector3& v = T.vel_;
    const Vector3& p = T.pos_;

    Matrix<9,9> Adj = Matrix<9,9>::Zero();
    Adj.block<3,3>(0,0) = R;
    Adj.block<3,3>(3,3) = R;
    Adj.block<3,3>(6,6) = R;
    Adj.block<3,3>(3,0) = SO3::hat(v) * R;
    Adj.block<3,3>(6,0) = SO3::hat(p) * R;

    return Adj;
}

Matrix<9,9> ExtendedPose::left_jacobian(const Vector<9>& tau)
{
    const Vector3 phi = tau.segment<3>(0);
    const Vector3 nu  = tau.segment<3>(3);
    const Vector3 rho = tau.segment<3>(3);
    const Matrix3 Q_nu  = internal::jac_Q_block(phi, nu);
    const Matrix3 Q_rho = internal::jac_Q_block(phi, rho);
    const Matrix3 J_SO3 = SO3::left_jacobian(phi);

    Matrix<9,9> J = Matrix<9,9>::Zero();
    J.block<3,3>(0,0) = J_SO3;
    J.block<3,3>(3,3) = J_SO3;
    J.block<3,3>(6,6) = J_SO3;
    J.block<3,3>(3,0) = Q_nu;
    J.block<3,3>(6,0) = Q_rho;

    return J;
}

Matrix<9,9> ExtendedPose::left_jacobian_inv(const Vector<9>& tau)
{
    const Vector3 phi = tau.segment<3>(0);
    const Vector3 nu  = tau.segment<3>(3);
    const Vector3 rho = tau.segment<3>(3);
    const Matrix3 Q_nu  = internal::jac_Q_block(phi, nu);
    const Matrix3 Q_rho = internal::jac_Q_block(phi, rho);
    const Matrix3 J_SO3_inv = SO3::left_jacobian_inv(phi);

    Matrix<9,9> J_inv = Matrix<9,9>::Zero();
    J_inv.block<3,3>(0,0) = J_SO3_inv;
    J_inv.block<3,3>(3,3) = J_SO3_inv;
    J_inv.block<3,3>(6,6) = J_SO3_inv;
    J_inv.block<3,3>(3,0) = -J_SO3_inv * Q_nu * J_SO3_inv;
    J_inv.block<3,3>(6,0) = -J_SO3_inv * Q_rho * J_SO3_inv;

    return J_inv;
}

Matrix<9,9> ExtendedPose::right_jacobian(const Vector<9>& tau)
{
    return left_jacobian(-tau);
}

Matrix<9,9> ExtendedPose::right_jacobian_inv(const Vector<9>& tau)
{
    return left_jacobian_inv(-tau);
}

bool operator==(const ExtendedPose& lhs, const ExtendedPose& rhs)
{
    return lhs.R_ == rhs.R_ && lhs.vel_ == rhs.vel_ && lhs.pos_ == rhs.pos_;
}

bool operator!=(const ExtendedPose& lhs, const ExtendedPose& rhs)
{
    return lhs.R_ != rhs.R_ || lhs.vel_ != rhs.vel_ || lhs.pos_ != rhs.pos_;
}

} // namespace ugl::lie
