#include "lie_group/mappings.h"

#include <cassert>
#include <cmath>

#include "ugl/math/vector.h"
#include "ugl/math/matrix.h"


namespace ugl::lie
{

Matrix3 skew(const Vector3& w)
{
    Matrix3 S;
    S << 0.0, -w(2), w(1),
         w(2), 0.0, -w(0),
        -w(1), w(0), 0.0;
    return S;
}

Vector3 unskew(const Matrix3& S)
{
    // constexpr double tolerance = 1e-6;
    // double trace = S.trace();
    // ROS_WARN_STREAM_COND(trace > tolerance, "Matrix to unskew has high trace: " << trace);
    // ROS_WARN_STREAM_COND(S(2,1) + S(1,2) > tolerance, "Matrix to unskew is not skew-symmetric in w(0): " << S(2,1) + S(1,2));
    // ROS_WARN_STREAM_COND(S(0,2) + S(2,0) > tolerance, "Matrix to unskew is not skew-symmetric in w(1): " << S(0,2) + S(2,0));
    // ROS_WARN_STREAM_COND(S(1,0) + S(0,1) > tolerance, "Matrix to unskew is not skew-symmetric in w(2): " << S(1,0) + S(0,1));

    // if (trace > tolerance || S(2,1) + S(1,2) > tolerance || S(0,2) + S(2,0)  > tolerance || S(1,0) + S(0,1) > tolerance)
    // {
    //     ROS_WARN_STREAM("Matrix:\n" << S);
    // }

    return Vector3(S(2,1)-S(1,2), S(0,2)-S(2,0), S(1,0)-S(0,1)) * 0.5;
    // return Vector3(S(2,1)-S(1,2), S(0,2)-S(2,0), S(1,0)-S(0,1)) * 0.6;
}

Matrix3 exp_map_SO_3(const Vector3& w)
{
    // const double phi = w.norm();
    // const Matrix3 S = skew(w);
    // const Matrix3 I = Matrix3::Identity();

    // Matrix3 R = I + (std::sin(phi)/phi) * S + ((1 - std::cos(phi))/(phi*phi)) * S*S;
    // return R;
    return math::exp(skew(w));
}

Vector<6> log_map_SE_3(const Matrix<4,4>& U)
{
    Matrix<4,4> log_U = math::log(U);
    Vector<6> u;
    u.segment<3>(0) = unskew(log_U.block<3,3>(0,0));
    u.segment<3>(3) = log_U.block<3,1>(0,3);
    return u;
}

Matrix<5,5> hat_map_se2_3(const Vector<9>& u)
{
    Matrix<5,5> mat = Matrix<5,5>::Zero();
    mat.block<3,3>(0,0) = skew(u.segment<3>(0));
    mat.block<3,1>(0,3) = u.segment<3>(3);
    mat.block<3,1>(0,4) = u.segment<3>(6);
    return mat;
}

/// Based on Barrau17a-Invariant_EKF_stable_observer and Barfoot17-State_Estimation_for_Robotics
Matrix<5,5> exp_map_SE2_3(const Vector<9>& zeta)
{
    const double phi = zeta.segment<3>(0).norm();
    const Matrix<5,5> S = hat_map_se2_3(zeta);
    const Matrix<5,5> I = Matrix<5,5>::Identity();

    return I + S + (1 - std::cos(phi))/(phi*phi) * S*S + (phi - std::sin(phi))/(phi*phi*phi) * S*S*S;
}

Vector<9> log_map_SE2_3(const Matrix<5,5>& U)
{
    Matrix<5,5> log_U = math::log(U);
    Vector<9> u;
    u.segment<3>(0) = unskew(log_U.block<3,3>(0,0));
    u.segment<3>(3) = log_U.block<3,1>(0,3);
    u.segment<3>(6) = log_U.block<3,1>(0,4);
    return u;
}

// Matrix<9,9> Adjoint_SE2_3(const State& X)
// {
//     const Rotation& R = X.get_rot();
//     const Vector3&  p = X.get_pos();
//     const Vector3&  v = X.get_vel();

//     Matrix<9,9> Adj = Matrix<9,9>::Zero();
//     Adj.block<3,3>(0,0) = R;
//     Adj.block<3,3>(3,3) = R;
//     Adj.block<3,3>(6,6) = R;
//     Adj.block<3,3>(3,0) = skew(p) * R;
//     Adj.block<3,3>(6,0) = skew(v) * R;

//     return Adj;
// }


}