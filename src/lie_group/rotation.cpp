#include "ugl/lie_group/rotation.h"

#include <cmath>

#include "ugl/math/vector.h"
#include "ugl/math/matrix.h"

namespace ugl::lie
{

Rotation Rotation::exp(const ugl::Vector3& w)
{
    constexpr double kTolerance = 1e-10;
    const double theta = w.norm();
    if (theta < kTolerance) {
        return Rotation::Identity();
    }
    const so3 S = hat(w);
    const Rotation R{ugl::Matrix3::Identity() + std::sin(theta)/theta * S + ((1-cos(theta))/(theta*theta))*S*S};
    return R;
}

ugl::Vector3 Rotation::log(const Rotation& R)
{
    constexpr double kTolerance = 1e-10;
    // TODO: Use std::clamp(*, -1.0, 1.0) on cos_theta to avoid potential NaN:s when using acos() later on?
    const double cos_theta = 0.5 * R.matrix_.trace() - 0.5;
    if ((cos_theta - 1.0) > -kTolerance) {
        return vee(R.matrix_ - ugl::Matrix3::Identity());
    }
    // TODO: Do we need another special case when theta is close to pi (i.e. cos_theta is close to -1)?

    const double theta = std::acos(cos_theta);
    return 0.5 * theta / std::sin(theta) * vee(R.matrix_ - R.matrix_.transpose());
}

so3 Rotation::hat(const ugl::Vector3& w)
{
    ugl::Matrix3 S;
    S << 0.0, -w(2), w(1),
         w(2), 0.0, -w(0),
        -w(1), w(0), 0.0;
    return S;
}

ugl::Vector3 Rotation::vee(const so3& S)
{
    return ugl::Vector3{S(2,1)-S(1,2), S(0,2)-S(2,0), S(1,0)-S(0,1)} * 0.5;
}

ugl::Matrix3 Rotation::left_jacobian(const ugl::Vector3& phi)
{
    constexpr double kTolerance = 1e-10;
    const double phi_norm = phi.norm();
    if (phi_norm < kTolerance) {
        return Matrix3::Identity() + 0.5 * hat(phi);
    }
    const Matrix3 W = hat(phi / phi_norm);
    return Matrix3::Identity() + ((1 - std::cos(phi_norm)) / phi_norm) * W + ((phi_norm - std::sin(phi_norm)) / phi_norm) * W*W;
}

ugl::Matrix3 Rotation::left_jacobian_inv(const ugl::Vector3& phi)
{
    constexpr double kTolerance = 1e-10;
    const double phi_norm = phi.norm();
    if (phi_norm < kTolerance) {
        return Matrix3::Identity() - 0.5 * hat(phi);
    }
    const Matrix3 W = hat(phi / phi_norm);
    const double phi_norm_half = phi_norm / 2;
    return Matrix3::Identity() - (phi_norm_half) * W + (1 - ((phi_norm_half) / std::tan(phi_norm_half))) * W*W;
}

ugl::Matrix3 Rotation::right_jacobian(const ugl::Vector3& phi)
{
    return left_jacobian(-phi);
}

ugl::Matrix3 Rotation::right_jacobian_inv(const ugl::Vector3& phi)
{
    return left_jacobian_inv(-phi);
}

} // namespace ugl::lie
