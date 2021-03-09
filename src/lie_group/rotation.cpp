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
        return Rotation{ugl::Matrix3::Identity() + hat(w)};
    }
    const so3 S = hat(w);
    return Rotation{ugl::Matrix3::Identity() + std::sin(theta)/theta * S + ((1-cos(theta))/(theta*theta)) * S*S};
}

ugl::Vector3 Rotation::log(const Rotation& R)
{
    constexpr double kTolerance = 1e-10;
    const double trace = R.matrix_.trace();
    if (trace + 1.0 < kTolerance) {
        const double R00 = R.matrix_(0,0);
        const double R11 = R.matrix_(1,1);
        const double R22 = R.matrix_(2,2);
        if (std::abs(R22 + 1.0) > kTolerance) {
            return M_PI / std::sqrt(2.0 * (R22 + 1.0)) * (R.matrix_.col(2) + ugl::Vector3::UnitZ());
        }
        else if (std::abs(R11 + 1.0) > kTolerance) {
            return M_PI / std::sqrt(2.0 * (R11 + 1.0)) * (R.matrix_.col(1) + ugl::Vector3::UnitY());
        }
        else /* if(std::abs(R00 + 1.0) > kTolerance) */ {
            return M_PI / std::sqrt(2.0 * (R00 + 1.0)) * (R.matrix_.col(0) + ugl::Vector3::UnitX());
        }
    }
    // Trace(R) close to 3.0 implies theta close to 0.0.
    if (3.0 - trace < kTolerance) {
        return vee(R.matrix_ - ugl::Matrix3::Identity());
    }
    const double theta = std::acos(0.5 * trace - 0.5);
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

bool operator==(const Rotation& lhs, const Rotation& rhs)
{
    return lhs.matrix_ == rhs.matrix_;
}

bool operator!=(const Rotation& lhs, const Rotation& rhs)
{
    return lhs.matrix_ != rhs.matrix_;
}

} // namespace ugl::lie
