#ifndef UGL_LIE_ROTATION_H
#define UGL_LIE_ROTATION_H

#include <ostream>

#include "ugl/math/vector.h"
#include "ugl/math/matrix.h"
#include "ugl/math/quaternion.h"

#include "ugl/lie_group/common.h"

namespace ugl::lie
{

// Lie-algebra of SO(3). Consists of skew-symmetric 3-by-3 matrices.
// TODO: Create unique class?
using so3 = ugl::Matrix3;

// The 3D rotation group, often denoted SO(3). The group is a matrix Lie group.
class Rotation
{
public:
    /// @brief Degrees of freedom of the Lie group.
    static constexpr int DoF = 3;

    using VectorType = ugl::Vector<DoF>;
    using TangentType = so3;

public:
    Rotation() = default;

    explicit Rotation(const ugl::Matrix3& matrix)
        : matrix_(matrix)
    {
        // TODO: Verify that matrix is a valid rotation matrix.
    }

    explicit Rotation(const ugl::UnitQuaternion& quat)
        : matrix_(quat.toRotationMatrix())
    {
    }

    const auto& matrix() const
    {
        return matrix_;
    }

    // Returns the inverse. The inverse is equal to the transpose for valid rotation matrices.
    Rotation inverse() const
    {
        return Rotation{matrix_.transpose()};
    }

    // Group composition
    Rotation& operator*=(const Rotation& rhs)
    {
        matrix_ *= rhs.matrix_;
        return *this;
    }

    // Convert to unit quaternion.
    ugl::UnitQuaternion to_quaternion() const
    {
        return ugl::UnitQuaternion{matrix_};
    }

    // Group identity element.
    static Rotation Identity()
    {
        return Rotation{};
    }

    // Maps R^3 -> SO(3).
    static auto exp(const ugl::Vector3& w) -> Rotation;

    // Maps SO(3) -> R^3.
    static auto log(const Rotation& R) -> ugl::Vector3;

    // Maps R^3 -> so3(). Also known as 'wedge' or '^'-operator. Returns the skew-symmetric cross-product matrix.
    static auto hat(const ugl::Vector3& w) -> so3;

    // Maps so(3) -> R^3. Is the local inverse of the hat operator.
    static auto vee(const so3& S) -> ugl::Vector3;

    /// @brief Calculates the left-Jacobian of SO(3) evaluated at a point phi.
    /// @param phi the evaluation point
    /// @return The left-Jacobian matrix
    static ugl::Matrix3 left_jacobian(const ugl::Vector3& phi);

    /// @brief Calculates the inverse left-Jacobian of SO(3) evaluated at a point phi.
    /// @param phi the evaluation point
    /// @return The inverse left-Jacobian matrix
    static ugl::Matrix3 left_jacobian_inv(const ugl::Vector3& phi);

    /// @brief Calculates the right-Jacobian of SE(3) evaluated at a point phi.
    /// @param phi the evaluation point
    /// @return The right-Jacobian matrix
    static ugl::Matrix3 right_jacobian(const ugl::Vector3& phi);

    /// @brief Calculates the inverse right-Jacobian of SE(3) evaluated at a point phi.
    /// @param phi the evaluation point
    /// @return The inverse right-Jacobian matrix
    static ugl::Matrix3 right_jacobian_inv(const ugl::Vector3& phi);

private:
    ugl::Matrix3 matrix_ = ugl::Matrix3::Identity();
};

// Group composition
inline
Rotation operator*(Rotation lhs, const Rotation& rhs)
{
    return lhs *= rhs;
}

inline
ugl::Vector3 operator*(const Rotation& lhs, const ugl::Vector3& rhs)
{
    return lhs.matrix() * rhs;
}

// The 3D rotation group, often denoted SO(3). The group is a matrix Lie group.
using SO3 = Rotation;

inline
ugl::Matrix3 skew(const ugl::Vector3& w)
{
    return SO3::hat(w);
}

inline
ugl::Vector3 unskew(const ugl::Matrix3& w)
{
    return SO3::vee(w);
}

inline
std::ostream& operator<<(std::ostream& os, const Rotation& R)
{
    return os << R.matrix();
}

} // namespace ugl::lie

#endif // UGL_LIE_ROTATION_H
