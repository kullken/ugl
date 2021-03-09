#ifndef UGL_LIE_POSE_H
#define UGL_LIE_POSE_H

#include "ugl/math/vector.h"
#include "ugl/math/matrix.h"
#include "ugl/math/quaternion.h"

#include "ugl/lie_group/common.h"
#include "ugl/lie_group/rotation.h"

namespace ugl::lie
{

// Lie-algebra of SE(3).
// TODO: Create unique class?
using se_3 = ugl::Matrix<4,4>;

// The group of 3D poses {R,p}. Also known as SE(3). The group is a matrix Lie group.
class Pose
{
public:
    /// @brief Degrees of freedom of the Lie group.
    static constexpr int DoF = 6;

    using VectorType = ugl::Vector<DoF>;
    using TangentType = se_3;

public:
    Pose() = default;

    explicit Pose(const Matrix<4,4>& matrix)
        : R_(matrix.block<3,3>(0,0))
        , pos_(matrix.block<3,1>(0,3))
    {
    }

    Pose(const Rotation& rotation, const Vector3& position)
        : R_(rotation)
        , pos_(position)
    {
    }

    Pose(const UnitQuaternion& quaternion, const Vector3& position)
        : R_(quaternion)
        , pos_(position)
    {
    }

    Matrix<4,4> matrix() const
    {
        Matrix<4,4> matrix = Matrix<4,4>::Identity();
        matrix.block<3,3>(0,0) = R_.matrix();
        matrix.block<3,1>(0,3) = pos_;
        return matrix;
    }

    void set_rotation(const Rotation& R) { R_ = R; }

    void set_position(const Vector3& pos) { pos_ = pos; }

    const Rotation& rotation() const { return R_; }

    const Vector3& position() const { return pos_; }

    // Group identity element.
    static Pose Identity()
    {
        return Pose{};
    }

    // Returns the inverse. The inverse is equal to the transpose for valid rotation matrices.
    Pose inverse() const
    {
        const Rotation Rinv = R_.inverse();
        return Pose{Rinv, -(Rinv * pos_)};
    }

    // Group composition
    Pose& operator*=(const Pose& rhs)
    {
        pos_ += R_ * rhs.pos_;
        R_ *= rhs.R_; // Must be last since R_ is used above.
        return *this;
    }

    /// @brief Transforms a vector from the local frame to the global frame.
    /// @param vec the vector in the local frame
    /// @return The vector in the global frame.
    Vector3 transform(const Vector3& vec) const;

    // Maps R^6 -> SE(3).
    static Pose exp(const Vector<6>& u);

    // Maps SE(3) -> R^6.
    static Vector<6> log(const Pose& T);

    // Maps R^ -> se(3). Also known as 'wedge' or '^'-operator.
    static se_3 hat(const Vector<6>& u);

    // Maps se2(3) -> R^9. Is the local inverse of the hat operator.
    static Vector<6> vee(const se_3& U);

    // Returns the adjoint matrix of a pose.
    static Matrix<6,6> adjoint(const Pose& T);

    /// @brief Calculates the left-Jacobian of SE(3) evaluated at a point tau.
    /// @param tau the evaluation point
    /// @return The left-Jacobian matrix
    static Matrix<6,6> left_jacobian(const Vector<6>& tau);

    /// @brief Calculates the inverse left-Jacobian of SE(3) evaluated at a point tau.
    /// @param tau the evaluation point
    /// @return The inverse left-Jacobian matrix
    static Matrix<6,6> left_jacobian_inv(const Vector<6>& tau);

    /// @brief Calculates the right-Jacobian of SE(3) evaluated at a point tau.
    /// @param tau the evaluation point
    /// @return The right-Jacobian matrix
    static Matrix<6,6> right_jacobian(const Vector<6>& tau);

    /// @brief Calculates the inverse right-Jacobian of SE(3) evaluated at a point tau.
    /// @param tau the evaluation point
    /// @return The inverse right-Jacobian matrix
    static Matrix<6,6> right_jacobian_inv(const Vector<6>& tau);

    friend bool operator==(const Pose& lhs, const Pose& rhs);
    friend bool operator!=(const Pose& lhs, const Pose& rhs);

private:
    // Rotation
    Rotation R_ = Rotation::Identity();
    // Position
    Vector3 pos_ = Vector3::Zero();
};

inline
Pose operator*(Pose lhs, const Pose& rhs)
{
    return lhs *= rhs;
}

// An alias to make math nerds happy.
using SE_3 = Pose;

} // namespace ugl::lie

#endif // UGL_LIE_POSE_H
