#pragma once

#include "ugl/math/vector.h"
#include "ugl/math/matrix.h"

#include "ugl/lie_group/rotation.h"

namespace ugl::lie
{

// Lie-algebra of SE2(3).
// TODO: Create unique class?
using se2_3 = ugl::Matrix<5,5>;

// The group of extended poses {R,v,p}. Also known as SE2(3). The group is a matrix Lie group.
class ExtendedPose
{
public:
    ExtendedPose() = default;

    explicit ExtendedPose(const Matrix<5,5>& matrix)
        : R_(matrix.block<3,3>(0,0))
        , vel_(matrix.block<3,1>(0,3))
        , pos_(matrix.block<3,1>(0,4))
    {
    }

    ExtendedPose(const Rotation& rotation, const Vector3& velocity, const Vector3& position)
        : R_(rotation)
        , vel_(velocity)
        , pos_(position)
    {
    }

    Matrix<5,5> matrix() const
    {
        Matrix<5,5> matrix = Matrix<5,5>::Identity();
        matrix.block<3,3>(0,0) = R_.matrix();
        matrix.block<3,1>(0,3) = vel_;
        matrix.block<3,1>(0,4) = pos_;
        return matrix;
    }

    void set_rotation(const Rotation& R) { R_ = R; }

    void set_velocity(const Vector3& vel) { vel_ = vel; }

    void set_position(const Vector3& pos) { pos_ = pos; }

    const Rotation& rotation() const { return R_; }

    const Vector3& velocity() const { return vel_; }

    const Vector3& position() const { return pos_; }

    // Group identity element.
    static ExtendedPose Identity() 
    {
        return ExtendedPose{};
    }

    // Returns the inverse. The inverse is equal to the transpose for valid rotation matrices.
    ExtendedPose inverse() const
    {
        const Rotation Rinv = R_.inverse();
        return ExtendedPose{Rinv, -(Rinv * vel_), -(Rinv * pos_)};
    }

    // Group composition
    ExtendedPose& operator*=(const ExtendedPose& rhs)
    {
        vel_ += R_ * rhs.vel_;
        pos_ += R_ * rhs.pos_;
        R_ *= rhs.R_; // Must be last since R_ is used above.
        return *this;
    }

    // Maps R^9 -> SE2(3).
    static ExtendedPose exp(const Vector<9>& u);

    // Maps SE2(3) -> R^9.
    static Vector<9> log(const ExtendedPose& T);

    // Maps R^9 -> se2(3). Also known as 'wedge' or '^'-operator.
    static se2_3 hat(const Vector<9>& u);

    // Maps se2(3) -> R^9. Is the local inverse of the hat operator.
    static Vector<9> vee(const se2_3& U);

    // Returns the adjoint matrix of an extended pose.
    static Matrix<9,9> adjoint(const ExtendedPose& T);

private:
    // Rotation
    Rotation R_ = Rotation::Identity();
    // Velocity
    Vector3 vel_ = Vector3::Zero();
    // Position
    Vector3 pos_ = Vector3::Zero();
};

inline
ExtendedPose operator*(ExtendedPose lhs, const ExtendedPose& rhs)
{
    return lhs *= rhs;
}

// An alias to make math nerds happy.
using SE2_3 = ExtendedPose;

} // namespace ugl::lie
