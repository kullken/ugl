#pragma once

#include <ostream>

#include "ugl/math/vector.h"
#include "ugl/math/matrix.h"
#include "ugl/math/quaternion.h"

namespace ugl::lie
{

// Lie-algebra of SO(2). Consists of skew-symmetric 3-by-3 matrices.
// TODO: Create unique class?
using so2 = ugl::Matrix<2,2>;

// The 2D rotation group, often denoted SO(2). The group is a matrix Lie group.
class Rotation2D
{
public:
    Rotation2D() = default;

    explicit Rotation2D(double angle);

    explicit Rotation2D(const ugl::Matrix<2,2>& matrix)
        : matrix_(matrix)
    {
        // TODO: Verify that matrix is a valid rotation matrix.
    }

    const auto& matrix() const
    {
        return matrix_;
    }

    // Returns the inverse. The inverse is equal to the transpose for valid rotation matrices.
    Rotation2D inverse() const
    {
        return Rotation2D{matrix_.transpose()};
    }

    // Group composition
    Rotation2D& operator*=(const Rotation2D& rhs)
    {
        matrix_ *= rhs.matrix_;
        return *this;
    }

    // Convert to unit quaternion.
    ugl::UnitQuaternion to_quaternion() const;

    // Group identity element.
    static Rotation2D Identity()
    {
        return Rotation2D{};
    }

    // // Maps R^? -> SO(2).
    // static auto exp(const ugl::Vector<?>& w) -> Rotation2D;

    // // Maps SO(2) -> R^?.
    // static auto log(const Rotation2D& R) -> ugl::Vector<?>;

    // // Maps R^? -> so2(). Also known as 'wedge' or '^'-operator. Returns the skew-symmetric cross-product matrix.
    // static auto hat(const ugl::Vector<?>& w) -> so2;

    // // Maps so(2) -> R^?. Is the local inverse of the hat operator.
    // static auto vee(const so2& S) -> ugl::Vector<?>;

private:
    ugl::Matrix<2,2> matrix_ = ugl::Matrix<2,2>::Identity();
};

// Group composition
inline
Rotation2D operator*(Rotation2D lhs, const Rotation2D& rhs)
{
    return lhs *= rhs;
}

inline
ugl::Vector<2> operator*(const Rotation2D& lhs, const ugl::Vector<2>& rhs)
{
    return lhs.matrix() * rhs;
}

// The 2D rotation group, SO(2). The group is a matrix Lie group.
using SO2 = Rotation2D;

inline
std::ostream& operator<<(std::ostream& os, const Rotation2D& R)
{
    return os << R.matrix();
}

} // namespace ugl::lie
