#include "ugl/lie_group/rotation2d.h"

#include <cmath>

#include "ugl/math/vector.h"
#include "ugl/math/matrix.h"

namespace ugl::lie
{

Rotation2D::Rotation2D(double angle)
{
    const auto sin_angle = std::sin(angle);
    const auto cos_angle = std::cos(angle);
    matrix_(0,0) =  cos_angle;
    matrix_(0,1) = -sin_angle;
    matrix_(1,0) =  sin_angle;
    matrix_(1,1) =  cos_angle;
}

ugl::UnitQuaternion Rotation2D::to_quaternion() const
{
    ugl::Matrix3 rot3d = ugl::Matrix3::Identity();
    rot3d.block<2,2>(0,0) = matrix_;
    return ugl::UnitQuaternion{rot3d};
}

} // namespace ugl::lie
