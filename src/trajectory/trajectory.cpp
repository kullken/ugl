#include "ugl/trajectory/trajectory.h"

#include "ugl/math/vector.h"
#include "ugl/math/quaternion.h"

#include "ugl/trajectory/bezier.h"
#include "ugl/trajectory/slerp_segment.h"

namespace ugl::trajectory
{

Trajectory::Trajectory(const Trajectory& other)
    : linear_trajectory_(other.linear_trajectory_ ? other.linear_trajectory_->clone() : nullptr)
    , angular_trajectory_(other.angular_trajectory_ ? other.angular_trajectory_->clone() : nullptr)
{
}

Trajectory& Trajectory::operator=(const Trajectory& other)
{
    linear_trajectory_ = other.linear_trajectory_ ? other.linear_trajectory_->clone() : nullptr;
    angular_trajectory_ = other.angular_trajectory_ ? other.angular_trajectory_->clone() : nullptr;
    return *this;
}

}