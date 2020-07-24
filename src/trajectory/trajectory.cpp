#include "ugl/trajectory/trajectory.h"

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