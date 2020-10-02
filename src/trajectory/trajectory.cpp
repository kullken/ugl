#include "ugl/trajectory/trajectory.h"

#include "ugl/lie_group/pose.h"
#include "ugl/lie_group/extended_pose.h"

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

lie::Pose Trajectory::get_pose(double t) const
{
    return lie::Pose{get_rotation(t), get_position(t)};
}

lie::ExtendedPose Trajectory::get_extended_pose(double t) const
{
    return lie::ExtendedPose{get_rotation(t), get_velocity(t), get_position(t)};
}

} // namespace ugl::trajectory
