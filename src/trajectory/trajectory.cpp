#include "ugl/trajectory/trajectory.h"

#include "ugl/math/vector.h"
#include "ugl/math/quaternion.h"

#include "ugl/trajectory/bezier.h"
#include "ugl/trajectory/slerp_segment.h"

namespace ugl::trajectory
{

Trajectory::Trajectory()
    : linear_trajectory_(std::make_unique<Bezier<0>>(Bezier<0>{0.0, {Vector3::Zero()}}))
    , angular_trajectory_(std::make_unique<SlerpSegment>(SlerpSegment{0.0, UnitQuaternion::Identity()}))
{
}

Trajectory::Trajectory(const Trajectory& other)
    : linear_trajectory_(other.linear_trajectory_->clone())
    , angular_trajectory_(other.angular_trajectory_->clone())
{
}

Trajectory& Trajectory::operator=(const Trajectory& other)
{
    linear_trajectory_ = other.linear_trajectory_->clone();
    angular_trajectory_ = other.angular_trajectory_->clone();
    return *this;
}

}