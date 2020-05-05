#pragma once

#include "geometry_cpp/math/vector.h"
#include "geometry_cpp/math/matrix.h"

#include "geometry_cpp/trajectory/linear_trajectory.h"
#include "geometry_cpp/trajectory/angular_trajectory.h"

namespace geometry::trajectory
{

class Trajectory
{
private:
    const LinearTrajectory m_linear_trajectory;
    const AngularTrajectory m_angular_trajectory;

public:
    Trajectory() = default;

    math::Vector3 get_position(double t) const;
    math::Vector3 get_velocity(double t) const;
    math::Vector3 get_acceleration(double t) const;

    math::Rotation get_rotation(double t) const;
    math::Vector3 get_angular_velocity(double t) const;
};

}