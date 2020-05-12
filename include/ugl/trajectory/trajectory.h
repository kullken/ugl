#pragma once

#include "ugl/math/vector.h"
#include "ugl/math/matrix.h"

#include "ugl/trajectory/linear_trajectory.h"
#include "ugl/trajectory/angular_trajectory.h"

namespace ugl::trajectory
{

class Trajectory
{
private:
    const LinearTrajectory m_linear_trajectory;
    const AngularTrajectory m_angular_trajectory;

public:
    Trajectory() = default;
    Trajectory(LinearTrajectory lin_traj, AngularTrajectory ang_traj)
        : m_linear_trajectory(lin_traj)
        , m_angular_trajectory(ang_traj)
    {
    }

    double duration() const { return std::min(m_linear_trajectory.duration(), m_angular_trajectory.duration()); }

    math::Vector3 get_position(double t) const { return m_linear_trajectory.get_position(t); }
    math::Vector3 get_velocity(double t) const { return m_linear_trajectory.get_velocity(t); }
    math::Vector3 get_acceleration(double t) const { return m_linear_trajectory.get_acceleration(t); }

    math::Rotation get_rotation(double t) const { return m_angular_trajectory.get_rotation(t); }
    math::Vector3 get_angular_velocity(double t) const { return m_angular_trajectory.get_angular_velocity(t); }
};

}