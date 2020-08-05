#pragma once

#include <memory>

#include "ugl/math/vector.h"
#include "ugl/math/matrix.h"
#include "ugl/math/quaternion.h"

namespace ugl::trajectory
{

class LinearTrajectory
{
public:
    virtual ~LinearTrajectory() = default;

    virtual std::unique_ptr<LinearTrajectory> clone() const = 0;

    virtual double duration() const = 0;

    // Returns the start position of the trajectory. Functionally equivalent 
    // to calling pos(0), but might be faster depending on implementation.
    virtual math::Vector3 start() const = 0;

    // Returns the end position of the trajectory. Functionally equivalent 
    // to calling pos(duration()), but might be faster depending on implementation.
    virtual math::Vector3 end() const = 0;

    virtual math::Vector3 pos(double t) const = 0;
    virtual math::Vector3 vel(double t) const = 0;
    virtual math::Vector3 acc(double t) const = 0;
};

class AngularTrajectory
{
public:
    virtual ~AngularTrajectory() = default;

    virtual std::unique_ptr<AngularTrajectory> clone() const = 0;

    virtual double duration() const = 0;

    virtual math::Rotation rotation(double t) const = 0;
    virtual math::UnitQuaternion quat(double t) const = 0;
    virtual math::Vector3 ang_vel(double t) const = 0;
};

class Trajectory
{
public:
    Trajectory() = default;
    Trajectory(const Trajectory& other);
    Trajectory(Trajectory&&) = default;
    Trajectory& operator=(const Trajectory& other);
    Trajectory& operator=(Trajectory&&) = default;
    ~Trajectory() = default;

    // TODO: Assert that XxxTrajectoryDerived is derived from XxxTrajectory.
    template<typename LinearTrajectoryDerived, typename AngularTrajectoryDerived>
    Trajectory(LinearTrajectoryDerived lin_traj, AngularTrajectoryDerived ang_traj)
        : linear_trajectory_(std::make_unique<LinearTrajectoryDerived>(lin_traj))
        , angular_trajectory_(std::make_unique<AngularTrajectoryDerived>(ang_traj))
    {
    }

    double duration() const { return std::min(linear_trajectory_->duration(), angular_trajectory_->duration()); }

    math::Vector3 get_position(double t) const { return linear_trajectory_->pos(t); }
    math::Vector3 get_velocity(double t) const { return linear_trajectory_->vel(t); }
    math::Vector3 get_acceleration(double t) const { return linear_trajectory_->acc(t); }

    math::Rotation get_rotation(double t) const { return angular_trajectory_->rotation(t); }
    math::UnitQuaternion get_quaternion(double t) const { return angular_trajectory_->quat(t); }
    math::Vector3 get_angular_velocity(double t) const { return angular_trajectory_->ang_vel(t); }

private:
    std::unique_ptr<const LinearTrajectory> linear_trajectory_ = nullptr;
    std::unique_ptr<const AngularTrajectory> angular_trajectory_ = nullptr;
};

}