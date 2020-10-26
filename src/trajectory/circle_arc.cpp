#include "ugl/trajectory/circle_arc.h"

#include <cmath>
#include <memory>

#include "ugl/math/vector.h"
#include "ugl/lie_group/pose.h"

#include "ugl/trajectory/trajectory.h"

namespace ugl::trajectory
{

CircleArc::CircleArc(double radians)
    : m_angle(radians)
{
}

CircleArc::CircleArc(double radians, double radius, double duration)
    : m_angle(radians)
    , m_radius(radius)
    , m_duration(duration)
{
}

CircleArc::CircleArc(double radians, double radius, double duration, const ugl::lie::Pose& transform)
    : m_angle(radians)
    , m_radius(radius)
    , m_duration(duration)
    , m_transform(transform)
{
}

std::unique_ptr<LinearTrajectory> CircleArc::clone() const
{
    return std::make_unique<CircleArc>(*this);
}

ugl::Vector3 CircleArc::start() const
{
    return m_transform.transform(ugl::Vector3::UnitX() * m_radius);
}

ugl::Vector3 CircleArc::end() const
{
    return m_transform.transform(ugl::Vector3{std::cos(m_angle), std::sin(m_angle), 0.0} * m_radius);
}

ugl::Vector3 CircleArc::pos(double t) const
{
    const double angle = theta(t);
    return m_transform.transform(ugl::Vector3{std::cos(angle), std::sin(angle), 0.0} * m_radius);
}

ugl::Vector3 CircleArc::vel(double t) const
{
    const double ang_vel = m_angle / m_duration;
    const double angle = theta(t);
    return m_transform.rotation() * ugl::Vector3{-std::sin(angle), std::cos(angle), 0.0} * ang_vel * m_radius;
}

ugl::Vector3 CircleArc::acc(double t) const
{
    const double ang_vel = m_angle / m_duration;
    const double angle = theta(t);
    return m_transform.rotation() * ugl::Vector3{-std::cos(angle), -std::sin(angle), 0.0} * ang_vel * ang_vel * m_radius;
}

double CircleArc::theta(double t) const
{
    return m_angle * t / m_duration;
}

} // namespace ugl::trajectory
