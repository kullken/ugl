#include "ugl/trajectory/helix.h"

#include <cmath>
#include <memory>

#include "ugl/math/vector.h"
#include "ugl/lie_group/pose.h"

#include "ugl/trajectory/trajectory.h"

namespace ugl::trajectory
{

Helix::Helix(double radians, double radius, double duration, double z_velocity)
    : m_circle_arc(radians, radius, duration)
    , m_z_velocity(z_velocity)
{
}

Helix::Helix(double radians, double radius, double duration, double z_velocity, const ugl::lie::Pose& transform)
    : m_circle_arc(radians, radius, duration)
    , m_z_velocity(z_velocity)
    , m_transform(transform)
{
}

std::unique_ptr<LinearTrajectory> Helix::clone() const
{
    return std::make_unique<Helix>(*this);
}

ugl::Vector3 Helix::start() const
{
    return m_transform.transform(m_circle_arc.start());
}

ugl::Vector3 Helix::end() const
{
    return m_transform.transform(m_circle_arc.end() + ugl::Vector3::UnitZ() * m_z_velocity * m_circle_arc.duration());
}

ugl::Vector3 Helix::pos(double t) const
{
    const ugl::Vector3 pos = m_circle_arc.pos(t) + ugl::Vector3::UnitZ() * m_z_velocity * t;
    return m_transform.transform(pos);
}

ugl::Vector3 Helix::vel(double t) const
{
    const ugl::Vector3 vel = m_circle_arc.vel(t) + ugl::Vector3::UnitZ() * m_z_velocity;
    return m_transform.rotate(vel);
}

ugl::Vector3 Helix::acc(double t) const
{
    const ugl::Vector3 acc = m_circle_arc.acc(t);
    return m_transform.rotate(acc);
}

} // namespace ugl::trajectory
