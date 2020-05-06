#include "trajectory/linear_trajectory.h"

#include <vector>
#include <cassert>
#include <stdexcept>

#include "math/vector.h"

namespace ugl::trajectory
{

LinearTrajectory::LinearTrajectory(std::vector<Segment> segments)
    : m_segments(segments)
{
    for (auto& segment : m_segments)
    {
        segment.time_offset = m_duration;
        m_duration += segment.duration();
    }
    // TODO: Assert that all segments are connected continously up to the x(?):th derivative.
}

math::Vector3 LinearTrajectory::get_position(double t) const
{
    const Segment& segment = get_segment_at(t);
    return segment.pos(t - segment.time_offset);
}

math::Vector3 LinearTrajectory::get_velocity(double t) const
{
    const Segment& segment = get_segment_at(t);
    return segment.vel(t - segment.time_offset);
}

math::Vector3 LinearTrajectory::get_acceleration(double t) const
{
    const Segment& segment = get_segment_at(t);
    return segment.acc(t - segment.time_offset);
}

const LinearTrajectory::Segment& LinearTrajectory::get_segment_at(double t) const
{
    assert(("", 0 <= t && t <= m_duration));

    for (auto it = std::rbegin(m_segments); it != std::rend(m_segments); ++it)
    {
        if (it->time_offset < t)
        {
            return *it;
        }
    }

    throw std::logic_error("Could not access trajectory segment."); 
}

}