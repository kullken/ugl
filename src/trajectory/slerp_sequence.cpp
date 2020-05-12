#include "trajectory/slerp_sequence.h"

#include <vector>
#include <cassert>
#include <stdexcept>

#include "math/vector.h"
#include "math/matrix.h"

namespace ugl::trajectory
{

SlerpSequence::SlerpSequence(std::vector<Segment> segments)
    : segments_(segments)
{
    for (auto& segment : segments_)
    {
        segment.time_offset = duration_;
        duration_ += segment.duration();
    }
    // TODO?: Assert that all segments are connected continously up to the x(?):th derivative.
}

math::Rotation SlerpSequence::rotation(double t) const
{
    const Segment& segment = get_segment_at(t);
    return segment.rotation(t - segment.time_offset);
}

math::Vector3 SlerpSequence::ang_vel(double t) const
{
    const Segment& segment = get_segment_at(t);
    return segment.ang_vel(t - segment.time_offset);
}

const SlerpSequence::Segment& SlerpSequence::get_segment_at(double t) const
{
    assert(("", 0 <= t && t <= duration_));

    for (auto it = std::rbegin(segments_); it != std::rend(segments_); ++it)
    {
        if (it->time_offset < t)
        {
            return *it;
        }
    }

    throw std::logic_error("Could not access trajectory segment.");
}

}