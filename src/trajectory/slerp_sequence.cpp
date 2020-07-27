#include "ugl/trajectory/slerp_sequence.h"

#include <cassert>
#include <stdexcept>
#include <vector>

#include "ugl/math/vector.h"
#include "ugl/math/matrix.h"

#include "ugl/trajectory/slerp_segment.h"

namespace ugl::trajectory
{

SlerpSequence::SlerpSequence(const std::vector<SlerpSegment>& segments)
{
    for (const auto& arc : segments)
    {
        segments_.emplace_back(arc, duration_);
        duration_ += arc.duration();
    }
    // TODO?: Assert that all segments are connected continously up to the x(?):th derivative.
}

math::Rotation SlerpSequence::rotation(double t) const
{
    const Segment& segment = get_segment_at(t);
    return segment.rotation(t - segment.time_offset);
}

math::UnitQuaternion SlerpSequence::quat(double t) const
{
    const Segment& segment = get_segment_at(t);
    return segment.quat(t - segment.time_offset);
}

math::Vector3 SlerpSequence::ang_vel(double t) const
{
    const Segment& segment = get_segment_at(t);
    return segment.ang_vel(t - segment.time_offset);
}

const SlerpSequence::Segment& SlerpSequence::get_segment_at(double t) const
{
    assert(0 <= t && t <= duration_);

    for (auto it = std::rbegin(segments_); it != std::rend(segments_); ++it)
    {
        if (it->time_offset <= t)
        {
            return *it;
        }
    }

    throw std::logic_error("Could not access trajectory segment.");
}

}