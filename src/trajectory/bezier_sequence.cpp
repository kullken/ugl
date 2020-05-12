#include "ugl/trajectory/bezier_sequence.h"

#include <vector>
#include <cassert>
#include <stdexcept>

#include "math/vector.h"

namespace ugl::trajectory
{

BezierSequence::BezierSequence(std::vector<Segment> segments)
    : segments_(segments)
{
    for (auto& segment : segments_)
    {
        segment.time_offset = duration_;
        duration_ += segment.duration();
    }
    // TODO?: Assert that all segments are connected continously up to the x(?):th derivative.
}

math::Vector3 BezierSequence::pos(double t) const
{
    const Segment& segment = get_segment_at(t);
    return segment.pos(t - segment.time_offset);
}

math::Vector3 BezierSequence::vel(double t) const
{
    const Segment& segment = get_segment_at(t);
    return segment.vel(t - segment.time_offset);
}

math::Vector3 BezierSequence::acc(double t) const
{
    const Segment& segment = get_segment_at(t);
    return segment.acc(t - segment.time_offset);
}

const BezierSequence::Segment& BezierSequence::get_segment_at(double t) const
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