#pragma once

#include <vector>

#include "ugl/math/vector.h"
#include "ugl/math/matrix.h"

#include "ugl/trajectory/slerp_segment.h"

namespace ugl::trajectory
{

class AngularTrajectory
{
    class Segment : public trajectory::SlerpSegment
    {
    public:
        double time_offset;
    };

private:
    double m_duration = 0;
    std::vector<Segment> m_segments;

public:
    AngularTrajectory() = default;
    AngularTrajectory(std::vector<Segment> segments);

    math::Rotation get_rotation(double t) const;
    math::Vector3 get_angular_velocity(double t) const;

private:
    const Segment& get_segment_at(double t) const;
};

}