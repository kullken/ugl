#pragma once

#include <vector>

#include "ugl/math/vector.h"
#include "ugl/math/matrix.h"

#include "ugl/trajectory/slerp_segment.h"
#include "ugl/trajectory/trajectory.h"

namespace ugl::trajectory
{

class SlerpSequence : public AngularTrajectory
{
    class Segment : public trajectory::SlerpSegment
    {
    public:
        double time_offset;
    };

private:
    double duration_ = 0;
    std::vector<Segment> segments_;

public:
    SlerpSequence() = default;
    SlerpSequence(std::vector<Segment> segments);

    double duration() const override { return duration_; }

    math::Rotation rotation(double t) const override;
    math::Vector3 ang_vel(double t) const override;

private:
    const Segment& get_segment_at(double t) const;
};

}