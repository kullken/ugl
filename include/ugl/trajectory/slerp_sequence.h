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
    class Segment : public SlerpSegment
    {
    public:
        double time_offset;
        Segment(const SlerpSegment& segment, double offset) 
            : SlerpSegment(segment)
            , time_offset(offset) {}
    };

public:
    SlerpSequence() = default;
    SlerpSequence(const SlerpSequence&) = default;

    explicit SlerpSequence(const std::vector<SlerpSegment>& segments);

    std::unique_ptr<AngularTrajectory> clone() const override
    {
        return std::make_unique<SlerpSequence>(*this);
    }

    double duration() const override { return duration_; }

    math::Rotation rotation(double t) const override;
    math::UnitQuaternion quat(double t) const override;
    math::Vector3 ang_vel(double t) const override;

private:
    const Segment& get_segment_at(double t) const;

private:
    double duration_ = 0;
    std::vector<Segment> segments_;
};

}