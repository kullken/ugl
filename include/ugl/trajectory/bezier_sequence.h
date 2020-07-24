#pragma once

#include <vector>
#include <memory>

#include "ugl/math/vector.h"

#include "ugl/trajectory/bezier.h"
#include "ugl/trajectory/trajectory.h"

namespace ugl::trajectory
{

// TODO?: Template with degree parameter?
class BezierSequence : public LinearTrajectory
{
    class Segment : public Bezier<3>
    {
    public:
        double time_offset;
    };

private:
    double duration_ = 0;
    std::vector<Segment> segments_;

public:
    BezierSequence() = default;

    explicit BezierSequence(const std::vector<Segment>& segments);

    std::unique_ptr<LinearTrajectory> clone() const override
    {
        return std::make_unique<BezierSequence>(*this);
    }

    double duration() const override { return duration_; }

    math::Vector3 pos(double t) const override;
    math::Vector3 vel(double t) const override;
    math::Vector3 acc(double t) const override;

private:
    const Segment& get_segment_at(double t) const;
};

}