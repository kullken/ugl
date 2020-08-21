#ifndef UGL_TRAJECTORY_BEZIER_SEQUENCE_H
#define UGL_TRAJECTORY_BEZIER_SEQUENCE_H

#include <cassert>
#include <memory>
#include <stdexcept>
#include <vector>

#include "ugl/math/vector.h"

#include "ugl/trajectory/bezier.h"
#include "ugl/trajectory/trajectory.h"

namespace ugl::trajectory
{

template<int degree>
class BezierSequence : public LinearTrajectory
{
    class Segment : public Bezier<degree>
    {
    public:
        double time_offset;
        Segment(const Bezier<degree>& bezier, double offset)
            : Bezier<degree>(bezier)
            , time_offset(offset) {}
    };

public:
    BezierSequence() = default;

    explicit BezierSequence(const std::vector<Bezier<degree>>& beziers)
    {
        for (const auto& bezier : beziers)
        {
            segments_.emplace_back(bezier, duration_);
            duration_ += bezier.duration();
        }
        // TODO: Assert that all segments are connected continously in position, velocity(?), acceleration(??) and jerk(???).
    }

    std::unique_ptr<LinearTrajectory> clone() const override
    {
        return std::make_unique<BezierSequence>(*this);
    }

    double duration() const override { return duration_; }
    const auto& segments() const { return segments_; }

    ugl::Vector3 start() const override { return segments_.front().start(); }
    ugl::Vector3 end() const override { return segments_.back().end(); }

    math::Vector3 pos(double t) const override
    {
        const Segment& segment = get_segment_at(t);
        return segment.pos(t - segment.time_offset);
    }

    math::Vector3 vel(double t) const override
    {
        const Segment& segment = get_segment_at(t);
        return segment.vel(t - segment.time_offset);
    }

    math::Vector3 acc(double t) const override
    {
        const Segment& segment = get_segment_at(t);
        return segment.acc(t - segment.time_offset);
    }

private:
    const Segment& get_segment_at(double t) const
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

private:
    double duration_ = 0;
    std::vector<Segment> segments_;
};

} // namespace ugl::trajectory

#endif // UGL_TRAJECTORY_BEZIER_SEQUENCE_H
