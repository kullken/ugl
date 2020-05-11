#pragma once

#include <vector>

#include "ugl/math/vector.h"

#include "ugl/trajectory/bezier.h"

namespace ugl::trajectory
{

class LinearTrajectory
{
    class Segment : public Bezier<math::Vector3, 3>
    {
    public:
        double time_offset;
    };

private:
    double m_duration = 0;
    std::vector<Segment> m_segments;

public:
    LinearTrajectory() = default;
    LinearTrajectory(std::vector<Segment> segments);

    double duration() const { return m_duration; }

    math::Vector3 get_position(double t) const;
    math::Vector3 get_velocity(double t) const;
    math::Vector3 get_acceleration(double t) const;

private:
    const Segment& get_segment_at(double t) const;
};

}