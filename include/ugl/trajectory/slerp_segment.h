#pragma once

#include <cassert>
#include <cmath>
#include <utility>
#include <algorithm>
#include <memory>

#include "ugl/math/vector.h"
#include "ugl/math/quaternion.h"
#include "ugl/math/slerp.h"

#include "ugl/trajectory/trajectory.h"

#include "ugl/lie_group/utility.h"

namespace ugl::trajectory
{

class SlerpSegment : public AngularTrajectory
{
private:
    const double duration_ = 1;
    const math::UnitQuaternion q0_ = lie::identity<math::UnitQuaternion>();
    const math::UnitQuaternion q1_;

public:
    SlerpSegment(const SlerpSegment&) = default;

    SlerpSegment(double duration, math::UnitQuaternion start, math::UnitQuaternion end)
        : duration_(duration)
        , q0_(start)
        , q1_(end)
    {
        assert(("Duration must be positive.", duration > 0));
    }

    SlerpSegment(double duration, math::UnitQuaternion end)
        : duration_(duration)
        , q1_(end)
    {
        assert(("Duration must be positive.", duration > 0));
    }

    SlerpSegment(math::UnitQuaternion start, math::UnitQuaternion end)
        : q0_(start)
        , q1_(end)
    {
    }

    explicit SlerpSegment(math::UnitQuaternion end)
        : q1_(end)
    {
    }

    std::unique_ptr<AngularTrajectory> clone() const override
    {
        return std::make_unique<SlerpSegment>(*this);
    }

    double duration() const { return duration_; }

    math::Rotation rotation(double t) const override
    {
        return math::Rotation(math::slerp(q0_, q1_, t/duration_));
    }


    math::Vector3 ang_vel([[maybe_unused]] double t) const override
    {
        return 2 / duration_ * math::log(q1_*q0_.inverse()).vec();
    }
};

}
