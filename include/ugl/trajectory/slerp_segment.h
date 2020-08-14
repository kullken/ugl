#pragma once

#include <cassert>
#include <memory>

#include "ugl/math/vector.h"
#include "ugl/math/quaternion.h"
#include "ugl/math/slerp.h"

#include "ugl/lie_group/rotation.h"

#include "ugl/trajectory/trajectory.h"

namespace ugl::trajectory
{

class SlerpSegment : public AngularTrajectory
{
public:
    SlerpSegment(double duration, const math::UnitQuaternion& start, const math::UnitQuaternion& end)
        : duration_(duration)
        , q0_(start)
        , q1_(end)
    {
        assert(duration > 0); // Duration must be positive.
    }

    SlerpSegment(double duration, const math::UnitQuaternion& end)
        : duration_(duration)
        , q1_(end)
    {
        assert(duration > 0); // Duration must be positive.
    }

    SlerpSegment(const math::UnitQuaternion& start, const math::UnitQuaternion& end)
        : q0_(start)
        , q1_(end)
    {
    }

    explicit SlerpSegment(const math::UnitQuaternion& end)
        : q1_(end)
    {
    }

    std::unique_ptr<AngularTrajectory> clone() const override
    {
        return std::make_unique<SlerpSegment>(*this);
    }

    double duration() const override { return duration_; }

    const ugl::UnitQuaternion& start() const { return q0_; }
    const ugl::UnitQuaternion& end() const { return q1_; }

    lie::Rotation rotation(double t) const override
    {
        return lie::Rotation{quat(t)};
    }

    math::UnitQuaternion quat(double t) const override
    {
        return math::slerp(q0_, q1_, t/duration_);
    }

    math::Vector3 ang_vel(double) const override
    {
        return 2 / duration_ * math::log(q1_*q0_.inverse()).vec();
    }

private:
    double duration_ = 1;
    math::UnitQuaternion q0_ = math::UnitQuaternion::Identity();
    math::UnitQuaternion q1_;
};

}
