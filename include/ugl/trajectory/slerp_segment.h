#pragma once

#include <cmath>
#include <utility>
#include <algorithm>

#include "ugl/math/vector.h"
#include "ugl/math/quaternion.h"
#include "ugl/math/slerp.h"

#include "ugl/lie_group/utility.h"

namespace ugl::trajectory
{

class SlerpSegment
{
private:
    const double duration_ = 1;
    const math::UnitQuaternion q0_ = lie::identity<math::UnitQuaternion>();
    const math::UnitQuaternion q1_;

public:
    SlerpSegment(double duration, math::UnitQuaternion start, math::UnitQuaternion end)
        : duration_(duration)
        , q0_(start)
        , q1_(end)
    {
        // TODO: Assert: duration_ > 0
    }

    SlerpSegment(double duration, math::UnitQuaternion end)
        : duration_(duration)
        , q1_(end)
    {
        // TODO: Assert: duration_ > 0
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

    double duration() const { return duration_; }

    math::UnitQuaternion rotation(double t) const
    {
        return math::slerp(q0_, q1_, t/duration_);
    }


    math::Vector3 angular_velocity([[maybe_unused]] double t) const
    {
        // TODO?: Input parameter t exists to future proof the API. Remove?
        return 2 / duration_ * math::log(q1_*q0_.inverse()).vec();
    }
};

}
