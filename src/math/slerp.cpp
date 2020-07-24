#include "ugl/math/slerp.h"

#include <cassert>

#include <unsupported/Eigen/MatrixFunctions>

#include "ugl/math/quaternion.h"

namespace ugl::math
{

UnitQuaternion slerp(const UnitQuaternion& q0, const UnitQuaternion& q1, double t)
{
    assert(0.0 <= t && t <= 1.0);
    return q0.slerp(t, q1);
}

} // namespace ugl::math
