#include "ugl/lie_group/rotation.h"

#include <cmath>

#include "ugl/math/vector.h"
#include "ugl/math/matrix.h"

namespace ugl::lie
{

Rotation Rotation::exp(const ugl::Vector3& w)
{
    return Rotation{math::exp(hat(w))};
}

ugl::Vector3 Rotation::log(const Rotation& R)
{
    return vee(math::log(R.matrix()));
}

so3 Rotation::hat(const ugl::Vector3& w)
{
    ugl::Matrix3 S;
    S << 0.0, -w(2), w(1),
         w(2), 0.0, -w(0),
        -w(1), w(0), 0.0;
    return S;
}

ugl::Vector3 Rotation::vee(const so3& S)
{
    return ugl::Vector3{S(2,1)-S(1,2), S(0,2)-S(2,0), S(1,0)-S(0,1)} * 0.5;
}

} // namespace ugl::lie
