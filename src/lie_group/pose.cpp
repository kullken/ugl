#include "ugl/lie_group/pose.h"

#include "ugl/math/vector.h"
#include "ugl/math/matrix.h"

#include "ugl/lie_group/rotation.h"

namespace ugl::lie
{

Pose Pose::exp(const Vector<6>& u)
{
    // TODO: Analytical solution
    return Pose{math::exp(hat(u))};
}

Vector<6> Pose::log(const Pose& T)
{
    // TODO: Analytical solution
    return vee(math::log(T.matrix()));
}

se_3 Pose::hat(const Vector<6>& u)
{
    se_3 U = se_3::Zero();
    U.block<3,3>(0,0) = SO3::hat(u.segment<3>(0));
    U.block<3,1>(0,3) = u.segment<3>(3);
    return U;
}

Vector<6> Pose::vee(const se_3& U)
{
    Vector<6> u;
    u << SO3::vee(U.block<3,3>(0,0)), U.block<3,1>(0,3);
    return u;
}

} // namespace ugl::lie
