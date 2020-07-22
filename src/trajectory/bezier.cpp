#include "ugl/trajectory/bezier.h"

#include "ugl/math/vector.h"

namespace ugl::trajectory
{

Bezier<5> createPenticBezier(double duration, ugl::Vector3 pos0, ugl::Vector3 vel0, ugl::Vector3 acc0, ugl::Vector3 pos1, ugl::Vector3 vel1, ugl::Vector3 acc1)
{
    const ugl::Vector3 p0 = pos0;
    const ugl::Vector3 p1 = duration/5 * vel0 + p0;
    const ugl::Vector3 p2 = duration*duration/20 * acc0 + 2*p1 - p0;

    const ugl::Vector3 p5 = pos1;
    const ugl::Vector3 p4 = -duration/5 * vel1 + p5;
    const ugl::Vector3 p3 = duration*duration/20 * acc1 + 2*p4 - p5;

    return Bezier<5>{duration, {p0, p1, p2, p3, p4, p5}};
}

} // namespace ugl::trajectory