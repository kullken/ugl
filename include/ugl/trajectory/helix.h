#ifndef UGL_TRAJECTORY_HELIX_H
#define UGL_TRAJECTORY_HELIX_H

#include <memory>

#include "ugl/math/vector.h"
#include "ugl/lie_group/pose.h"

#include "ugl/trajectory/trajectory.h"
#include "ugl/trajectory/circle_arc.h"

namespace ugl::trajectory
{

class Helix: public LinearTrajectory
{
public:
    Helix(double radians, double radius, double duration, double z_velocity);
    Helix(double radians, double radius, double duration, double z_velocity, const ugl::lie::Pose& transform);

    std::unique_ptr<LinearTrajectory> clone() const override;

    double duration() const override { return m_circle_arc.duration(); }

    ugl::Vector3 start() const override;
    ugl::Vector3 end() const override;

    ugl::Vector3 pos(double t) const override;
    ugl::Vector3 vel(double t) const override;
    ugl::Vector3 acc(double t) const override;

private:
    CircleArc m_circle_arc;
    double m_z_velocity;
    ugl::lie::Pose m_transform = ugl::lie::Pose::Identity();
};

} // namespace ugl::trajectory

#endif // UGL_TRAJECTORY_HELIX_H
