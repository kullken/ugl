#ifndef UGL_TRAJECTORY_CIRCLE_ARC_H
#define UGL_TRAJECTORY_CIRCLE_ARC_H

#include <memory>

#include "ugl/math/vector.h"
#include "ugl/lie_group/pose.h"

#include "ugl/trajectory/trajectory.h"

namespace ugl::trajectory
{

class CircleArc: public LinearTrajectory
{
public:
    explicit CircleArc(double radians);
    CircleArc(double radians, double radius, double duration);
    CircleArc(double radians, double radius, double duration, const ugl::lie::Pose& transform);

    std::unique_ptr<LinearTrajectory> clone() const override;

    double duration() const override { return m_duration; }

    ugl::Vector3 start() const override;
    ugl::Vector3 end() const override;

    ugl::Vector3 pos(double t) const override;
    ugl::Vector3 vel(double t) const override;
    ugl::Vector3 acc(double t) const override;

private:
    /// @brief Compute the angle from local x-axis at time t.
    /// @param t the time in seconds since start of the trajectory
    /// @return The angle in radians.
    double theta(double t) const;

private:
    double m_angle;
    double m_radius = 1.0;
    double m_duration = 1.0;
    ugl::lie::Pose m_transform = ugl::lie::Pose::Identity();
};

} // namespace ugl::trajectory

#endif // UGL_TRAJECTORY_CIRCLE_ARC_H
