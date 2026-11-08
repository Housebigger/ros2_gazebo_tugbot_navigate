#pragma once
#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>
#include <cmath>

namespace go2_observer_cameras
{
inline gz::math::Pose3d ChasePose(
  const gz::math::Pose3d & target, const gz::math::Vector3d & offset,
  bool followYaw, double focusHeight)
{
  const gz::math::Quaterniond heading(0, 0, followYaw ? target.Rot().Yaw() : 0);
  const auto position = target.Pos() + heading.RotateVector(offset);
  const auto direction = target.Pos() + gz::math::Vector3d(0, 0, focusHeight) - position;
  const double yaw = std::atan2(direction.Y(), direction.X());
  const double pitch = std::atan2(-direction.Z(), std::hypot(direction.X(), direction.Y()));
  return {position, gz::math::Quaterniond(0, pitch, yaw)};
}
}
