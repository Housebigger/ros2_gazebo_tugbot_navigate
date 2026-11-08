#include <cmath>
#include <iostream>
#include <stdexcept>
#include "go2_observer_cameras/camera_pose.hpp"

using gz::math::Pose3d;
using gz::math::Vector3d;
using go2_observer_cameras::ChasePose;

void Check(bool condition, const char * explanation)
{
  if (!condition) throw std::runtime_error(explanation);
}

int main()
{
  try {
    const Vector3d offset(-1.2, -0.9, 3.5);
    const Pose3d base(1, 2, 0.34, 0, 0, 0);
    const auto camera = ChasePose(base, offset, true, -0.1);
    Check(camera.Pos().Equal(Vector3d(-0.2, 1.1, 3.84), 1e-9),
      "Camera must remain behind, beside and above the target");
    const auto direction = (base.Pos() + Vector3d(0, 0, -0.1) - camera.Pos()).Normalized();
    Check(camera.Rot().RotateVector(Vector3d::UnitX).Equal(direction, 1e-9),
      "Gazebo camera +X must point at the Go2 body and legs");
    Check(std::abs(camera.Rot().Roll()) < 1e-9, "The camera horizon must stay level");
    const auto turning = ChasePose(Pose3d(1, 2, .34, 0, 0, M_PI_2), offset, true, -.1);
    Check(turning.Pos().Equal(Vector3d(1.9, .8, 3.84), 1e-9),
      "The rear-quarter offset must rotate with target yaw");
    const auto tilting = ChasePose(Pose3d(1, 2, .34, .2, -.12, 0), offset, true, -.1);
    Check(tilting.Equal(camera, 1e-9), "Body roll and pitch must not shake the observer");
    const auto fixed = ChasePose(Pose3d(1, 2, .34, .2, -.12, M_PI_2), offset, false, -.1);
    Check(fixed.Equal(camera, 1e-9), "Fixed-world mode must keep the offset independent of yaw");
    const auto left = ChasePose(Pose3d(1, 2, .34, 0, 0, M_PI-1e-6), offset, true, -.1);
    const auto right = ChasePose(Pose3d(1, 2, .34, 0, 0, -M_PI+1e-6), offset, true, -.1);
    Check(left.Pos().Distance(right.Pos()) < 1e-5,
      "Crossing the yaw wrap must not jump the camera");
    std::cout << "camera pose checks passed\n";
    return 0;
  } catch (const std::exception & e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
