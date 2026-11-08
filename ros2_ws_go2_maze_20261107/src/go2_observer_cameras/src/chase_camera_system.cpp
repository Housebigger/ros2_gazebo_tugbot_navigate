#include <chrono>
#include <memory>
#include <string>

#include <gz/common/Console.hh>
#include <gz/msgs/pose_v.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Static.hh>
#include <gz/transport/Node.hh>

#include "go2_observer_cameras/camera_pose.hpp"

namespace go2_observer_cameras
{
// This system owns only an independent, collision-free camera model. It never
// writes the Go2 target's pose, velocities, forces or joint components.
class ChaseCameraSystem : public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPreUpdate,
  public gz::sim::ISystemPostUpdate
{
public:
  void Configure(const gz::sim::Entity & entity,
    const std::shared_ptr<const sdf::Element> & sdf,
    gz::sim::EntityComponentManager & ecm, gz::sim::EventManager &) override
  {
    rig_ = gz::sim::Model(entity);
    targetName_ = sdf->Get<std::string>("target_model", "go2").first;
    targetLinkName_ = sdf->Get<std::string>("target_link", "base_link").first;
    offset_ = sdf->Get<gz::math::Vector3d>("offset", {-1.2, -.9, 3.5}).first;
    focusHeight_ = sdf->Get<double>("focus_height", -.1).first;
    followYaw_ = sdf->Get<bool>("follow_yaw", true).first;
    const auto name = ecm.Component<gz::sim::components::Name>(entity);
    const auto isStatic = ecm.Component<gz::sim::components::Static>(entity);
    enabled_ = rig_.Valid(ecm) && name && name->Data() != targetName_ &&
      isStatic && isStatic->Data() && offset_.Z() > 0 && offset_.Length() > .1;
    if (!enabled_) {
      gzerr << "Chase camera requires a separate static camera model and an upward offset\n";
      return;
    }
    tfPublisher_ = node_.Advertise<gz::msgs::Pose_V>("/camera/chase/tf");
  }

  void PreUpdate(const gz::sim::UpdateInfo & info,
    gz::sim::EntityComponentManager & ecm) override
  {
    if (!enabled_ || info.paused) return;
    const auto target = ecm.EntityByComponents(
      gz::sim::components::Model(), gz::sim::components::Name(targetName_));
    if (target == gz::sim::kNullEntity) return;
    const auto link = gz::sim::Model(target).LinkByName(ecm, targetLinkName_);
    if (link == gz::sim::kNullEntity) return;
    rig_.SetWorldPoseCmd(ecm, ChasePose(gz::sim::worldPose(link, ecm),
      offset_, followYaw_, focusHeight_));
    tracking_ = true;
  }

  void PostUpdate(const gz::sim::UpdateInfo & info,
    const gz::sim::EntityComponentManager & ecm) override
  {
    if (!enabled_ || !tracking_ || info.paused) return;
    const auto now = std::chrono::duration_cast<std::chrono::nanoseconds>(info.simTime);
    if (now >= lastPublication_ && now-lastPublication_ < std::chrono::milliseconds(20)) return;
    lastPublication_ = now;
    // Publish the measured ECS pose after physics, rather than just the pose
    // command. Optical +Z is forward, +X right, +Y down.
    const auto actual = gz::sim::worldPose(rig_.Entity(), ecm);
    const auto optical = actual.Rot() * gz::math::Quaterniond(-M_PI_2, 0, -M_PI_2);
    gz::msgs::Pose_V transforms;
    auto * pose = transforms.add_pose();
    auto * header = pose->mutable_header();
    header->mutable_stamp()->set_sec(now.count()/1000000000);
    header->mutable_stamp()->set_nsec(now.count()%1000000000);
    auto * parent = header->add_data();
    parent->set_key("frame_id"); parent->add_value("odom");
    auto * child = header->add_data();
    child->set_key("child_frame_id"); child->add_value("camera_chase_optical");
    pose->set_name("camera_chase_optical");
    pose->mutable_position()->set_x(actual.Pos().X());
    pose->mutable_position()->set_y(actual.Pos().Y());
    pose->mutable_position()->set_z(actual.Pos().Z());
    pose->mutable_orientation()->set_x(optical.X());
    pose->mutable_orientation()->set_y(optical.Y());
    pose->mutable_orientation()->set_z(optical.Z());
    pose->mutable_orientation()->set_w(optical.W());
    tfPublisher_.Publish(transforms);
  }

private:
  gz::sim::Model rig_{gz::sim::kNullEntity};
  std::string targetName_, targetLinkName_;
  gz::math::Vector3d offset_;
  double focusHeight_{-.1};
  bool followYaw_{true}, enabled_{false}, tracking_{false};
  std::chrono::nanoseconds lastPublication_{0};
  gz::transport::Node node_;
  gz::transport::Node::Publisher tfPublisher_;
};
}

GZ_ADD_PLUGIN(go2_observer_cameras::ChaseCameraSystem, gz::sim::System,
  gz::sim::ISystemConfigure, gz::sim::ISystemPreUpdate, gz::sim::ISystemPostUpdate)
GZ_ADD_PLUGIN_ALIAS(go2_observer_cameras::ChaseCameraSystem,
  "go2_observer_cameras::ChaseCameraSystem")
