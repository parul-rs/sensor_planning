// SpacecraftPosePlugin.cc
//
// Gz Harmonic (gz-sim 8) system plugin.
//
// Subscribes to /target_pose_cmd and /chaser_pose_cmd (geometry_msgs/Pose)
// and sets entity poses directly via the ECM each PreUpdate step.
//
// This replaces the subprocess `gz service .../set_pose` architecture:
//   - Zero IPC overhead (runs inside the Gazebo process)
//   - Updates applied before every physics step
//   - Dynamics nodes become simple ROS2 publishers

#include <gz/sim/System.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Model.hh>
#include <gz/plugin/Register.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Quaternion.hh>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <mutex>
#include <string>
#include <thread>

namespace nbv_space
{

class SpacecraftPosePlugin
    : public gz::sim::System,
      public gz::sim::ISystemConfigure,
      public gz::sim::ISystemPreUpdate
{
public:
  // ISystemConfigure
  void Configure(
      const gz::sim::Entity & /*entity*/,
      const std::shared_ptr<const sdf::Element> & /*sdf*/,
      gz::sim::EntityComponentManager & /*ecm*/,
      gz::sim::EventManager & /*eventMgr*/) override
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }

    node_ = std::make_shared<rclcpp::Node>("spacecraft_pose_plugin");

    // Target: dynamics node publishes here instead of calling gz service
    target_sub_ = node_->create_subscription<geometry_msgs::msg::Pose>(
        "/target_pose_cmd", rclcpp::QoS(1).best_effort(),
        [this](const geometry_msgs::msg::Pose::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(mutex_);
          target_pose_   = *msg;
          target_updated_ = true;
        });

    // Chaser: dynamics node publishes here
    chaser_sub_ = node_->create_subscription<geometry_msgs::msg::Pose>(
        "/chaser_pose_cmd", rclcpp::QoS(1).best_effort(),
        [this](const geometry_msgs::msg::Pose::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(mutex_);
          chaser_pose_   = *msg;
          chaser_updated_ = true;
        });

    // Spin in a background thread so callbacks fire independently of the
    // Gazebo update loop
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_);
    spin_thread_ = std::thread([this]() { executor_->spin(); });

    gzmsg << "[SpacecraftPosePlugin] Ready.\n"
          << "  Listening on /target_pose_cmd and /chaser_pose_cmd\n";
  }

  // ISystemPreUpdate
  void PreUpdate(
      const gz::sim::UpdateInfo & /*info*/,
      gz::sim::EntityComponentManager & ecm) override
  {
    std::lock_guard<std::mutex> lock(mutex_);

    if (target_updated_) {
      SetEntityPose(ecm, "target", target_pose_);
      target_updated_ = false;
    }
    if (chaser_updated_) {
      SetEntityPose(ecm, "chaser", chaser_pose_);
      chaser_updated_ = false;
    }
  }

  // Destructor
  ~SpacecraftPosePlugin() override
  {
    if (executor_) {
      executor_->cancel();
    }
    if (spin_thread_.joinable()) {
      spin_thread_.join();
    }
  }

private:
  // Helpers
  void SetEntityPose(
      gz::sim::EntityComponentManager & ecm,
      const std::string & model_name,
      const geometry_msgs::msg::Pose & ros_pose)
  {
    // Walk every Model entity and match by name
    gz::sim::Entity entity = gz::sim::kNullEntity;
    ecm.Each<gz::sim::components::Model,
             gz::sim::components::Name>(
        [&](const gz::sim::Entity & e,
            const gz::sim::components::Model *,
            const gz::sim::components::Name * name_comp) -> bool
        {
          if (name_comp->Data() == model_name) {
            entity = e;
            return false;   // stop iteration
          }
          return true;      // keep going
        });

    if (entity == gz::sim::kNullEntity) {
      // Model not yet spawned: silently skip, will retry next step
      return;
    }

    gz::math::Pose3d pose(
        gz::math::Vector3d(
            ros_pose.position.x,
            ros_pose.position.y,
            ros_pose.position.z),
        gz::math::Quaternion(
            ros_pose.orientation.w,   // gz quaternion ctor order: w, x, y, z
            ros_pose.orientation.x,
            ros_pose.orientation.y,
            ros_pose.orientation.z));

    auto * pose_comp = ecm.Component<gz::sim::components::Pose>(entity);
    if (pose_comp) {
      *pose_comp = gz::sim::components::Pose(pose);
      ecm.SetChanged(
          entity,
          gz::sim::components::Pose::typeId,
          gz::sim::ComponentState::OneTimeChange);
    }
  }

  // Member
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr target_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr chaser_sub_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
  std::thread spin_thread_;

  std::mutex mutex_;
  geometry_msgs::msg::Pose target_pose_;
  geometry_msgs::msg::Pose chaser_pose_;
  bool target_updated_{false};
  bool chaser_updated_{false};
};

}  // namespace nbv_space

// Register as a Gazebo system plugin
GZ_ADD_PLUGIN(
    nbv_space::SpacecraftPosePlugin,
    gz::sim::System,
    nbv_space::SpacecraftPosePlugin::ISystemConfigure,
    nbv_space::SpacecraftPosePlugin::ISystemPreUpdate)