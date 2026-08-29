// conveyor_belt_plugin.cpp
// ==========================================================================
// Drives conveyor_belt's prismatic "belt_joint" at a commanded velocity so
// parcels actually move along the belt, the way a real Amazon-style
// conveyor line runs continuously rather than teleporting products.
//
// Starts moving immediately at <initial_velocity> (default 0.12 m/s) so the
// belt is live as soon as Gazebo starts — no manual publish required — and
// can be changed at runtime via /conveyor/velocity (std_msgs/Float64).
//
// SDF parameters:
//   <joint_name>       joint to drive (default: belt_joint)
//   <initial_velocity> m/s, applied from the first world update (default 0.12)
// ==========================================================================

#include <memory>
#include <mutex>
#include <string>

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

namespace warehouse_gazebo
{

class ConveyorBeltPlugin : public gazebo::ModelPlugin
{
public:
  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override
  {
    model_ = model;

    std::string joint_name = "belt_joint";
    if (sdf->HasElement("joint_name")) {
      joint_name = sdf->Get<std::string>("joint_name");
    }
    if (sdf->HasElement("initial_velocity")) {
      velocity_ = sdf->Get<double>("initial_velocity");
    }

    joint_ = model_->GetJoint(joint_name);
    if (!joint_) {
      gzerr << "[conveyor_belt_plugin] joint '" << joint_name
            << "' not found on model '" << model_->GetName() << "'.\n";
      return;
    }

    ros_node_ = gazebo_ros::Node::Get(sdf);
    vel_sub_ = ros_node_->create_subscription<std_msgs::msg::Float64>(
      "/conveyor/velocity", 10,
      std::bind(&ConveyorBeltPlugin::OnVelocityCmd, this, std::placeholders::_1));

    update_conn_ = gazebo::event::Events::ConnectWorldUpdateBegin(
      std::bind(&ConveyorBeltPlugin::OnUpdate, this));

    RCLCPP_INFO(
      ros_node_->get_logger(),
      "ConveyorBeltPlugin driving '%s' at %.3f m/s (change via /conveyor/velocity).",
      joint_name.c_str(), velocity_);
  }

private:
  void OnVelocityCmd(const std_msgs::msg::Float64::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    velocity_ = msg->data;
  }

  void OnUpdate()
  {
    double v;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      v = velocity_;
    }
    joint_->SetVelocity(0, v);
  }

  gazebo::physics::ModelPtr model_;
  gazebo::physics::JointPtr joint_;
  gazebo::event::ConnectionPtr update_conn_;
  gazebo_ros::Node::SharedPtr ros_node_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr vel_sub_;

  std::mutex mutex_;
  double velocity_{0.12};
};

GZ_REGISTER_MODEL_PLUGIN(ConveyorBeltPlugin)

}  // namespace warehouse_gazebo
