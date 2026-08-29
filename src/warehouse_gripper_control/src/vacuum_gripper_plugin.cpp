// vacuum_gripper_plugin.cpp
// ==========================================================================
// Gazebo (classic, gazebo11) ModelPlugin that turns the passive suction-cup
// contact sensor already defined in vacuum_gripper.xacro into a real,
// physically-attaching vacuum gripper.
//
// This mirrors how an industrial vacuum gripper actually works and is
// actually controlled: a digital on/off (here: two Trigger services) plus a
// vacuum-switch feedback line reporting whether suction actually grabbed
// something (here: /vacuum_gripper/state). It does NOT model an "open/close"
// jaw — vacuum end effectors don't have one.
//
// Mechanism (same technique as the well-known gazebo_ros_link_attacher /
// linkattacher_plugin packages): on ON, if the suction cup's contact sensor
// currently reports touching another model, create a physics "fixed" joint
// between the suction cup link and that model's link. On OFF, destroy the
// joint so the object drops under gravity.
//
// Topics/services:
//   Sub  /vacuum_gripper/contact_states   gazebo_msgs/msg/ContactsState
//        (published by the libgazebo_ros_bumper.so sensor plugin already
//         attached to suction_cup_link in vacuum_gripper.xacro)
//   Srv  /vacuum_gripper/on               std_srvs/srv/Trigger
//   Srv  /vacuum_gripper/off              std_srvs/srv/Trigger
//   Pub  /vacuum_gripper/state            std_msgs/msg/Bool (10 Hz)
//   Pub  /diagnostics                     diagnostic_msgs/msg/DiagnosticArray
//        (attached/detached state + whether the contact sensor is alive,
//         picked up by warehouse_bringup's diagnostic_aggregator)
//
// SDF parameter:
//   <link_name> — link the plugin attaches to (default: suction_cup_link)
// ==========================================================================

#include <memory>
#include <mutex>
#include <string>

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_msgs/msg/bool.hpp>
#include <gazebo_msgs/msg/contacts_state.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>

namespace warehouse_gripper_control
{

class VacuumGripperPlugin : public gazebo::ModelPlugin
{
public:
  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override
  {
    model_ = model;

    std::string link_name = "suction_cup_link";
    if (sdf->HasElement("link_name")) {
      link_name = sdf->Get<std::string>("link_name");
    }

    suction_link_ = model_->GetLink(link_name);
    if (!suction_link_) {
      gzerr << "[vacuum_gripper_plugin] link '" << link_name
            << "' not found on model '" << model_->GetName()
            << "' — plugin will not attach.\n";
      return;
    }

    ros_node_ = gazebo_ros::Node::Get(sdf);

    contact_sub_ = ros_node_->create_subscription<gazebo_msgs::msg::ContactsState>(
      "/vacuum_gripper/contact_states", rclcpp::SensorDataQoS(),
      std::bind(&VacuumGripperPlugin::OnContact, this, std::placeholders::_1));

    state_pub_ = ros_node_->create_publisher<std_msgs::msg::Bool>(
      "/vacuum_gripper/state", 10);

    on_srv_ = ros_node_->create_service<std_srvs::srv::Trigger>(
      "/vacuum_gripper/on",
      std::bind(
        &VacuumGripperPlugin::OnVacuumOn, this,
        std::placeholders::_1, std::placeholders::_2));

    off_srv_ = ros_node_->create_service<std_srvs::srv::Trigger>(
      "/vacuum_gripper/off",
      std::bind(
        &VacuumGripperPlugin::OnVacuumOff, this,
        std::placeholders::_1, std::placeholders::_2));

    diag_pub_ = ros_node_->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics", 10);
    last_contact_msg_time_ = ros_node_->now();

    state_timer_ = ros_node_->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&VacuumGripperPlugin::PublishState, this));

    RCLCPP_INFO(
      ros_node_->get_logger(),
      "VacuumGripperPlugin ready on link '%s' (model '%s').",
      link_name.c_str(), model_->GetName().c_str());
  }

private:
  void OnContact(const gazebo_msgs::msg::ContactsState::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    last_contact_msg_time_ = ros_node_->now();
    contact_model_.clear();
    for (const auto & state : msg->states) {
      const std::string other =
        PickOtherModel(state.collision1_name, state.collision2_name);
      if (!other.empty()) {
        contact_model_ = other;
        break;
      }
    }
  }

  // Given the two scoped collision names of a contact ("model::link::collision"),
  // return the model name on the side that is NOT this gripper's own model.
  std::string PickOtherModel(const std::string & a, const std::string & b) const
  {
    const std::string my_model = model_->GetName();
    const std::string model_a = a.substr(0, a.find("::"));
    const std::string model_b = b.substr(0, b.find("::"));
    if (model_a != my_model) {return model_a;}
    if (model_b != my_model) {return model_b;}
    return "";
  }

  void OnVacuumOn(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    std::lock_guard<std::mutex> lock(mutex_);

    if (attached_) {
      response->success = true;
      response->message = "already attached to " + attached_model_;
      return;
    }

    if (contact_model_.empty()) {
      response->success = false;
      response->message = "vacuum ON requested but no object in contact";
      return;
    }

    gazebo::physics::WorldPtr world = model_->GetWorld();
    gazebo::physics::ModelPtr target_model = world->ModelByName(contact_model_);
    if (!target_model || target_model->GetLinks().empty()) {
      response->success = false;
      response->message = "target model '" + contact_model_ + "' not found";
      contact_model_.clear();
      return;
    }
    gazebo::physics::LinkPtr target_link = target_model->GetLinks()[0];

    attach_joint_ = world->Physics()->CreateJoint("fixed", model_);
    attach_joint_->Load(suction_link_, target_link, ignition::math::Pose3d());
    attach_joint_->Init();

    attached_ = true;
    attached_model_ = contact_model_;

    response->success = true;
    response->message = "attached " + attached_model_;
    RCLCPP_INFO(
      ros_node_->get_logger(), "Vacuum ON — attached '%s'.",
      attached_model_.c_str());
  }

  void OnVacuumOff(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!attached_) {
      response->success = true;
      response->message = "already detached";
      return;
    }

    if (attach_joint_) {
      attach_joint_->Detach();
      attach_joint_.reset();
    }

    RCLCPP_INFO(
      ros_node_->get_logger(), "Vacuum OFF — released '%s'.",
      attached_model_.c_str());

    attached_ = false;
    attached_model_.clear();

    response->success = true;
    response->message = "detached";
  }

  void PublishState()
  {
    bool attached;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      attached = attached_;
    }
    std_msgs::msg::Bool msg;
    msg.data = attached;
    state_pub_->publish(msg);
    PublishDiagnostics(attached);
  }

  void PublishDiagnostics(bool attached)
  {
    const double sensor_age_s = (ros_node_->now() - last_contact_msg_time_).seconds();
    const bool sensor_alive = sensor_age_s < 2.0;   // bumper publishes continuously

    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "warehouse_gripper_control: Gripper";
    status.hardware_id = "vacuum_gripper";
    if (!sensor_alive) {
      status.level   = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      status.message = "contact sensor stale (no /vacuum_gripper/contact_states)";
    } else if (attached) {
      status.level   = diagnostic_msgs::msg::DiagnosticStatus::OK;
      status.message = "attached: " + attached_model_;
    } else {
      status.level   = diagnostic_msgs::msg::DiagnosticStatus::OK;
      status.message = "idle, no object attached";
    }
    diagnostic_msgs::msg::KeyValue kv;
    kv.key = "attached"; kv.value = attached ? "true" : "false";
    status.values.push_back(kv);

    diagnostic_msgs::msg::DiagnosticArray arr;
    arr.header.stamp = ros_node_->now();
    arr.status.push_back(status);
    diag_pub_->publish(arr);
  }

  gazebo::physics::ModelPtr model_;
  gazebo::physics::LinkPtr suction_link_;
  gazebo::physics::JointPtr attach_joint_;

  gazebo_ros::Node::SharedPtr ros_node_;
  rclcpp::Subscription<gazebo_msgs::msg::ContactsState>::SharedPtr contact_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr state_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr on_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr off_srv_;
  rclcpp::TimerBase::SharedPtr state_timer_;
  rclcpp::Time last_contact_msg_time_;

  std::mutex mutex_;
  std::string contact_model_;
  bool attached_{false};
  std::string attached_model_;
};

GZ_REGISTER_MODEL_PLUGIN(VacuumGripperPlugin)

}  // namespace warehouse_gripper_control
