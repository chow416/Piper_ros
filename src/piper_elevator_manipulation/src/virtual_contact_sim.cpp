#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>

#include <Eigen/Core>

#include <algorithm>
#include <cmath>
#include <memory>
#include <map>
#include <stdexcept>
#include <string>

// Simulation-only adapter: a virtual contact plane, not a physical force
// sensor. Never launch it in a hardware bringup or use it to qualify hardware.
class VirtualContactSim : public rclcpp::Node {
 public:
  VirtualContactSim()
      : rclcpp::Node("piper_virtual_contact_sim",
                     rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)) {
    if (!get_parameter("use_sim_time").as_bool()) {
      throw std::runtime_error("Virtual contact requires use_sim_time=true");
    }
    target_ = Eigen::Vector3d(get_parameter("button_x").as_double(),
                              get_parameter("button_y").as_double(),
                              get_parameter("button_z").as_double());
    direction_ = Eigen::Vector3d(
        get_parameter("press_direction_x").as_double(),
        get_parameter("press_direction_y").as_double(),
        get_parameter("press_direction_z").as_double());
    offset_m_ = get_parameter("virtual_contact_offset_m").as_double();
    if (!target_.allFinite() || !direction_.allFinite() ||
        direction_.norm() < 1.0e-6 || offset_m_ < 0.0 ||
        offset_m_ > 0.01) {
      throw std::invalid_argument("Invalid virtual contact geometry");
    }
    direction_.normalize();

    // Model loading needs a shared node, so finish initialization in start().
  }

  void start() {
    loader_ = std::make_unique<robot_model_loader::RobotModelLoader>(
        shared_from_this());
    model_ = loader_->getModel();
    if (!model_ || !model_->hasLinkModel("press_tcp")) {
      throw std::runtime_error("Virtual contact could not load press_tcp model");
    }
    publisher_ = create_publisher<std_msgs::msg::Bool>(
        "/press/contact", rclcpp::SensorDataQoS());
    subscription_ = create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::JointState::ConstSharedPtr message) {
          std::map<std::string, double> positions;
          for (std::size_t index = 0;
               index < message->name.size() && index < message->position.size();
               ++index) {
            positions[message->name[index]] = message->position[index];
          }
          for (const char* joint : {"joint1", "joint2", "joint3", "joint4", "joint5"}) {
            const auto found = positions.find(joint);
            if (found == positions.end() || !std::isfinite(found->second)) {
              return;  // Missing/stale state must never create a contact sample.
            }
          }
          moveit::core::RobotState state(model_);
          state.setToDefaultValues();
          for (const auto& [name, position] : positions) {
            if (model_->hasJointModel(name) && std::isfinite(position)) {
              state.setVariablePosition(name, position);
            }
          }
          state.update();
          const Eigen::Vector3d tcp =
              state.getGlobalLinkTransform("press_tcp").translation();
          std_msgs::msg::Bool contact;
          const double depth_m = (tcp - target_).dot(direction_);
          contact.data = depth_m >= offset_m_;
          if (contact.data && !previous_contact_) {
            RCLCPP_INFO(get_logger(),
                        "Virtual contact crossed at depth %.4f m", depth_m);
          }
          previous_contact_ = contact.data;
          publisher_->publish(contact);
        });
    RCLCPP_WARN(get_logger(),
                "SIMULATION ONLY: virtual contact triggers %.4f m past the "
                "target plane. This is not hardware contact sensing.", offset_m_);
  }

 private:
  Eigen::Vector3d target_;
  Eigen::Vector3d direction_;
  double offset_m_{0.0};
  bool previous_contact_{false};
  std::unique_ptr<robot_model_loader::RobotModelLoader> loader_;
  moveit::core::RobotModelPtr model_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<VirtualContactSim>();
    node->start();
    rclcpp::spin(node);
  } catch (const std::exception& error) {
    RCLCPP_ERROR(rclcpp::get_logger("piper_virtual_contact_sim"),
                 "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
