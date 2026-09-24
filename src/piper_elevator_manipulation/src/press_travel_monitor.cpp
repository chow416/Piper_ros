#include "piper_elevator_manipulation/press_travel_monitor.hpp"

#include <moveit/robot_state/robot_state.h>

#include <cmath>
#include <map>
#include <stdexcept>
#include <utility>

namespace piper_elevator_manipulation {

PressTravelMonitor::PressTravelMonitor(
    const rclcpp::Node::SharedPtr& node,
    moveit::core::RobotModelConstPtr model,
    std::string tip_link,
    Eigen::Vector3d press_direction,
    std::chrono::milliseconds max_age)
    : model_(std::move(model)),
      tip_link_(std::move(tip_link)),
      press_direction_(press_direction.normalized()),
      max_age_(max_age) {
  if (!model_ || !model_->hasLinkModel(tip_link_) ||
      !press_direction.allFinite() || press_direction.norm() < 1.0e-9 ||
      max_age.count() <= 0) {
    throw std::invalid_argument("Invalid press travel monitor configuration");
  }
  subscription_ = node->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::JointState::ConstSharedPtr message) {
        std::map<std::string, double> positions;
        for (std::size_t i = 0;
             i < message->name.size() && i < message->position.size(); ++i) {
          positions[message->name[i]] = message->position[i];
        }
        for (const char* name : {"joint1", "joint2", "joint3", "joint4", "joint5"}) {
          const auto found = positions.find(name);
          if (found == positions.end() || !std::isfinite(found->second)) {
            return;
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
            state.getGlobalLinkTransform(tip_link_).translation();
        std::lock_guard<std::mutex> lock(mutex_);
        latest_tcp_ = tcp;
        last_sample_ = std::chrono::steady_clock::now();
      });
}

bool PressTravelMonitor::resetAtCurrentPosition() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!latest_tcp_ ||
      std::chrono::steady_clock::now() - last_sample_ > max_age_) {
    return false;
  }
  origin_tcp_ = latest_tcp_;
  return true;
}

bool PressTravelMonitor::isFresh() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return latest_tcp_.has_value() &&
         std::chrono::steady_clock::now() - last_sample_ <= max_age_;
}

std::optional<Eigen::Vector3d> PressTravelMonitor::latestTcpPosition() const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!latest_tcp_ ||
      std::chrono::steady_clock::now() - last_sample_ > max_age_) {
    return std::nullopt;
  }
  return latest_tcp_;
}

std::optional<double> PressTravelMonitor::forwardTravelMeters() const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!latest_tcp_ || !origin_tcp_ ||
      std::chrono::steady_clock::now() - last_sample_ > max_age_) {
    return std::nullopt;
  }
  return (*latest_tcp_ - *origin_tcp_).dot(press_direction_);
}

}  // namespace piper_elevator_manipulation
