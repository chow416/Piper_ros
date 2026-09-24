#pragma once

#include <moveit/robot_model/robot_model.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <Eigen/Core>

#include <chrono>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

namespace piper_elevator_manipulation {

// Independent safety cap: calculate actual TCP displacement from fresh joint
// feedback instead of trusting the commanded trajectory endpoint.
class PressTravelMonitor {
 public:
  PressTravelMonitor(const rclcpp::Node::SharedPtr& node,
                     moveit::core::RobotModelConstPtr model,
                     std::string tip_link,
                     Eigen::Vector3d press_direction,
                     std::chrono::milliseconds max_age);

  bool resetAtCurrentPosition();
  bool isFresh() const;
  std::optional<Eigen::Vector3d> latestTcpPosition() const;
  std::optional<double> forwardTravelMeters() const;

 private:
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
  moveit::core::RobotModelConstPtr model_;
  std::string tip_link_;
  Eigen::Vector3d press_direction_;
  std::chrono::milliseconds max_age_;
  mutable std::mutex mutex_;
  std::optional<Eigen::Vector3d> latest_tcp_;
  std::optional<Eigen::Vector3d> origin_tcp_;
  std::chrono::steady_clock::time_point last_sample_{};
};

}  // namespace piper_elevator_manipulation
