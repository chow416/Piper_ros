#pragma once

#include "piper_elevator_manipulation/contact_monitor.hpp"
#include "piper_elevator_manipulation/press_travel_monitor.hpp"

#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include <chrono>
#include <memory>
#include <string>

namespace piper_elevator_manipulation {

enum class ExecutionStatus {
  completed,
  contact_detected,
  contact_not_detected,
  contact_signal_stale,
  travel_feedback_stale,
  travel_limit_exceeded,
  controller_failed,
  timed_out,
};

struct ExecutionResult {
  ExecutionStatus status;
  std::string detail;
};

// Sends only already-validated trajectories. Press mode cancels the active
// controller goal on stable contact; completing maximum travel is a failure.
class TrajectoryExecutor {
 public:
  explicit TrajectoryExecutor(const rclcpp::Node::SharedPtr& node);

  ExecutionResult execute(const trajectory_msgs::msg::JointTrajectory& trajectory,
                          std::chrono::milliseconds timeout,
                          ContactMonitor* contact_monitor = nullptr,
                          PressTravelMonitor* travel_monitor = nullptr,
                          double max_travel_m = 0.0);

 private:
  using Follow = control_msgs::action::FollowJointTrajectory;
  using GoalHandle = rclcpp_action::ClientGoalHandle<Follow>;

  bool cancelAndWait(const GoalHandle::SharedPtr& goal_handle);

  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<Follow>::SharedPtr client_;
};

}  // namespace piper_elevator_manipulation
