#include "piper_elevator_manipulation/trajectory_executor.hpp"

#include <algorithm>
#include <cmath>
#include <future>
#include <thread>

namespace piper_elevator_manipulation {

TrajectoryExecutor::TrajectoryExecutor(const rclcpp::Node::SharedPtr& node)
    : node_(node), client_(rclcpp_action::create_client<Follow>(
                       node, "/arm_controller/follow_joint_trajectory")) {}

bool TrajectoryExecutor::cancelAndWait(
    const GoalHandle::SharedPtr& goal_handle) {
  auto cancel = client_->async_cancel_goal(goal_handle);
  if (cancel.wait_for(std::chrono::seconds(2)) != std::future_status::ready) {
    RCLCPP_ERROR(node_->get_logger(),
                 "Controller did not acknowledge trajectory cancellation");
    return false;
  }
  const auto response = cancel.get();
  if (!response || response->goals_canceling.empty()) {
    RCLCPP_ERROR(node_->get_logger(),
                 "Controller did not accept trajectory cancellation");
    return false;
  }
  auto result = client_->async_get_result(goal_handle);
  if (result.wait_for(std::chrono::seconds(2)) != std::future_status::ready) {
    RCLCPP_ERROR(node_->get_logger(),
                 "Controller did not confirm a stopped trajectory");
    return false;
  }
  return result.get().code == rclcpp_action::ResultCode::CANCELED;
}

ExecutionResult TrajectoryExecutor::execute(
    const trajectory_msgs::msg::JointTrajectory& trajectory,
    std::chrono::milliseconds timeout,
    ContactMonitor* contact_monitor,
    PressTravelMonitor* travel_monitor,
    double max_travel_m) {
  if (trajectory.points.size() < 2 || trajectory.joint_names.size() != 5 ||
      timeout.count() <= 0) {
    return {ExecutionStatus::controller_failed, "Invalid trajectory or timeout"};
  }
  if (!client_->wait_for_action_server(std::chrono::seconds(3))) {
    return {ExecutionStatus::controller_failed,
            "arm_controller action server unavailable"};
  }
  if (contact_monitor != nullptr) {
    contact_monitor->reset();
    if (!contact_monitor->isFresh()) {
      return {ExecutionStatus::contact_signal_stale,
              "No fresh contact sample before pressing"};
    }
  }
  if (travel_monitor != nullptr &&
      (!std::isfinite(max_travel_m) || max_travel_m <= 0.0 ||
       !travel_monitor->resetAtCurrentPosition())) {
    return {ExecutionStatus::travel_feedback_stale,
            "No fresh TCP feedback before pressing"};
  }

  Follow::Goal goal;
  goal.trajectory = trajectory;
  auto goal_future = client_->async_send_goal(goal);
  if (goal_future.wait_for(std::chrono::seconds(3)) !=
      std::future_status::ready) {
    return {ExecutionStatus::controller_failed,
            "Controller did not acknowledge trajectory goal"};
  }
  auto goal_handle = goal_future.get();
  if (!goal_handle) {
    return {ExecutionStatus::controller_failed, "Controller rejected trajectory"};
  }
  auto result_future = client_->async_get_result(goal_handle);
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (rclcpp::ok()) {
    if (travel_monitor != nullptr) {
      const auto travel = travel_monitor->forwardTravelMeters();
      if (!travel.has_value()) {
        return cancelAndWait(goal_handle)
            ? ExecutionResult{ExecutionStatus::travel_feedback_stale,
                              "TCP travel feedback became stale"}
            : ExecutionResult{ExecutionStatus::controller_failed,
                              "Could not stop after TCP feedback became stale"};
      }
      if (*travel >= max_travel_m) {
        return cancelAndWait(goal_handle)
            ? ExecutionResult{ExecutionStatus::travel_limit_exceeded,
                              "Measured TCP travel reached the safety cap"}
            : ExecutionResult{ExecutionStatus::controller_failed,
                              "Could not stop at TCP travel safety cap"};
      }
    }
    if (contact_monitor != nullptr) {
      if (!contact_monitor->isFresh()) {
        return cancelAndWait(goal_handle)
            ? ExecutionResult{ExecutionStatus::contact_signal_stale,
                              "Contact signal became stale during press"}
            : ExecutionResult{ExecutionStatus::controller_failed,
                              "Could not stop after contact signal became stale"};
      }
      if (contact_monitor->hasStableContact()) {
        return cancelAndWait(goal_handle)
            ? ExecutionResult{ExecutionStatus::contact_detected,
                              "Stable contact detected; trajectory cancelled"}
            : ExecutionResult{ExecutionStatus::controller_failed,
                              "Contact detected but controller stop was not confirmed"};
      }
    }
    if (result_future.wait_for(std::chrono::milliseconds(0)) ==
        std::future_status::ready) {
      const auto result = result_future.get();
      if (result.code == rclcpp_action::ResultCode::SUCCEEDED &&
          result.result && result.result->error_code == Follow::Result::SUCCESSFUL) {
        return contact_monitor == nullptr
            ? ExecutionResult{ExecutionStatus::completed, "Trajectory completed"}
            : ExecutionResult{ExecutionStatus::contact_not_detected,
                              "Maximum press travel completed without contact"};
      }
      return {ExecutionStatus::controller_failed,
              "Controller result was not successful"};
    }
    if (std::chrono::steady_clock::now() >= deadline) {
      return cancelAndWait(goal_handle)
          ? ExecutionResult{ExecutionStatus::timed_out,
                            "Trajectory execution timed out"}
          : ExecutionResult{ExecutionStatus::controller_failed,
                            "Timed out and controller stop was not confirmed"};
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  cancelAndWait(goal_handle);
  return {ExecutionStatus::controller_failed, "ROS shutdown during execution"};
}

}  // namespace piper_elevator_manipulation
