#pragma once

#include "piper_elevator_manipulation/press_constants.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <map>
#include <mutex>
#include <optional>
#include <string>

namespace piper_elevator_manipulation {

// /joint_states arrives on the node's spin thread. Keeping the newest sample in
// a cache lets start-state capture wait for data without installing a second
// executor on a node that is already being spun, which is what allows the task
// to run more than once in a process.
class JointStateCache {
 public:
  explicit JointStateCache(const rclcpp::Node::SharedPtr& node) : node_(node) {
    subscription_ = node_->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::JointState::ConstSharedPtr message) {
          std::lock_guard<std::mutex> lock(mutex_);
          latest_ = *message;
          received_at_ = std::chrono::steady_clock::now();
          condition_.notify_all();
        });
  }

  // Returns joint1..joint5 from the newest feedback sample, waiting up to
  // `timeout` for anything to arrive. A sample older than `max_age` is refused:
  // a stopped publisher means the hardware interface has faulted, and planning
  // from a frozen pose would command a jump.
  std::optional<std::map<std::string, double>> waitForArmPositions(
      std::chrono::seconds timeout,
      std::chrono::milliseconds max_age = std::chrono::milliseconds(500)) {
    std::unique_lock<std::mutex> lock(mutex_);
    condition_.wait_for(lock, timeout, [this] { return latest_.has_value(); });
    return extractLocked(max_age);
  }

  // Non-blocking version of the same read, used to watch the arm settle after a
  // trajectory.
  std::optional<std::map<std::string, double>> latestArmPositions(
      std::chrono::milliseconds max_age = std::chrono::milliseconds(500)) const {
    std::unique_lock<std::mutex> lock(mutex_);
    return extractLocked(max_age);
  }

 private:
  std::optional<std::map<std::string, double>> extractLocked(
      std::chrono::milliseconds max_age) const {
    if (!latest_.has_value()) {
      return std::nullopt;
    }
    const auto age = std::chrono::steady_clock::now() - received_at_;
    if (age > max_age) {
      RCLCPP_ERROR(node_->get_logger(),
                   "Newest /joint_states sample is %.0f ms old; refusing to "
                   "plan from a frozen pose.",
                   std::chrono::duration<double, std::milli>(age).count());
      return std::nullopt;
    }

    const sensor_msgs::msg::JointState& latest = *latest_;
    std::map<std::string, double> all_positions;
    for (std::size_t i = 0;
         i < latest.name.size() && i < latest.position.size(); ++i) {
      all_positions[latest.name[i]] = latest.position[i];
    }

    std::map<std::string, double> arm_positions;
    for (const char* joint_name : kArmJoints) {
      const auto found = all_positions.find(joint_name);
      if (found == all_positions.end()) {
        RCLCPP_ERROR(node_->get_logger(),
                     "Joint state does not contain required joint '%s'.",
                     joint_name);
        return std::nullopt;
      }
      arm_positions[joint_name] = found->second;
    }
    return arm_positions;
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
  mutable std::mutex mutex_;
  std::condition_variable condition_;
  std::optional<sensor_msgs::msg::JointState> latest_;
  std::chrono::steady_clock::time_point received_at_{};
};

// Compares where the arm actually is against the last point of a trajectory it
// was just told to follow.
//
// This matters more than it looks. The trajectory action reports completion at
// the controller's joint tolerance, which is not the same as the arm having
// arrived: the joints can still be closing the gap when the next phase starts.
// A blind sleep cannot tell "still catching up" from "this is as close as it
// ever gets", and on a press only a few millimetres deep that difference is the
// whole task. So wait until the error stops shrinking, then print the numbers.
// Defined here rather than declared here and defined out of line, for two
// reasons. Its return type is deduced, and a deduced return type cannot be
// forward-declared. And a free function defined at namespace scope in a header
// is not implicitly inline, so it needs the specifier.
//
// `inline` sits on its own line on purpose: the function definition below is
// required to stay byte-identical to the one press_button_demo.cpp used to
// contain, and scripts/verify_pure_move.py compares from the `auto` line down.
inline
auto lastCommandedPositions(
    const trajectory_msgs::msg::JointTrajectory& trajectory) {
  std::map<std::string, double> commanded;
  if (trajectory.points.empty()) {
    return commanded;
  }
  const auto& final_point = trajectory.points.back();
  for (std::size_t i = 0;
       i < final_point.positions.size() && i < trajectory.joint_names.size();
       ++i) {
    commanded[trajectory.joint_names[i]] = final_point.positions[i];
  }
  return commanded;
}

void logJointTracking(const rclcpp::Logger& logger, const std::string& title,
                      const std::map<std::string, double>& commanded,
                      const std::map<std::string, double>& measured);

// Waits for the joints to stop closing the gap, then reports. Bounded, so a
// persistent offset (load deflection, for instance, which no amount of waiting
// will fix) does not stall the task.
void waitForSettledJoints(const rclcpp::Node::SharedPtr& node,
                          JointStateCache& joint_states,
                          const trajectory_msgs::msg::JointTrajectory& trajectory,
                          const std::string& title);

}  // namespace piper_elevator_manipulation
