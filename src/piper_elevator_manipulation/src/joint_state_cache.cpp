#include "piper_elevator_manipulation/joint_state_cache.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <thread>

namespace piper_elevator_manipulation {
namespace {

constexpr double kRadiansToDegrees = 57.29577951308232;

}  // namespace

void logJointTracking(const rclcpp::Logger& logger, const std::string& title,
                      const std::map<std::string, double>& commanded,
                      const std::map<std::string, double>& measured) {
  if (commanded.empty() || measured.empty()) {
    RCLCPP_WARN(logger, "No joint data to report for %s.", title.c_str());
    return;
  }
  RCLCPP_INFO(logger, "Joint tracking at %s:", title.c_str());
  double worst = 0.0;
  std::string worst_joint;
  for (const auto& [name, want] : commanded) {
    const auto found = measured.find(name);
    if (found == measured.end()) {
      continue;
    }
    const double error = found->second - want;
    if (std::abs(error) > worst) {
      worst = std::abs(error);
      worst_joint = name;
    }
    RCLCPP_INFO(logger, "  %-7s commanded %+8.3f deg  measured %+8.3f deg  "
                        "lags by %+7.3f deg",
                name.c_str(), want * kRadiansToDegrees,
                found->second * kRadiansToDegrees, error * kRadiansToDegrees);
  }
  RCLCPP_INFO(logger, "  worst joint lag: %.3f deg on %s",
              worst * kRadiansToDegrees, worst_joint.c_str());
}

void waitForSettledJoints(const rclcpp::Node::SharedPtr& node,
                          JointStateCache& joint_states,
                          const trajectory_msgs::msg::JointTrajectory& trajectory,
                          const std::string& title) {
  const auto commanded = lastCommandedPositions(trajectory);
  std::map<std::string, double> measured;
  double best = std::numeric_limits<double>::infinity();
  int samples_without_progress = 0;
  const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    const auto sample = joint_states.latestArmPositions();
    if (!sample.has_value()) {
      break;
    }
    measured = *sample;
    double worst = 0.0;
    for (const auto& [name, want] : commanded) {
      const auto found = measured.find(name);
      if (found != measured.end()) {
        worst = std::max(worst, std::abs(found->second - want));
      }
    }
    if (worst < best - 1.0e-4) {
      best = worst;
      samples_without_progress = 0;
    } else if (++samples_without_progress >= 3) {
      break;
    }
  }
  logJointTracking(node->get_logger(), title, commanded, measured);
}

}  // namespace piper_elevator_manipulation
