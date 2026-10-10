#include "piper_elevator_manipulation/press_solution.hpp"

#include "piper_elevator_manipulation/press_constants.hpp"

#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/task_constructor/stage.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <cstdint>

namespace piper_elevator_manipulation {

void collectMotions(const mtc::SolutionBase& solution,
                    std::vector<PlannedMotion>* motions) {
  if (const auto* sequence = dynamic_cast<const mtc::SolutionSequence*>(&solution)) {
    for (const auto* part : sequence->solutions()) {
      collectMotions(*part, motions);
    }
    return;
  }
  if (const auto* wrapped = dynamic_cast<const mtc::WrappedSolution*>(&solution)) {
    collectMotions(*wrapped->wrapped(), motions);
    return;
  }
  const auto* segment = dynamic_cast<const mtc::SubTrajectory*>(&solution);
  if (segment == nullptr || segment->trajectory() == nullptr ||
      segment->trajectory()->getWayPointCount() < 2 ||
      solution.creator() == nullptr) {
    return;
  }
  moveit_msgs::msg::RobotTrajectory message;
  segment->trajectory()->getRobotTrajectoryMsg(
      message, {kArmJoints[0], kArmJoints[1], kArmJoints[2],
                kArmJoints[3], kArmJoints[4]});
  motions->push_back({solution.creator()->name(), message.joint_trajectory});
}

std::chrono::milliseconds motionTimeout(
    const trajectory_msgs::msg::JointTrajectory& trajectory) {
  const double duration = rclcpp::Duration(
      trajectory.points.back().time_from_start).seconds();
  return std::chrono::milliseconds(static_cast<int64_t>(
      std::max(5.0, duration * 3.0 + 5.0) * 1000.0));
}

bool solutionUsesStage(const mtc::SolutionBase& solution,
                       const mtc::Stage* sought_stage) {
  if (solution.creator() == sought_stage) {
    return true;
  }
  if (const auto* sequence = dynamic_cast<const mtc::SolutionSequence*>(&solution)) {
    for (const auto* segment : sequence->solutions()) {
      if (solutionUsesStage(*segment, sought_stage)) {
        return true;
      }
    }
  }
  if (const auto* wrapped = dynamic_cast<const mtc::WrappedSolution*>(&solution)) {
    return solutionUsesStage(*wrapped->wrapped(), sought_stage);
  }
  return false;
}

// Duration of a planned trajectory, used to report how long each phase is
// expected to take. Worth logging separately from the wall-clock time, because
// it is the number the speed scaling factors control and it can be read from a
// dry run without moving the arm.
double plannedSeconds(const trajectory_msgs::msg::JointTrajectory& trajectory) {
  if (trajectory.points.empty()) {
    return 0.0;
  }
  return rclcpp::Duration(trajectory.points.back().time_from_start).seconds();
}

}  // namespace piper_elevator_manipulation
