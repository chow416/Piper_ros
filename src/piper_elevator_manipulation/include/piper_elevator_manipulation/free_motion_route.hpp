#pragma once

#include <moveit/robot_model/robot_model.h>
#include <moveit/task_constructor/container.h>
#include <moveit/task_constructor/solvers/planner_interface.h>
#include <rclcpp/rclcpp.hpp>

#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace piper_elevator_manipulation {

// A waypoint records all five joint angles in radians. It is not a timed
// trajectory: every segment is planned and collision checked again at runtime.
struct JointWaypoint {
  std::string name;
  std::map<std::string, double> positions;
};

struct FreeMotionRoute {
  std::string name;
  std::vector<JointWaypoint> waypoints;
  double max_joint_step_rad;
  double start_bound_tolerance_rad;
  double velocity_scaling;
  double acceleration_scaling;
};

// Returns no route, with a reason, when the configured route cannot be used.
// The caller can then retain the sampling planner as the safe fallback.
std::optional<FreeMotionRoute> loadFreeMotionRoute(
    const rclcpp::Node::SharedPtr& node,
    const moveit::core::RobotModelConstPtr& robot_model,
    const std::string& group_name,
    std::string* reason);

struct FreeMotionBranches {
  std::unique_ptr<moveit::task_constructor::Fallbacks> stage;
  // Non-owning pointer; valid while `stage` or its containing Task exists.
  const moveit::task_constructor::SerialContainer* prerecorded{nullptr};
};

// Both alternatives end at the same target joint state. MTC tries the checked
// waypoint route first, then invokes the sampling planner only if it fails.
FreeMotionBranches makeFreeMotionBranches(
    const std::optional<FreeMotionRoute>& route,
    const std::map<std::string, double>& target_joints,
    const moveit::task_constructor::solvers::PlannerInterfacePtr& sampling_planner,
    const std::string& group_name,
    const std::string& controller_name);

}  // namespace piper_elevator_manipulation
