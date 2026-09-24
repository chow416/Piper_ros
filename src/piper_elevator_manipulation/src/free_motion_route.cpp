#include "piper_elevator_manipulation/free_motion_route.hpp"

#include <moveit/task_constructor/solvers/joint_interpolation.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/planning_scene/planning_scene.h>

#include <algorithm>
#include <cmath>
#include <set>
#include <utility>

namespace piper_elevator_manipulation {
namespace {

namespace mtc = moveit::task_constructor;

// The simulation's zero-position joints can jitter a few 1e-5 rad beyond
// their hard limits. Correct only that bounded sensor/physics noise. The
// parent planner still checks the corrected start, every sample, and the goal
// for both bounds and collision; larger errors remain planning failures.
class TolerantStartJointInterpolationPlanner final
    : public mtc::solvers::JointInterpolationPlanner {
 public:
  explicit TolerantStartJointInterpolationPlanner(double tolerance_rad)
      : tolerance_rad_(tolerance_rad) {}

  Result plan(const planning_scene::PlanningSceneConstPtr& from,
              const planning_scene::PlanningSceneConstPtr& to,
              const moveit::core::JointModelGroup* group, double timeout,
              robot_trajectory::RobotTrajectoryPtr& result,
              const moveit_msgs::msg::Constraints& constraints) override {
    auto corrected_scene = from->diff();
    auto& state = corrected_scene->getCurrentStateNonConst();
    bool corrected = false;
    for (const auto& variable : group->getVariableNames()) {
      const auto& bounds = state.getRobotModel()->getVariableBounds(variable);
      if (!bounds.position_bounded_) {
        continue;
      }
      const double position = state.getVariablePosition(variable);
      if (!std::isfinite(position)) {
        return {false, "Start state contains a non-finite joint position"};
      }
      const double bounded = std::clamp(
          position, bounds.min_position_, bounds.max_position_);
      const double correction = std::abs(position - bounded);
      if (correction > tolerance_rad_) {
        return {false, "Start state exceeds configured bound tolerance at " +
                           variable};
      }
      if (correction > 0.0) {
        state.setVariablePosition(variable, bounded);
        corrected = true;
      }
    }
    if (corrected) {
      state.update();
      RCLCPP_WARN(rclcpp::get_logger("piper_free_motion"),
                  "Clamped start-state joint-limit noise within %.6f rad before "
                  "collision-checking the prerecorded route.", tolerance_rad_);
    }
    return mtc::solvers::JointInterpolationPlanner::plan(
        corrected_scene, to, group, timeout, result, constraints);
  }

 private:
  double tolerance_rad_;
};

std::unique_ptr<mtc::stages::MoveTo> makeJointMove(
    const std::string& name,
    const mtc::solvers::PlannerInterfacePtr& planner,
    const std::map<std::string, double>& goal,
    const std::string& group_name,
    const std::string& controller_name) {
  auto stage = std::make_unique<mtc::stages::MoveTo>(name, planner);
  stage->setGroup(group_name);
  stage->setGoal(goal);
  mtc::TrajectoryExecutionInfo execution_info;
  execution_info.controller_names = {controller_name};
  stage->setTrajectoryExecutionInfo(std::move(execution_info));
  return stage;
}

bool readPositiveScaling(const rclcpp::Node::SharedPtr& node,
                         const std::string& key, double* value,
                         std::string* reason) {
  if (!node->get_parameter(key, *value) || !std::isfinite(*value) ||
      *value <= 0.0 || *value > 1.0) {
    *reason = key + " must be finite and in (0, 1]";
    return false;
  }
  return true;
}

}  // namespace

std::optional<FreeMotionRoute> loadFreeMotionRoute(
    const rclcpp::Node::SharedPtr& node,
    const moveit::core::RobotModelConstPtr& robot_model,
    const std::string& group_name,
    std::string* reason) {
  const auto* group = robot_model->getJointModelGroup(group_name);
  if (group == nullptr || group->getVariableNames().size() != 5) {
    *reason = "route requires a five-joint planning group '" + group_name + "'";
    return std::nullopt;
  }

  FreeMotionRoute route;
  std::vector<std::string> waypoint_names;
  if (!node->get_parameter("free_motion.route_name", route.name) ||
      route.name.empty() ||
      !node->get_parameter("free_motion.waypoint_names", waypoint_names) ||
      waypoint_names.empty()) {
    *reason = "route_name or waypoint_names is missing/empty";
    return std::nullopt;
  }
  if (!node->get_parameter("free_motion.max_joint_step_rad",
                           route.max_joint_step_rad) ||
      !std::isfinite(route.max_joint_step_rad) ||
      route.max_joint_step_rad <= 0.0 || route.max_joint_step_rad > 0.2) {
    *reason = "free_motion.max_joint_step_rad must be in (0, 0.2]";
    return std::nullopt;
  }
  if (!node->get_parameter("free_motion.start_bound_tolerance_rad",
                           route.start_bound_tolerance_rad) ||
      !std::isfinite(route.start_bound_tolerance_rad) ||
      route.start_bound_tolerance_rad < 0.0 ||
      route.start_bound_tolerance_rad > 0.005) {
    *reason = "free_motion.start_bound_tolerance_rad must be in [0, 0.005]";
    return std::nullopt;
  }
  if (!readPositiveScaling(node, "free_motion.velocity_scaling",
                           &route.velocity_scaling, reason) ||
      !readPositiveScaling(node, "free_motion.acceleration_scaling",
                           &route.acceleration_scaling, reason)) {
    return std::nullopt;
  }

  std::set<std::string> seen_names;
  for (const auto& waypoint_name : waypoint_names) {
    if (waypoint_name.empty() || !seen_names.insert(waypoint_name).second) {
      *reason = "waypoint names must be nonempty and unique";
      return std::nullopt;
    }
    JointWaypoint waypoint;
    waypoint.name = waypoint_name;
    for (const auto& joint_name : group->getVariableNames()) {
      double position = 0.0;
      const std::string key = "free_motion." + waypoint_name + "." + joint_name;
      if (!node->get_parameter(key, position) || !std::isfinite(position)) {
        *reason = key + " is missing or not finite";
        return std::nullopt;
      }
      const auto& bounds = robot_model->getVariableBounds(joint_name);
      if (position < bounds.min_position_ || position > bounds.max_position_) {
        *reason = key + " is outside the URDF joint limits";
        return std::nullopt;
      }
      waypoint.positions[joint_name] = position;
    }
    route.waypoints.push_back(std::move(waypoint));
  }
  return route;
}

FreeMotionBranches makeFreeMotionBranches(
    const std::optional<FreeMotionRoute>& route,
    const std::map<std::string, double>& target_joints,
    const mtc::solvers::PlannerInterfacePtr& sampling_planner,
    const std::string& group_name,
    const std::string& controller_name) {
  FreeMotionBranches result;
  result.stage = std::make_unique<mtc::Fallbacks>("free motion: recorded route, then sampling");

  if (route.has_value()) {
    auto prerecorded = std::make_unique<mtc::SerialContainer>(
        "prerecorded route: " + route->name);
    result.prerecorded = prerecorded.get();

    auto route_planner =
        std::make_shared<TolerantStartJointInterpolationPlanner>(
            route->start_bound_tolerance_rad);
    route_planner->setProperty("max_step", route->max_joint_step_rad);
    route_planner->setMaxVelocityScalingFactor(route->velocity_scaling);
    route_planner->setMaxAccelerationScalingFactor(route->acceleration_scaling);

    for (const auto& waypoint : route->waypoints) {
      prerecorded->add(makeJointMove("via " + waypoint.name, route_planner,
                                     waypoint.positions, group_name, controller_name));
    }
    prerecorded->add(makeJointMove("from route to computed pre-press", route_planner,
                                   target_joints, group_name, controller_name));
    result.stage->add(std::move(prerecorded));
  }

  result.stage->add(makeJointMove("sampling fallback to pre-press", sampling_planner,
                                  target_joints, group_name, controller_name));
  return result;
}

}  // namespace piper_elevator_manipulation
