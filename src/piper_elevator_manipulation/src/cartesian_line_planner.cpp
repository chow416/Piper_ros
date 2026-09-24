#include "piper_elevator_manipulation/cartesian_line_planner.hpp"

#include "piper_elevator_manipulation/five_dof_ik.hpp"

#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/time_parameterization.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <map>
#include <stdexcept>
#include <utility>

namespace piper_elevator_manipulation {
namespace {

std::map<std::string, double> jointPositions(
    const moveit::core::RobotState& state,
    const moveit::core::JointModelGroup* group) {
  std::map<std::string, double> positions;
  for (const auto& name : group->getVariableNames()) {
    positions[name] = state.getVariablePosition(name);
  }
  return positions;
}

double axisError(const moveit::core::RobotState& state,
                 const std::string& tip_link,
                 const Eigen::Vector3d& panel_normal) {
  const auto& transform = state.getGlobalLinkTransform(tip_link);
  const double cosine = std::clamp(
      transform.linear().col(2).dot(panel_normal), -1.0, 1.0);
  return std::acos(cosine);
}

}  // namespace

CartesianLinePlanner::CartesianLinePlanner(
    std::string tip_link, Eigen::Vector3d panel_normal,
    CartesianLineOptions options)
    : tip_link_(std::move(tip_link)),
      panel_normal_(panel_normal.normalized()),
      options_(options) {
  if (!panel_normal.allFinite() || panel_normal.norm() < 1.0e-9 ||
      !std::isfinite(options_.max_translation_step_m) ||
      !std::isfinite(options_.max_lateral_error_m) ||
      !std::isfinite(options_.max_joint_jump_rad) ||
      !std::isfinite(options_.max_start_bound_error_rad) ||
      !std::isfinite(options_.position_tolerance_m) ||
      !std::isfinite(options_.direction_tolerance_rad) ||
      options_.max_translation_step_m < 0.0005 ||
      options_.max_translation_step_m > 0.01 ||
      options_.max_lateral_error_m < 0.0 ||
      options_.max_lateral_error_m > 0.005 ||
      options_.max_joint_jump_rad <= 0.0 ||
      options_.max_joint_jump_rad > 0.35 ||
      options_.max_start_bound_error_rad < 0.0 ||
      options_.max_start_bound_error_rad > 0.005 ||
      options_.position_tolerance_m <= 0.0 ||
      options_.position_tolerance_m > 0.003 ||
      options_.direction_tolerance_rad <= 0.0 ||
      options_.direction_tolerance_rad > 0.10) {
    throw std::invalid_argument("Invalid Cartesian line planner options");
  }
}

void CartesianLinePlanner::init(
    const moveit::core::RobotModelConstPtr& robot_model) {
  if (!robot_model->hasLinkModel(tip_link_)) {
    throw std::invalid_argument("Unknown Cartesian TCP link: " + tip_link_);
  }
}

CartesianLinePlanner::Result CartesianLinePlanner::plan(
    const planning_scene::PlanningSceneConstPtr& from,
    const planning_scene::PlanningSceneConstPtr& to,
    const moveit::core::JointModelGroup* group, double timeout,
    robot_trajectory::RobotTrajectoryPtr& result,
    const moveit_msgs::msg::Constraints& constraints) {
  if (group == nullptr || group->getVariableNames().size() != 5 ||
      !panel_normal_.allFinite() || timeout <= 0.0) {
    return {false, "Invalid Cartesian planning group, direction, or timeout"};
  }
  timeout = std::min(timeout, 20.0);
  if (!constraints.joint_constraints.empty() ||
      !constraints.position_constraints.empty() ||
      !constraints.orientation_constraints.empty() ||
      !constraints.visibility_constraints.empty()) {
    return {false, "Cartesian line planner does not support path constraints"};
  }
  const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(timeout);
  auto corrected_scene = from->diff();
  auto& start = corrected_scene->getCurrentStateNonConst();
  for (const auto& name : group->getVariableNames()) {
    const auto& bounds = start.getRobotModel()->getVariableBounds(name);
    const double original = start.getVariablePosition(name);
    if (!std::isfinite(original)) {
      return {false, "Non-finite Cartesian start joint: " + name};
    }
    const double bounded = std::clamp(
        original, bounds.min_position_, bounds.max_position_);
    if (std::abs(original - bounded) > options_.max_start_bound_error_rad) {
      return {false, "Cartesian start joint exceeds bound tolerance: " + name};
    }
    start.setVariablePosition(name, bounded);
  }
  start.update();
  if (!start.satisfiesBounds(group) ||
      corrected_scene->isStateColliding(start, group->getName())) {
    return {false, "Cartesian start state violates bounds or is in collision"};
  }

  const auto& goal = to->getCurrentState();
  if (!goal.satisfiesBounds(group) ||
      corrected_scene->isStateColliding(goal, group->getName())) {
    return {false, "Cartesian goal violates bounds or is in collision"};
  }
  const Eigen::Vector3d start_tcp =
      start.getGlobalLinkTransform(tip_link_).translation();
  const Eigen::Vector3d goal_tcp =
      goal.getGlobalLinkTransform(tip_link_).translation();
  const Eigen::Vector3d displacement = goal_tcp - start_tcp;
  const double axial_distance = displacement.dot(panel_normal_);
  const double lateral_error =
      (displacement - axial_distance * panel_normal_).norm();
  if (std::abs(axial_distance) < 1.0e-5 ||
      lateral_error > options_.max_lateral_error_m) {
    return {false, "TCP endpoints do not define a panel-normal line"};
  }
  if (axisError(start, tip_link_, panel_normal_) >
          options_.direction_tolerance_rad ||
      axisError(goal, tip_link_, panel_normal_) >
          options_.direction_tolerance_rad) {
    return {false, "TCP press axis is not aligned with panel normal"};
  }

  const int samples = std::max(1, static_cast<int>(
      std::ceil(std::abs(axial_distance) / options_.max_translation_step_m)));
  result = std::make_shared<robot_trajectory::RobotTrajectory>(
      from->getRobotModel(), group);
  result->addSuffixWayPoint(start, 0.0);

  FiveDofIkOptions ik_options;
  ik_options.random_restarts = 0;
  ik_options.position_tolerance = options_.position_tolerance_m;
  ik_options.direction_tolerance_rad = options_.direction_tolerance_rad;
  FiveDofIkSolver local_ik(ik_options);
  FiveDofIkSolver recovery_ik;
  auto previous = jointPositions(start, group);
  for (int sample = 1; sample <= samples; ++sample) {
    if (std::chrono::steady_clock::now() >= deadline) {
      return {false, "Cartesian line planning timed out"};
    }
    const double fraction = static_cast<double>(sample) / samples;
    const Eigen::Vector3d position =
        start_tcp + fraction * axial_distance * panel_normal_;
    std::string reason;
    auto joints = local_ik.solve(from->getRobotModel(), group->getName(),
                                 tip_link_, previous, position,
                                 panel_normal_, &reason);
    if (!joints.has_value()) {
      joints = recovery_ik.solve(from->getRobotModel(), group->getName(),
                                 tip_link_, previous, position,
                                 panel_normal_, &reason);
    }
    if (!joints.has_value()) {
      return {false, "5D IK failed on Cartesian line: " + reason};
    }
    moveit::core::RobotState waypoint(result->getLastWayPoint());
    for (const auto& [name, value] : *joints) {
      if (std::abs(value - previous.at(name)) > options_.max_joint_jump_rad) {
        return {false, "Joint jump on Cartesian line at " + name};
      }
      waypoint.setVariablePosition(name, value);
    }
    waypoint.update();
    const auto& actual = waypoint.getGlobalLinkTransform(tip_link_);
    if ((actual.translation() - position).norm() >
            options_.position_tolerance_m ||
        axisError(waypoint, tip_link_, panel_normal_) >
            options_.direction_tolerance_rad ||
        !waypoint.satisfiesBounds(group) ||
        corrected_scene->isStateColliding(waypoint, group->getName())) {
      return {false, "Cartesian sample failed pose, bounds, or collision check"};
    }
    result->addSuffixWayPoint(waypoint, 0.0);
    previous = *joints;
  }

  const auto timing = properties().get<trajectory_processing::TimeParameterizationPtr>(
      "time_parameterization");
  if (!timing || !timing->computeTimeStamps(
                     *result,
                     properties().get<double>("max_velocity_scaling_factor"),
                     properties().get<double>("max_acceleration_scaling_factor"))) {
    return {false, "Failed to time-parameterize Cartesian line"};
  }
  return {true, ""};
}

CartesianLinePlanner::Result CartesianLinePlanner::plan(
    const planning_scene::PlanningSceneConstPtr&,
    const moveit::core::LinkModel&, const Eigen::Isometry3d&,
    const Eigen::Isometry3d&, const moveit::core::JointModelGroup*,
    double, robot_trajectory::RobotTrajectoryPtr&,
    const moveit_msgs::msg::Constraints&) {
  return {false, "Use a 5D joint goal for Cartesian line planning"};
}

}  // namespace piper_elevator_manipulation
