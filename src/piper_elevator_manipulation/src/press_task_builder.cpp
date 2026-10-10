#include "piper_elevator_manipulation/press_task_builder.hpp"

#include "piper_elevator_manipulation/cartesian_line_planner.hpp"
#include "piper_elevator_manipulation/five_dof_ik.hpp"
#include "piper_elevator_manipulation/free_motion_route.hpp"
#include "piper_elevator_manipulation/press_constants.hpp"

#include <moveit/task_constructor/solvers/pipeline_planner.h>
#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/stages/move_to.h>

#include <algorithm>
#include <stdexcept>
#include <utility>
#include <vector>

namespace piper_elevator_manipulation {

namespace stages = moveit::task_constructor::stages;
namespace solvers = moveit::task_constructor::solvers;

namespace {

std::map<std::string, double> solveWaypoint(
    const FiveDofIkSolver& solver,
    const moveit::core::RobotModelConstPtr& robot_model,
    const std::map<std::string, double>& seed,
    const Eigen::Vector3d& position,
    const Eigen::Vector3d& direction,
    const std::string& waypoint_name) {
  std::string reason;
  auto solution = solver.solve(robot_model, kArmGroup, kEndEffectorLink,
                               seed, position, direction, &reason);
  if (!solution.has_value()) {
    throw std::runtime_error("5D IK failed for " + waypoint_name + ": " + reason);
  }
  return *solution;
}

}  // namespace

PlanningTask createTask(
    const rclcpp::Node::SharedPtr& node,
    const PressTaskParameters& parameters,
    const std::map<std::string, double>& start_positions) {
  auto task = std::make_unique<mtc::Task>();
  task->stages()->setName("five-axis elevator button press");
  task->setTimeout(20.0);
  task->loadRobotModel(node);

  std::map<std::string, double> bounded_start_positions = start_positions;
  for (auto& [joint_name, position] : bounded_start_positions) {
    const auto& bounds = task->getRobotModel()->getVariableBounds(joint_name);
    position = std::clamp(position, bounds.min_position_, bounds.max_position_);
  }

  const Eigen::Vector3d direction = parameters.press_direction.normalized();
  const Eigen::Vector3d pre_press_position =
      parameters.target_position - parameters.approach_distance * direction;
  // The approach stops `contact_standoff` short of the aim point rather than on
  // it. See the note on the field: the standoff is what keeps a calibration
  // error from being spent on the target instead of on a gap.
  const Eigen::Vector3d contact_position =
      parameters.target_position - parameters.contact_standoff * direction;
  // The press starts where the approach stopped and pushes `press_depth` further.
  // Measured from the contact waypoint rather than from the aim point on
  // purpose: the standoff is what absorbs a calibration error, so changing it
  // has to move the end of the press too. Measured from the aim point, the
  // standoff would move only the start, and the rod would drive to the same
  // place no matter how large the standoff got -- which is exactly the trap
  // this avoids.
  const Eigen::Vector3d pressed_position =
      contact_position + parameters.press_depth * direction;

  // The free-motion route is loaded before solving IK because its final waypoint
  // is the pose the arm actually holds when the approach begins.
  std::string route_reason;
  const auto route = piper_elevator_manipulation::loadFreeMotionRoute(
      node, task->getRobotModel(), kArmGroup, &route_reason);
  if (route.has_value()) {
    RCLCPP_INFO(node->get_logger(),
                "Free motion route '%s' has %zu prerecorded waypoint(s).",
                route->name.c_str(), route->waypoints.size());
  } else {
    RCLCPP_WARN(node->get_logger(),
                "No usable prerecorded route (%s); using sampling planner.",
                route_reason.c_str());
  }

  // A five-axis arm asked for a position plus a press-axis direction has a whole
  // family of solutions, and the seed decides which branch comes back. Seeding
  // from the captured start state answers the wrong question: by the time the
  // press begins the arm has already driven the prerecorded route, so the branch
  // should be the one nearest the route's end pose. This also matters on
  // hardware, where a parked arm can hold joints pinned against their limits and
  // the start-state seed then lands on a degenerate configuration.
  //
  // Seeds are tried in order and the first complete waypoint chain wins, so a
  // route that cannot be solved for degrades to the start state rather than
  // failing the whole task.
  std::vector<std::pair<std::string, std::map<std::string, double>>> ik_seeds;
  if (route.has_value() && !route->waypoints.empty()) {
    ik_seeds.emplace_back("free-motion route end '" + route->name + "'",
                          route->waypoints.back().positions);
  }
  ik_seeds.emplace_back("captured start state", bounded_start_positions);

  FiveDofIkSolver ik_solver;
  std::map<std::string, double> pre_press_joints;
  std::map<std::string, double> contact_joints;
  std::map<std::string, double> pressed_joints;
  std::string used_seed;
  std::string last_failure;
  for (const auto& [seed_name, seed_positions] : ik_seeds) {
    try {
      pre_press_joints = solveWaypoint(
          ik_solver, task->getRobotModel(), seed_positions, pre_press_position,
          direction, "pre-press waypoint");
      contact_joints = solveWaypoint(
          ik_solver, task->getRobotModel(), pre_press_joints,
          contact_position, direction, "contact waypoint");
      pressed_joints = solveWaypoint(
          ik_solver, task->getRobotModel(), contact_joints, pressed_position,
          direction, "pressed waypoint");
    } catch (const std::exception& failure) {
      last_failure = seed_name + ": " + failure.what();
      RCLCPP_WARN(node->get_logger(),
                  "5D IK failed from seed '%s' (%s); trying the next seed.",
                  seed_name.c_str(), failure.what());
      continue;
    }
    used_seed = seed_name;
    break;
  }
  if (used_seed.empty()) {
    throw std::runtime_error(
        "5D IK failed for every seed. Last failure - " + last_failure);
  }
  RCLCPP_INFO(node->get_logger(), "5D IK solved from seed '%s'.",
              used_seed.c_str());

  auto sampling_planner =
      std::make_shared<solvers::PipelinePlanner>(node, "ompl");
  sampling_planner->setMaxVelocityScalingFactor(
      freeMotionScaling(node, "free_motion.velocity_scaling", 0.10));
  sampling_planner->setMaxAccelerationScalingFactor(
      freeMotionScaling(node, "free_motion.acceleration_scaling", 0.10));

  piper_elevator_manipulation::CartesianLineOptions line_options;
  line_options.max_translation_step_m = getOrDeclareParameter(
      node, "cartesian.max_step_m", 0.002);
  line_options.max_lateral_error_m = getOrDeclareParameter(
      node, "cartesian.max_lateral_error_m", 0.003);
  line_options.max_joint_jump_rad = getOrDeclareParameter(
      node, "cartesian.max_joint_jump_rad", 0.20);
  auto line_planner =
      std::make_shared<piper_elevator_manipulation::CartesianLinePlanner>(
          kEndEffectorLink, direction, line_options);
  const double approach_scaling = speedScaling(
      node, "cartesian.approach_speed_scaling", 0.05);
  line_planner->setMaxVelocityScalingFactor(approach_scaling);
  line_planner->setMaxAccelerationScalingFactor(approach_scaling);
  auto press_planner =
      std::make_shared<piper_elevator_manipulation::CartesianLinePlanner>(
          kEndEffectorLink, direction, line_options);
  const double press_scaling = speedScaling(
      node, "cartesian.press_speed_scaling", 0.01);
  if (press_scaling > approach_scaling) {
    throw std::invalid_argument(
        "press speed scaling cannot exceed approach speed scaling");
  }
  press_planner->setMaxVelocityScalingFactor(press_scaling);
  press_planner->setMaxAccelerationScalingFactor(press_scaling);

  task->add(std::make_unique<stages::CurrentState>("capture live start state"));

  auto addJointMove = [&task](const std::string& name,
                              const solvers::PlannerInterfacePtr& planner,
                              const std::map<std::string, double>& goal) {
    auto stage = std::make_unique<stages::MoveTo>(name, planner);
    stage->setGroup(kArmGroup);
    stage->setGoal(goal);
    mtc::TrajectoryExecutionInfo execution_info;
    execution_info.controller_names = {"arm_controller"};
    stage->setTrajectoryExecutionInfo(std::move(execution_info));
    task->add(std::move(stage));
  };

  auto free_motion = piper_elevator_manipulation::makeFreeMotionBranches(
      route, pre_press_joints, sampling_planner, kArmGroup, "arm_controller");
  const mtc::SerialContainer* prerecorded_free_motion = free_motion.prerecorded;
  task->add(std::move(free_motion.stage));

  addJointMove("advance to button contact", line_planner, contact_joints);
  addJointMove("press button", press_planner, pressed_joints);
  addJointMove("retreat to pre-press waypoint", line_planner,
               pre_press_joints);

  addJointMove("return to captured start state", sampling_planner,
               bounded_start_positions);
  return {std::move(task), prerecorded_free_motion, pre_press_joints,
          bounded_start_positions};
}

std::unique_ptr<mtc::Task> createRecoveryTask(
    const rclcpp::Node::SharedPtr& node,
    const Eigen::Vector3d& panel_normal,
    const std::map<std::string, double>& pre_press_joints,
    const std::map<std::string, double>& return_joints) {
  auto task = std::make_unique<mtc::Task>();
  task->stages()->setName("retreat and return from actual contact state");
  task->setTimeout(20.0);
  task->loadRobotModel(node);
  task->add(std::make_unique<stages::CurrentState>("capture actual press stop"));

  auto line = std::make_shared<piper_elevator_manipulation::CartesianLinePlanner>(
      kEndEffectorLink, panel_normal);
  const double retreat_scaling = speedScaling(
      node, "cartesian.retreat_speed_scaling", 0.04);
  line->setMaxVelocityScalingFactor(retreat_scaling);
  line->setMaxAccelerationScalingFactor(retreat_scaling);
  auto retreat = std::make_unique<stages::MoveTo>(
      "retreat to pre-press waypoint", line);
  retreat->setGroup(kArmGroup);
  retreat->setGoal(pre_press_joints);
  mtc::TrajectoryExecutionInfo execution_info;
  execution_info.controller_names = {"arm_controller"};
  retreat->setTrajectoryExecutionInfo(execution_info);
  task->add(std::move(retreat));

  auto sampling = std::make_shared<solvers::PipelinePlanner>(node, "ompl");
  sampling->setMaxVelocityScalingFactor(
      freeMotionScaling(node, "free_motion.velocity_scaling", 0.10));
  sampling->setMaxAccelerationScalingFactor(
      freeMotionScaling(node, "free_motion.acceleration_scaling", 0.10));
  auto return_home = std::make_unique<stages::MoveTo>(
      "return to captured start state", sampling);
  return_home->setGroup(kArmGroup);
  return_home->setGoal(return_joints);
  return_home->setTrajectoryExecutionInfo(execution_info);
  task->add(std::move(return_home));
  return task;
}

}  // namespace piper_elevator_manipulation
