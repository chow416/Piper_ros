#include "piper_elevator_manipulation/five_dof_ik.hpp"
#include "piper_elevator_manipulation/cartesian_line_planner.hpp"
#include "piper_elevator_manipulation/contact_monitor.hpp"
#include "piper_elevator_manipulation/free_motion_route.hpp"
#include "piper_elevator_manipulation/press_travel_monitor.hpp"
#include "piper_elevator_manipulation/target_source.hpp"
#include "piper_elevator_manipulation/trajectory_executor.hpp"

#include <Eigen/Core>
#include <moveit/task_constructor/solvers/joint_interpolation.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>
#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/task_constructor/task.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace mtc = moveit::task_constructor;
namespace stages = moveit::task_constructor::stages;
namespace solvers = moveit::task_constructor::solvers;
using piper_elevator_manipulation::FiveDofIkSolver;

namespace {
constexpr char kArmGroup[] = "arm";
constexpr char kEndEffectorLink[] = "press_tcp";
constexpr char kPlanningFrame[] = "world";
constexpr std::array<const char*, 5> kArmJoints{
    "joint1", "joint2", "joint3", "joint4", "joint5"};

struct PressTaskParameters {
  Eigen::Vector3d target_position;
  Eigen::Vector3d press_direction;
  double approach_distance;
  double press_depth;
};

struct PlanningTask {
  std::unique_ptr<mtc::Task> task;
  const mtc::SerialContainer* prerecorded_free_motion{nullptr};
  std::map<std::string, double> pre_press_joints;
  std::map<std::string, double> return_joints;
};

struct PlannedMotion {
  std::string stage_name;
  trajectory_msgs::msg::JointTrajectory trajectory;
};

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

template <typename T>
T getOrDeclareParameter(const rclcpp::Node::SharedPtr& node,
                        const std::string& name, const T& default_value) {
  if (!node->has_parameter(name)) {
    return node->declare_parameter<T>(name, default_value);
  }
  T value{};
  if (!node->get_parameter(name, value)) {
    RCLCPP_WARN(node->get_logger(),
                "Parameter '%s' has an unexpected type; using its default.",
                name.c_str());
    return default_value;
  }
  return value;
}

double speedScaling(const rclcpp::Node::SharedPtr& node,
                    const std::string& key, double default_value) {
  const double value = getOrDeclareParameter(node, key, default_value);
  if (!std::isfinite(value) || value <= 0.0 || value > 0.2) {
    throw std::invalid_argument(key + " must be in (0, 0.2]");
  }
  return value;
}

// Same contract as free_motion_route.cpp's readPositiveScaling, so the OMPL
// fallback and the prerecorded route cannot drift apart. Those two must agree:
// they are alternative branches of the same free-motion segment, and a
// different speed between them would be visible as a lurch whenever the
// recorded route is rejected.
double freeMotionScaling(const rclcpp::Node::SharedPtr& node,
                         const std::string& key, double default_value) {
  const double value = getOrDeclareParameter(node, key, default_value);
  if (!std::isfinite(value) || value <= 0.0 || value > 1.0) {
    throw std::invalid_argument(key + " must be finite and in (0, 1]");
  }
  return value;
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

Eigen::Vector3d vector3Parameter(const rclcpp::Node::SharedPtr& node,
                                 const std::string& name,
                                 const Eigen::Vector3d& default_value) {
  const std::vector<double> default_list{default_value.x(), default_value.y(),
                                         default_value.z()};
  const auto values = getOrDeclareParameter(node, name, default_list);
  if (values.size() != 3) {
    throw std::invalid_argument(name + " must list exactly three numbers");
  }
  return Eigen::Vector3d(values[0], values[1], values[2]);
}

std::chrono::milliseconds pressTimeout(const rclcpp::Node::SharedPtr& node) {
  const double seconds = getOrDeclareParameter(node, "press_timeout_s", 15.0);
  if (!std::isfinite(seconds) || seconds < 1.0 || seconds > 30.0) {
    throw std::invalid_argument("press_timeout_s must be in [1, 30]");
  }
  return std::chrono::milliseconds(
      static_cast<int64_t>(seconds * 1000.0));
}

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

constexpr double kRadiansToDegrees = 57.29577951308232;

// Compares where the arm actually is against the last point of a trajectory it
// was just told to follow.
//
// This matters more than it looks. The trajectory action reports completion at
// the controller's joint tolerance, which is not the same as the arm having
// arrived: the joints can still be closing the gap when the next phase starts.
// A blind sleep cannot tell "still catching up" from "this is as close as it
// ever gets", and on a press only a few millimetres deep that difference is the
// whole task. So wait until the error stops shrinking, then print the numbers.
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

// Waits for the joints to stop closing the gap, then reports. Bounded, so a
// persistent offset (load deflection, for instance, which no amount of waiting
// will fix) does not stall the task.
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
  const Eigen::Vector3d pressed_position =
      parameters.target_position + parameters.press_depth * direction;

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
          parameters.target_position, direction, "contact waypoint");
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

// Executes the validated solution one phase at a time.
//
// `require_contact` selects the stop condition for the press phase:
//   true  - simulation. The press is cancelled on stable virtual contact and
//           completing the full travel is treated as a failure.
//   false - hardware without a contact sensor. The planned trajectory defines
//           the travel, and the independently measured TCP travel is still
//           enforced but as a safety cap above the plan, so a correct press
//           ends because the trajectory completed, never because the cap hit.
int executePressSequence(
    const rclcpp::Node::SharedPtr& node,
    const PressTaskParameters& parameters,
    const PlanningTask& planning_task,
    JointStateCache& joint_states,
    bool require_contact) {
  std::vector<PlannedMotion> motions;
  collectMotions(*planning_task.task->solutions().front(), &motions);
  piper_elevator_manipulation::TrajectoryExecutor executor(node);
  // The contact subscriber is created either way, but on hardware nothing
  // publishes /press/contact and the monitor is simply not passed to the
  // executor, so no contact ever gates the motion.
  piper_elevator_manipulation::ContactMonitor contact(
      node, "/press/contact", 2, std::chrono::milliseconds(250));
  piper_elevator_manipulation::PressTravelMonitor travel(
      node, planning_task.task->getRobotModel(), kEndEffectorLink,
      parameters.press_direction, std::chrono::milliseconds(250));

  bool approach_completed = false;
  const PlannedMotion* press_motion = nullptr;
  const PlannedMotion* approach_motion = nullptr;
  for (const auto& motion : motions) {
    if (motion.stage_name == "press button") {
      press_motion = &motion;
      break;
    }
    if (motion.stage_name == "retreat to pre-press waypoint" ||
        motion.stage_name == "return to captured start state") {
      throw std::runtime_error("MTC solution has an unexpected phase order");
    }
    RCLCPP_INFO(node->get_logger(), "Executing phase: %s (planned %.2f s)",
                motion.stage_name.c_str(), plannedSeconds(motion.trajectory));
    const auto result = executor.execute(
        motion.trajectory, motionTimeout(motion.trajectory));
    if (result.status !=
        piper_elevator_manipulation::ExecutionStatus::completed) {
      RCLCPP_ERROR(node->get_logger(), "CONTROLLER_FAILED at %s: %s",
                   motion.stage_name.c_str(), result.detail.c_str());
      return 1;
    }
    approach_completed |= motion.stage_name == "advance to button contact";
    if (motion.stage_name == "advance to button contact") {
      approach_motion = &motion;
    }
  }
  if (!approach_completed || press_motion == nullptr) {
    throw std::runtime_error("MTC solution lacks approach or press trajectory");
  }
  // Wait until the joints stop closing the gap rather than trusting a fixed
  // sleep, and report what the controller actually achieved. The contact and
  // travel monitors measure from here, so an arm that is still catching up
  // reads as a short press.
  if (approach_motion != nullptr) {
    waitForSettledJoints(node, joint_states, approach_motion->trajectory,
                         "approach end");
  }
  if (const auto tcp = travel.latestTcpPosition()) {
    RCLCPP_INFO(node->get_logger(),
                "Actual TCP at approach end: (%.4f, %.4f, %.4f), "
                "axial offset from target %.4f m",
                tcp->x(), tcp->y(), tcp->z(),
                (*tcp - parameters.target_position).dot(
                    parameters.press_direction));
  }
  if (require_contact && contact.isContactActive()) {
    RCLCPP_ERROR(node->get_logger(),
                 "Contact was already active before the press phase");
    return 1;
  }

  // On hardware the trajectory is the stop condition, so the cap sits above the
  // planned travel and only fires when the arm moves further than commanded.
  const double travel_margin = require_contact
      ? 0.0
      : getOrDeclareParameter(node, "hardware_travel_margin_m", 0.003);
  const double travel_cap = parameters.press_depth + travel_margin;
  RCLCPP_INFO(node->get_logger(),
              "Executing %s press (planned travel %.4f m, safety cap %.4f m)",
              require_contact ? "contact-limited" : "fixed-travel",
              parameters.press_depth, travel_cap);
  const auto press_result = executor.execute(
      press_motion->trajectory,
      std::min(motionTimeout(press_motion->trajectory), pressTimeout(node)),
      require_contact ? &contact : nullptr, &travel, travel_cap);
  const bool contact_detected = press_result.status ==
      piper_elevator_manipulation::ExecutionStatus::contact_detected;
  const bool press_completed = press_result.status ==
      piper_elevator_manipulation::ExecutionStatus::completed;
  if (require_contact && !contact_detected) {
    RCLCPP_ERROR(node->get_logger(), "Press stopped without contact: %s",
                 press_result.detail.c_str());
  } else if (!require_contact && !press_completed) {
    RCLCPP_ERROR(node->get_logger(),
                 "Fixed-travel press did not complete cleanly: %s",
                 press_result.detail.c_str());
  } else {
    RCLCPP_INFO(node->get_logger(), "%s", press_result.detail.c_str());
  }
  if (press_result.status ==
      piper_elevator_manipulation::ExecutionStatus::controller_failed) {
    RCLCPP_ERROR(node->get_logger(),
                 "Controller stop unconfirmed; refusing automatic recovery");
    return 1;
  }
  // Only meaningful for the fixed-travel press: a contact-cancelled goal stops
  // somewhere in the middle, so its planned endpoint is not the target.
  if (!require_contact) {
    const auto measured = joint_states.latestArmPositions();
    if (measured.has_value()) {
      logJointTracking(node->get_logger(), "press end",
                       lastCommandedPositions(press_motion->trajectory),
                       *measured);
    }
  }

  // The original MTC retreat begins at maximum press travel. After contact
  // cancellation the actual arm is elsewhere, so replan from live state.
  std::this_thread::sleep_for(std::chrono::milliseconds(250));
  auto recovery = createRecoveryTask(
      node, parameters.press_direction,
      planning_task.pre_press_joints, planning_task.return_joints);
  if (recovery->plan(1) != moveit::core::MoveItErrorCode::SUCCESS ||
      recovery->solutions().empty()) {
    RCLCPP_ERROR(node->get_logger(),
                 "RECOVERY_FAILED: no validated retreat/return from actual state");
    return 1;
  }
  std::vector<PlannedMotion> recovery_motions;
  collectMotions(*recovery->solutions().front(), &recovery_motions);
  if (recovery_motions.size() != 2 ||
      recovery_motions.front().stage_name != "retreat to pre-press waypoint" ||
      recovery_motions.back().stage_name != "return to captured start state") {
    RCLCPP_ERROR(node->get_logger(),
                 "RECOVERY_FAILED: unexpected recovery trajectory layout");
    return 1;
  }
  for (const auto& motion : recovery_motions) {
    RCLCPP_INFO(node->get_logger(), "Executing recovery: %s",
                motion.stage_name.c_str());
    const auto result = executor.execute(
        motion.trajectory, motionTimeout(motion.trajectory));
    if (result.status !=
        piper_elevator_manipulation::ExecutionStatus::completed) {
      RCLCPP_ERROR(node->get_logger(), "RECOVERY_FAILED: %s",
                   result.detail.c_str());
      return 1;
    }
  }
  if (require_contact) {
    if (!contact_detected) {
      RCLCPP_ERROR(node->get_logger(),
                   "CONTACT_NOT_DETECTED; arm retreated and returned to start");
      return 1;
    }
    RCLCPP_INFO(node->get_logger(),
                "SUCCESS: contact stopped press; arm retreated and returned");
    return 0;
  }
  if (!press_completed) {
    RCLCPP_ERROR(node->get_logger(),
                 "FIXED_TRAVEL_PRESS_FAILED; arm retreated and returned to start");
    return 1;
  }
  RCLCPP_INFO(node->get_logger(),
              "SUCCESS: fixed-travel press completed; arm retreated and returned");
  return 0;
}
}  // namespace

namespace {

// Settings that describe how to run a press, as opposed to what to press.
struct RunSettings {
  bool execute{false};
  std::string execution_mode{"simulation"};
  bool hardware_enable{false};
  bool simulated_contact{false};
  bool use_sim_time{false};
  double travel_margin{0.003};
};

bool validatePressParameters(const rclcpp::Node::SharedPtr& node,
                             PressTaskParameters& parameters) {
  if (!parameters.target_position.allFinite() ||
      !parameters.press_direction.allFinite() ||
      parameters.press_direction.norm() < 1.0e-6) {
    RCLCPP_ERROR(node->get_logger(),
                 "Target position and non-zero press direction must be finite.");
    return false;
  }
  if (!std::isfinite(parameters.approach_distance) ||
      parameters.approach_distance < 0.01 ||
      parameters.approach_distance > 0.15) {
    RCLCPP_ERROR(node->get_logger(),
                 "approach_distance must be in [0.01, 0.15] m");
    return false;
  }
  if (!std::isfinite(parameters.press_depth) ||
      parameters.press_depth <= 0.0 || parameters.press_depth > 0.015) {
    RCLCPP_ERROR(node->get_logger(), "press_depth must be in (0, 0.015] m");
    return false;
  }
  parameters.press_direction.normalize();
  return true;
}

// Waits for one accepted target, reporting rejected messages as they pile up so
// that a silent wait can be told apart from a publisher that is being refused.
std::optional<piper_elevator_manipulation::PressTarget> waitForTarget(
    const rclcpp::Node::SharedPtr& node,
    piper_elevator_manipulation::TargetSource& source, double timeout_seconds) {
  const auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                            std::chrono::duration<double>(timeout_seconds));
  std::size_t reported_rejections = 0;
  if (timeout_seconds > 0.0) {
    RCLCPP_INFO(node->get_logger(),
                "Waiting for a press target on '%s' (giving up after %.1f s)...",
                source.topic().c_str(), timeout_seconds);
  } else {
    RCLCPP_INFO(node->get_logger(),
                "Waiting for a press target on '%s' (waiting indefinitely)...",
                source.topic().c_str());
  }
  while (rclcpp::ok()) {
    const auto target = source.waitForTarget(std::chrono::milliseconds(1000));
    if (target.has_value()) {
      return target;
    }
    const std::size_t rejected = source.rejectedCount();
    if (rejected != reported_rejections) {
      RCLCPP_WARN(node->get_logger(), "%zu target(s) rejected so far; last reason: %s",
                  rejected, source.lastRejectionReason().c_str());
      reported_rejections = rejected;
    }
    if (timeout_seconds > 0.0 && std::chrono::steady_clock::now() >= deadline) {
      RCLCPP_ERROR(node->get_logger(),
                   "No valid target on '%s' within %.1f s. %zu message(s) were "
                   "rejected; last reason: %s",
                   source.topic().c_str(), timeout_seconds, rejected,
                   source.lastRejectionReason().empty()
                       ? "nothing was received"
                       : source.lastRejectionReason().c_str());
      return std::nullopt;
    }
  }
  return std::nullopt;
}

// Plans and, when the gates allow it, runs exactly one press. Returns 0 when
// the press completed and the arm returned to where it started.
int runSinglePress(const rclcpp::Node::SharedPtr& node,
                   const RunSettings& settings,
                   const PressTaskParameters& parameters,
                   JointStateCache& joint_states) {
  RCLCPP_INFO(node->get_logger(),
              "Waiting for the five-joint start state before planning...");
  const auto start_positions =
      joint_states.waitForArmPositions(std::chrono::seconds(5));
  if (!start_positions.has_value()) {
    RCLCPP_ERROR(node->get_logger(),
                 "Could not capture joint1..joint5 from /joint_states.");
    return 1;
  }

  RCLCPP_INFO(node->get_logger(),
              "%s target=(%.3f, %.3f, %.3f), direction=(%.3f, %.3f, %.3f), "
              "approach=%.3f m, press=%.3f m",
              kPlanningFrame, parameters.target_position.x(),
              parameters.target_position.y(), parameters.target_position.z(),
              parameters.press_direction.x(), parameters.press_direction.y(),
              parameters.press_direction.z(), parameters.approach_distance,
              parameters.press_depth);

  int exit_code = 0;
  try {
    auto planning_task = createTask(node, parameters, *start_positions);
    auto& task = planning_task.task;
    const auto result = task->plan(1);
    if (result != moveit::core::MoveItErrorCode::SUCCESS ||
        task->solutions().empty()) {
      RCLCPP_ERROR(node->get_logger(),
                   "Planning failed; inspect the failed MTC stage in RViz.");
      exit_code = 1;
    } else {
      const bool used_prerecorded_route =
          planning_task.prerecorded_free_motion != nullptr &&
          solutionUsesStage(*task->solutions().front(),
                            planning_task.prerecorded_free_motion);
      if (!used_prerecorded_route &&
          planning_task.prerecorded_free_motion != nullptr) {
        planning_task.prerecorded_free_motion->traverseRecursively(
            [&node](const mtc::Stage& stage, unsigned int) {
              if (stage.numFailures() > 0) {
                RCLCPP_WARN(node->get_logger(),
                            "Recorded route stage '%s' failed %zu time(s): %s",
                            stage.name().c_str(), stage.numFailures(),
                            stage.failures().front()->comment().c_str());
              }
              return true;
            });
      }
      RCLCPP_INFO(node->get_logger(), "free_motion.path_source=%s",
                  used_prerecorded_route ? "prerecorded" : "sampling");
      task->introspection().publishSolution(*task->solutions().front());
      RCLCPP_INFO(node->get_logger(),
                  "Planning succeeded: approach, contact, press, retreat, return.");
      // Report the planned duration of every phase, and its total. This is the
      // number the speed scaling factors control, and unlike wall-clock time it
      // can be checked from a dry run without moving the arm.
      std::vector<PlannedMotion> planned_phases;
      collectMotions(*task->solutions().front(), &planned_phases);
      double planned_total = 0.0;
      for (const auto& phase : planned_phases) {
        const double seconds = plannedSeconds(phase.trajectory);
        planned_total += seconds;
        RCLCPP_INFO(node->get_logger(), "  planned %-34s %5.2f s",
                    phase.stage_name.c_str(), seconds);
      }
      RCLCPP_INFO(node->get_logger(),
                  "  planned total (motion only)           %5.2f s",
                  planned_total);

      if (settings.execute) {
        if (settings.execution_mode == "simulation") {
          if (!settings.simulated_contact || !settings.use_sim_time) {
            RCLCPP_ERROR(node->get_logger(),
                         "Execution disabled: simulation mode requires "
                         "simulated_contact:=true and use_sim_time:=true.");
            exit_code = 1;
          } else {
            exit_code = executePressSequence(node, parameters, planning_task,
                                             joint_states, true);
          }
        } else if (settings.use_sim_time) {
          RCLCPP_ERROR(node->get_logger(),
                       "Execution disabled: hardware_fixed_travel requires "
                       "use_sim_time:=false.");
          exit_code = 1;
        } else if (!settings.hardware_enable) {
          RCLCPP_ERROR(node->get_logger(),
                       "Execution disabled: pass hardware_enable:=true to command "
                       "the real arm. Keep the emergency stop within reach and "
                       "confirm the approach path is clear.");
          exit_code = 1;
        } else {
          RCLCPP_WARN(node->get_logger(),
                      "HARDWARE EXECUTION: no contact sensor. The press travel comes "
                      "from the planned trajectory; the measured TCP travel is only a "
                      "safety cap %.4f m above it.",
                      parameters.press_depth + settings.travel_margin);
          exit_code =
              executePressSequence(node, parameters, planning_task,
                                   joint_states, false);
        }
      } else {
        RCLCPP_INFO(node->get_logger(),
                    "Dry run only. In Gazebo, set execute:=true "
                    "simulated_contact:=true after reviewing the plan.");
      }
    }
  } catch (const mtc::InitStageException& exception) {
    RCLCPP_ERROR_STREAM(node->get_logger(),
                        "MTC initialization failed:\n" << exception);
    exit_code = 1;
  } catch (const std::exception& exception) {
    RCLCPP_ERROR(node->get_logger(), "%s", exception.what());
    exit_code = 1;
  }
  return exit_code;
}

}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  auto node = rclcpp::Node::make_shared("piper_press_button_demo", options);
  const auto logger = node->get_logger();

  RunSettings settings;
  settings.execute = getOrDeclareParameter(node, "execute", false);
  settings.execution_mode =
      getOrDeclareParameter(node, "execution_mode", std::string("simulation"));
  // A second, explicit gate for real hardware. Reading the target from launch
  // arguments and commanding a real arm must never be one careless flag apart.
  settings.hardware_enable =
      getOrDeclareParameter(node, "hardware_enable", false);
  settings.simulated_contact =
      getOrDeclareParameter(node, "simulated_contact", false);
  settings.use_sim_time = getOrDeclareParameter(node, "use_sim_time", false);
  settings.travel_margin =
      getOrDeclareParameter(node, "hardware_travel_margin_m", 0.003);
  if (settings.execution_mode != "simulation" &&
      settings.execution_mode != "hardware_fixed_travel") {
    RCLCPP_ERROR(logger,
                 "execution_mode must be 'simulation' or 'hardware_fixed_travel'");
    rclcpp::shutdown();
    return 2;
  }

  // Where the target comes from. 'parameters' is a fixed point read from the
  // launch arguments; 'topic' is the perception interface, where each accepted
  // message asks for one press.
  const std::string target_source_name =
      getOrDeclareParameter(node, "target_source", std::string("parameters"));
  const bool targets_from_topic = target_source_name == "topic";
  if (!targets_from_topic && target_source_name != "parameters") {
    RCLCPP_ERROR(logger, "target_source must be 'parameters' or 'topic', not '%s'",
                 target_source_name.c_str());
    rclcpp::shutdown();
    return 2;
  }
  const int64_t max_presses = getOrDeclareParameter<int64_t>(node, "max_presses", 1);
  if (max_presses < 0) {
    RCLCPP_ERROR(logger, "max_presses must be 0 (unlimited) or a positive count");
    rclcpp::shutdown();
    return 2;
  }

  const double approach_distance =
      getOrDeclareParameter(node, "approach_distance", 0.030);
  const double press_depth = getOrDeclareParameter(node, "press_depth", 0.004);

  std::unique_ptr<piper_elevator_manipulation::TargetSource> target_channel;
  try {
    if (targets_from_topic) {
      piper_elevator_manipulation::TargetSourceOptions source_options;
      source_options.topic = getOrDeclareParameter(
          node, "target_topic", std::string("/press/target_pose"));
      source_options.direction_sign =
          getOrDeclareParameter(node, "press_direction_sign", 1.0);
      source_options.max_age = std::chrono::milliseconds(static_cast<int64_t>(
          1000.0 * getOrDeclareParameter(node, "target_max_age_s", 1.0)));
      source_options.workspace_min = vector3Parameter(
          node, "workspace_min", Eigen::Vector3d(-0.30, -0.50, -0.02));
      source_options.workspace_max = vector3Parameter(
          node, "workspace_max", Eigen::Vector3d(0.65, 0.50, 0.80));
      source_options.accepted_frames = getOrDeclareParameter(
          node, "target_accepted_frames",
          std::vector<std::string>{"world", "base_link"});
      if (source_options.direction_sign != 1.0 &&
          source_options.direction_sign != -1.0) {
        RCLCPP_ERROR(logger, "press_direction_sign must be +1 or -1");
        rclcpp::shutdown();
        return 2;
      }
      target_channel = std::make_unique<piper_elevator_manipulation::TargetSource>(
          node, source_options);
    }
  } catch (const std::exception& exception) {
    RCLCPP_ERROR(logger, "%s", exception.what());
    rclcpp::shutdown();
    return 2;
  }

  JointStateCache joint_states(node);
  std::thread spin_thread([node] { rclcpp::spin(node); });

  int exit_code = 0;
  int64_t press_count = 0;
  while (rclcpp::ok() && (max_presses == 0 || press_count < max_presses)) {
    PressTaskParameters parameters;
    if (targets_from_topic) {
      // Drop anything that arrived while the arm was moving. A target observed
      // behind the arm is not worth reusing, and this keeps each press tied to
      // a fresh observation instead of to whatever the queue happened to hold.
      target_channel->discardPendingTarget();
      const auto target = waitForTarget(
          node, *target_channel,
          getOrDeclareParameter(node, "target_wait_timeout_s", 0.0));
      if (!target.has_value()) {
        exit_code = 1;
        break;
      }
      parameters.target_position = target->position;
      parameters.press_direction = target->press_direction;
      parameters.approach_distance = approach_distance;
      parameters.press_depth = press_depth;
    } else {
      parameters =
          PressTaskParameters{
              Eigen::Vector3d(
                  getOrDeclareParameter(node, "button_x", 0.300),
                  getOrDeclareParameter(node, "button_y", 0.000),
                  getOrDeclareParameter(node, "button_z", 0.240)),
              Eigen::Vector3d(
                  getOrDeclareParameter(node, "press_direction_x", 1.0),
                  getOrDeclareParameter(node, "press_direction_y", 0.0),
                  getOrDeclareParameter(node, "press_direction_z", 0.0)),
              approach_distance, press_depth};
    }

    if (!validatePressParameters(node, parameters)) {
      exit_code = 2;
      break;
    }
    exit_code = runSinglePress(node, settings, parameters, joint_states);
    ++press_count;
    if (targets_from_topic) {
      RCLCPP_INFO(logger, "Press %ld finished with exit code %d.",
                  static_cast<long>(press_count), exit_code);
    }
  }

  if (target_channel) {
    RCLCPP_INFO(logger, "Target source '%s': %zu message(s) rejected in total.",
                target_channel->topic().c_str(), target_channel->rejectedCount());
  }

  rclcpp::shutdown();
  spin_thread.join();
  return exit_code;
}
