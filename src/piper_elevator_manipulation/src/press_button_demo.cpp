#include "piper_elevator_manipulation/five_dof_ik.hpp"

#include <Eigen/Core>
#include <moveit/task_constructor/solvers/joint_interpolation.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>
#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/stages/modify_planning_scene.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/task_constructor/task.h>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <map>
#include <memory>
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

std::optional<std::map<std::string, double>> captureStartState(
    const rclcpp::Node::SharedPtr& node, std::chrono::seconds timeout) {
  std::optional<sensor_msgs::msg::JointState> latest;
  auto subscription = node->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", rclcpp::SensorDataQoS(),
      [&latest](sensor_msgs::msg::JointState::ConstSharedPtr message) {
        latest = *message;
      });

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (rclcpp::ok() && !latest.has_value() &&
         std::chrono::steady_clock::now() < deadline) {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  executor.remove_node(node);
  subscription.reset();

  if (!latest.has_value()) {
    return std::nullopt;
  }

  std::map<std::string, double> all_positions;
  for (std::size_t i = 0;
       i < latest->name.size() && i < latest->position.size(); ++i) {
    all_positions[latest->name[i]] = latest->position[i];
  }

  std::map<std::string, double> arm_positions;
  for (const char* joint_name : kArmJoints) {
    const auto found = all_positions.find(joint_name);
    if (found == all_positions.end()) {
      RCLCPP_ERROR(node->get_logger(),
                   "Joint state does not contain required joint '%s'.",
                   joint_name);
      return std::nullopt;
    }
    arm_positions[joint_name] = found->second;
  }
  return arm_positions;
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

std::unique_ptr<mtc::Task> createTask(
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

  FiveDofIkSolver ik_solver;
  const auto pre_press_joints = solveWaypoint(
      ik_solver, task->getRobotModel(), bounded_start_positions, pre_press_position,
      direction, "pre-press waypoint");
  const auto contact_joints = solveWaypoint(
      ik_solver, task->getRobotModel(), pre_press_joints,
      parameters.target_position, direction, "contact waypoint");
  const auto pressed_joints = solveWaypoint(
      ik_solver, task->getRobotModel(), contact_joints, pressed_position,
      direction, "pressed waypoint");

  auto sampling_planner =
      std::make_shared<solvers::PipelinePlanner>(node, "ompl");
  sampling_planner->setMaxVelocityScalingFactor(0.10);
  sampling_planner->setMaxAccelerationScalingFactor(0.10);

  auto local_planner = std::make_shared<solvers::JointInterpolationPlanner>();
  local_planner->setProperty("max_step", 0.025);
  local_planner->setMaxVelocityScalingFactor(0.05);
  local_planner->setMaxAccelerationScalingFactor(0.05);

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

  addJointMove("approach and align press axis", sampling_planner,
               pre_press_joints);

  auto allow_contact = std::make_unique<stages::ModifyPlanningScene>(
      "allow press tool contact with target surface");
  allow_contact->allowCollisions(
      "<octomap>", std::vector<std::string>{"press_tool_link", "press_tcp"},
      true);
  task->add(std::move(allow_contact));

  addJointMove("advance to button contact", local_planner, contact_joints);
  addJointMove("press button", local_planner, pressed_joints);
  addJointMove("retreat to pre-press waypoint", local_planner,
               pre_press_joints);

  auto forbid_contact = std::make_unique<stages::ModifyPlanningScene>(
      "restore press tool collision checking");
  forbid_contact->allowCollisions(
      "<octomap>", std::vector<std::string>{"press_tool_link", "press_tcp"},
      false);
  task->add(std::move(forbid_contact));

  addJointMove("return to captured start state", sampling_planner,
               bounded_start_positions);
  return task;
}
}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  auto node = rclcpp::Node::make_shared("piper_press_button_demo", options);

  PressTaskParameters parameters{
      Eigen::Vector3d(
          getOrDeclareParameter(node, "button_x", 0.300),
          getOrDeclareParameter(node, "button_y", 0.000),
          getOrDeclareParameter(node, "button_z", 0.240)),
      Eigen::Vector3d(
          getOrDeclareParameter(node, "press_direction_x", 1.0),
          getOrDeclareParameter(node, "press_direction_y", 0.0),
          getOrDeclareParameter(node, "press_direction_z", 0.0)),
      getOrDeclareParameter(node, "approach_distance", 0.030),
      getOrDeclareParameter(node, "press_depth", 0.004)};
  const bool execute = getOrDeclareParameter(node, "execute", false);

  if (!parameters.target_position.allFinite() ||
      !parameters.press_direction.allFinite() ||
      parameters.press_direction.norm() < 1.0e-6) {
    RCLCPP_ERROR(node->get_logger(),
                 "Target position and non-zero press direction must be finite.");
    rclcpp::shutdown();
    return 2;
  }
  if (parameters.approach_distance < 0.01 ||
      parameters.approach_distance > 0.15) {
    RCLCPP_ERROR(node->get_logger(),
                 "approach_distance must be in [0.01, 0.15] m");
    rclcpp::shutdown();
    return 2;
  }
  if (parameters.press_depth <= 0.0 || parameters.press_depth > 0.015) {
    RCLCPP_ERROR(node->get_logger(),
                 "press_depth must be in (0, 0.015] m");
    rclcpp::shutdown();
    return 2;
  }
  parameters.press_direction.normalize();

  RCLCPP_INFO(node->get_logger(),
              "Waiting for the five-joint start state before planning...");
  const auto start_positions =
      captureStartState(node, std::chrono::seconds(5));
  if (!start_positions.has_value()) {
    RCLCPP_ERROR(node->get_logger(),
                 "Could not capture joint1..joint5 from /joint_states.");
    rclcpp::shutdown();
    return 1;
  }

  std::thread spin_thread([node] { rclcpp::spin(node); });
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
    auto task = createTask(node, parameters, *start_positions);
    const auto result = task->plan(1);
    if (result != moveit::core::MoveItErrorCode::SUCCESS ||
        task->solutions().empty()) {
      RCLCPP_ERROR(node->get_logger(),
                   "Planning failed; inspect the failed MTC stage in RViz.");
      exit_code = 1;
    } else {
      task->introspection().publishSolution(*task->solutions().front());
      RCLCPP_INFO(node->get_logger(),
                  "Planning succeeded: approach, contact, press, retreat, return.");

      if (execute) {
        const auto execution_result =
            task->execute(*task->solutions().front());
        if (execution_result != moveit::core::MoveItErrorCode::SUCCESS) {
          RCLCPP_ERROR(node->get_logger(), "Trajectory execution failed.");
          exit_code = 1;
        } else {
          RCLCPP_INFO(node->get_logger(),
                      "Press sequence completed and arm returned to its start state.");
        }
      } else {
        RCLCPP_INFO(node->get_logger(),
                    "Dry run only. Set execute:=true after reviewing the plan.");
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

  rclcpp::shutdown();
  spin_thread.join();
  return exit_code;
}
