#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <moveit/task_constructor/solvers/cartesian_path.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>
#include <moveit/task_constructor/stages/compute_ik.h>
#include <moveit/task_constructor/stages/connect.h>
#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/stages/generate_pose.h>
#include <moveit/task_constructor/stages/move_relative.h>
#include <moveit/task_constructor/stages/modify_planning_scene.h>
#include <moveit/task_constructor/task.h>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <rclcpp/rclcpp.hpp>

#include <cmath>
#include <memory>
#include <thread>

namespace mtc = moveit::task_constructor;
namespace stages = moveit::task_constructor::stages;
namespace solvers = moveit::task_constructor::solvers;

namespace {
constexpr char kArmGroup[] = "arm";
constexpr char kEndEffectorLink[] = "link6";
constexpr char kPlanningFrame[] = "base_link";

geometry_msgs::msg::PoseStamped makePrePressPose(double x, double y, double z) {
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = kPlanningFrame;
  pose.pose.position.x = x;
  pose.pose.position.y = y;
  pose.pose.position.z = z;

  // The demo button panel faces the robot along +X.  This orientation makes
  // link6 point toward the panel.  It is deliberately a parameterized demo
  // target. The planning/control baseline intentionally accepts an explicit
  // world-frame target and has no dependency on the paused perception package.
  // gripper_base +Z is the tool/press direction. Rotate it onto base_link +X.
  const Eigen::Quaterniond orientation(
      Eigen::AngleAxisd(0.5 * M_PI, Eigen::Vector3d::UnitY()));
  pose.pose.orientation.x = orientation.x();
  pose.pose.orientation.y = orientation.y();
  pose.pose.orientation.z = orientation.z();
  pose.pose.orientation.w = orientation.w();
  return pose;
}

template <typename T>
T getOrDeclareParameter(const rclcpp::Node::SharedPtr& node, const std::string& name,
                        const T& default_value) {
  if (!node->has_parameter(name)) {
    return node->declare_parameter<T>(name, default_value);
  }
  T value{};
  if (!node->get_parameter(name, value)) {
    RCLCPP_WARN(node->get_logger(), "Parameter '%s' has an unexpected type; using its default.", name.c_str());
    return default_value;
  }
  return value;
}

std::unique_ptr<mtc::Task> createTask(const rclcpp::Node::SharedPtr& node,
                                      const geometry_msgs::msg::PoseStamped& pre_press_pose,
                                      double press_distance, double tool_length) {
  auto task = std::make_unique<mtc::Task>();
  task->stages()->setName("elevator button press");
  // MTC's default task timeout is effectively infinite.  A bounded timeout is
  // important for a service/demo node: an unreachable target must
  // return control to its caller instead of blocking indefinitely.
  task->setTimeout(15.0);
  task->loadRobotModel(node);

  auto sampling_planner = std::make_shared<solvers::PipelinePlanner>(node, "ompl");
  sampling_planner->setPlannerId("RRTConnectkConfigDefault");

  auto cartesian_planner = std::make_shared<solvers::CartesianPath>();
  cartesian_planner->setStepSize(0.005);
  cartesian_planner->setJumpThreshold(2.0);
  cartesian_planner->setMaxVelocityScalingFactor(0.10);
  cartesian_planner->setMaxAccelerationScalingFactor(0.10);
  Eigen::Isometry3d tool_tip = Eigen::Isometry3d::Identity();
  tool_tip.translation().z() = tool_length;

  auto current_state = std::make_unique<stages::CurrentState>("read current state");
  auto* current_state_ptr = current_state.get();
  task->add(std::move(current_state));

  auto connect = std::make_unique<stages::Connect>(
      "plan to pre-press pose",
      stages::Connect::GroupPlannerVector{{kArmGroup, sampling_planner}});
  connect->setTimeout(5.0);
  connect->properties().configureInitFrom(mtc::Stage::PARENT);
  task->add(std::move(connect));

  auto generate_pose = std::make_unique<stages::GeneratePose>("generate pre-press pose");
  generate_pose->setMonitoredStage(current_state_ptr);
  generate_pose->setPose(pre_press_pose);

  auto compute_ik = std::make_unique<stages::ComputeIK>("compute pre-press IK", std::move(generate_pose));
  compute_ik->setGroup(kArmGroup);
  compute_ik->setIKFrame(tool_tip, kEndEffectorLink);
  compute_ik->setMaxIKSolutions(12);
  compute_ik->setMinSolutionDistance(0.25);
  compute_ik->setTimeout(1.5);
  compute_ik->properties().configureInitFrom(mtc::Stage::INTERFACE, {"target_pose"});
  task->add(std::move(compute_ik));

  auto makeCartesianStage = [&](const std::string& name, double distance) {
    auto stage = std::make_unique<stages::MoveRelative>(name, cartesian_planner);
    stage->setGroup(kArmGroup);
    stage->setIKFrame(tool_tip, kEndEffectorLink);
    stage->setMinMaxDistance(std::abs(distance), std::abs(distance));
    stage->properties().set("marker_ns", name);
    geometry_msgs::msg::Vector3Stamped direction;
    direction.header.frame_id = kPlanningFrame;
    direction.vector.x = distance > 0.0 ? 1.0 : -1.0;
    stage->setDirection(direction);
    return stage;
  };

  // Contact with the selected button is intentional. The target surface is
  // represented by the anonymous MoveIt OctoMap, so allow only the gripper
  // links to touch it during the short press/retreat segment. Global arm
  // collision checking remains active, including while planning to pre-press.
  auto allow_contact = std::make_unique<stages::ModifyPlanningScene>(
      "allow gripper contact with target surface");
  allow_contact->allowCollisions(
      "<octomap>", std::vector<std::string>{ "gripper_base", "link7", "link8" }, true);
  task->add(std::move(allow_contact));

  task->add(makeCartesianStage("press button +X", press_distance));
  task->add(makeCartesianStage("retreat from panel -X", -press_distance));

  auto forbid_contact = std::make_unique<stages::ModifyPlanningScene>(
      "restore gripper collision checking");
  forbid_contact->allowCollisions(
      "<octomap>", std::vector<std::string>{ "gripper_base", "link7", "link8" }, false);
  task->add(std::move(forbid_contact));
  return task;
}
}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  auto node = rclcpp::Node::make_shared("piper_press_button_demo", options);

  const double button_x = getOrDeclareParameter(node, "button_x", 0.300);
  const double button_y = getOrDeclareParameter(node, "button_y", 0.000);
  const double button_z = getOrDeclareParameter(node, "button_z", 0.240);
  const double press_distance = getOrDeclareParameter(node, "press_distance", 0.040);
  const double tool_length = getOrDeclareParameter(node, "tool_length", 0.1358);
  const bool execute = getOrDeclareParameter(node, "execute", false);

  if (press_distance <= 0.02 || press_distance > 0.06) {
    RCLCPP_ERROR(node->get_logger(), "press_distance must be in (0.02, 0.06] m");
    rclcpp::shutdown();
    return 2;
  }
  if (tool_length <= 0.05 || tool_length > 0.25) {
    RCLCPP_ERROR(node->get_logger(), "tool_length must be in (0.05, 0.25] m");
    rclcpp::shutdown();
    return 2;
  }
  std::thread spin_thread([node] { rclcpp::spin(node); });
  const auto pre_press_pose = makePrePressPose(
      button_x - press_distance, button_y, button_z);
  RCLCPP_INFO(node->get_logger(),
              "World-frame target: button=(%.3f, %.3f, %.3f), pre-press x=%.3f, press=%.3f m",
              button_x, button_y, button_z, pre_press_pose.pose.position.x, press_distance);

  auto task = createTask(node, pre_press_pose, press_distance, tool_length);
  try {
    const auto result = task->plan(1);
    if (result != moveit::core::MoveItErrorCode::SUCCESS || task->solutions().empty()) {
      RCLCPP_ERROR(node->get_logger(), "Planning failed; adjust button_x/button_y/button_z.");
      rclcpp::shutdown();
      spin_thread.join();
      return 1;
    }
    task->introspection().publishSolution(*task->solutions().front());
    RCLCPP_INFO(node->get_logger(), "Planning succeeded.");

    if (execute) {
      const auto execution_result = task->execute(*task->solutions().front());
      if (execution_result != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_ERROR(node->get_logger(), "Trajectory execution failed.");
        rclcpp::shutdown();
        spin_thread.join();
        return 1;
      }
      RCLCPP_INFO(node->get_logger(), "Button press demo completed: press and retreat finished.");
    } else {
      RCLCPP_INFO(node->get_logger(), "Dry run only. Set execute:=true to move the Gazebo arm.");
    }
  } catch (const mtc::InitStageException& ex) {
    RCLCPP_ERROR_STREAM(node->get_logger(), "MTC initialization failed:\n" << ex);
    rclcpp::shutdown();
    spin_thread.join();
    return 1;
  }

  rclcpp::shutdown();
  spin_thread.join();
  return 0;
}
