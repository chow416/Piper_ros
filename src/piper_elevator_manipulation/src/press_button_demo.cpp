#include "piper_elevator_manipulation/five_dof_ik.hpp"
#include "piper_elevator_manipulation/cartesian_line_planner.hpp"
#include "piper_elevator_manipulation/contact_monitor.hpp"
#include "piper_elevator_manipulation/free_motion_route.hpp"
#include "piper_elevator_manipulation/joint_state_cache.hpp"
#include "piper_elevator_manipulation/press_constants.hpp"
#include "piper_elevator_manipulation/press_executor.hpp"
#include "piper_elevator_manipulation/press_parameters.hpp"
#include "piper_elevator_manipulation/press_solution.hpp"
#include "piper_elevator_manipulation/press_task_builder.hpp"
#include "piper_elevator_manipulation/press_travel_monitor.hpp"
#include "piper_elevator_manipulation/target_source.hpp"
#include "piper_elevator_manipulation/trajectory_executor.hpp"

#include <Eigen/Core>
#include <yaml-cpp/yaml.h>
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
#include <cstdio>
#include <fstream>
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
using piper_elevator_manipulation::EditablePressParams;
using piper_elevator_manipulation::FiveDofIkSolver;
using piper_elevator_manipulation::JointStateCache;
using piper_elevator_manipulation::PlannedMotion;
using piper_elevator_manipulation::PlanningTask;
using piper_elevator_manipulation::PressTaskParameters;
using piper_elevator_manipulation::collectMotions;
using piper_elevator_manipulation::createRecoveryTask;
using piper_elevator_manipulation::createTask;
using piper_elevator_manipulation::executePressSequence;
using piper_elevator_manipulation::freeMotionScaling;
using piper_elevator_manipulation::getOrDeclareParameter;
using piper_elevator_manipulation::kArmGroup;
using piper_elevator_manipulation::kArmJoints;
using piper_elevator_manipulation::kEndEffectorLink;
using piper_elevator_manipulation::kPlanningFrame;
using piper_elevator_manipulation::lastCommandedPositions;
using piper_elevator_manipulation::logJointTracking;
using piper_elevator_manipulation::motionTimeout;
using piper_elevator_manipulation::plannedSeconds;
using piper_elevator_manipulation::pressTimeout;
using piper_elevator_manipulation::readPressParamsFile;
using piper_elevator_manipulation::solutionUsesStage;
using piper_elevator_manipulation::speedScaling;
using piper_elevator_manipulation::validatePressParameters;
using piper_elevator_manipulation::vector3Parameter;
using piper_elevator_manipulation::waitForSettledJoints;

namespace {

struct RunSettings {
  bool execute{false};
  std::string execution_mode{"simulation"};
  bool hardware_enable{false};
  bool simulated_contact{false};
  bool use_sim_time{false};
  double travel_margin{0.003};
};

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
  // 5 mm by default. Enough that the approach visibly stops clear of the target
  // on hardware, which is also what makes the gap measurable.
  const double contact_standoff =
      getOrDeclareParameter(node, "contact_standoff_m", 0.005);
  const double press_depth = getOrDeclareParameter(node, "press_depth", 0.004);

  // Optional operator-editable file. Empty means "use the ROS parameters", which
  // is what every launch except press_button_hardware keeps doing; that one
  // points it at config/press_params.yaml, where the numbers sit next to the
  // ranges they have to satisfy.
  const std::string press_params_file =
      getOrDeclareParameter(node, "press_params_file", std::string(""));

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
  std::string reported_params;
  bool overlap_warned = false;
  while (rclcpp::ok() && (max_presses == 0 || press_count < max_presses)) {
    // Re-read the editable file every press so an edit lands on the next one
    // without restarting the service. The ROS parameters stay as the fallback,
    // so a missing or broken file behaves exactly as it did before the file
    // existed.
    double approach = approach_distance;
    double standoff = contact_standoff;
    double depth = press_depth;
    std::string params_source = "ROS parameters";
    if (!press_params_file.empty()) {
      EditablePressParams edited;
      edited.approach_distance = approach_distance;
      edited.contact_standoff = contact_standoff;
      edited.press_depth = press_depth;
      if (readPressParamsFile(node, press_params_file, &edited)) {
        approach = edited.approach_distance;
        standoff = edited.contact_standoff;
        depth = edited.press_depth;
        params_source = press_params_file;
      }
    }
    // Say what will be used, but only when it changes. An operator editing the
    // file wants to see the new values land; a line per press would just be
    // noise, and this log is read while watching the arm.
    char summary[256];
    std::snprintf(summary, sizeof(summary),
                  "approach %.1f mm, contact standoff %.1f mm, press depth "
                  "%.1f mm",
                  approach * 1000.0, standoff * 1000.0, depth * 1000.0);
    if (summary != reported_params) {
      reported_params = summary;
      RCLCPP_INFO(logger, "Press parameters (from %s): %s",
                  params_source.c_str(), summary);
    }
    // The file wins over the launch arguments, because "one knob" is the whole
    // point of it. But winning silently would mean `contact_standoff_m:=0.02` on
    // the launch line doing nothing with no explanation, which is the worse
    // failure of the two. So say it, once.
    //
    // This is only reachable by launching the node directly now: the
    // `--standoff` / `--depth` flags that used to be the way in are refused
    // outright by tools/press_board.sh, precisely because of the failure this
    // warning describes.
    if (!overlap_warned && params_source != "ROS parameters" &&
        (std::abs(approach - approach_distance) > 1e-9 ||
         std::abs(standoff - contact_standoff) > 1e-9 ||
         std::abs(depth - press_depth) > 1e-9)) {
      overlap_warned = true;
      RCLCPP_WARN(logger,
                  "%s overrides the launch arguments for this press. Move the "
                  "file away, or pass press_params_file:='', if you meant to "
                  "drive these from the command line.",
                  press_params_file.c_str());
    }

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
      parameters.approach_distance = approach;
      parameters.contact_standoff = standoff;
      parameters.press_depth = depth;
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
              approach, standoff, depth};
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
