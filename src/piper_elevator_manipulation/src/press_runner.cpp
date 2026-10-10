#include "piper_elevator_manipulation/press_runner.hpp"

#include "piper_elevator_manipulation/press_constants.hpp"
#include "piper_elevator_manipulation/press_executor.hpp"
#include "piper_elevator_manipulation/press_solution.hpp"
#include "piper_elevator_manipulation/press_task_builder.hpp"

#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/task_constructor/stage.h>
#include <moveit/task_constructor/task.h>

#include <chrono>
#include <vector>

namespace piper_elevator_manipulation {

namespace mtc = moveit::task_constructor;

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

}  // namespace piper_elevator_manipulation
