#include "piper_elevator_manipulation/press_runner.hpp"

#include "piper_elevator_manipulation/press_constants.hpp"
#include "piper_elevator_manipulation/press_executor.hpp"
#include "piper_elevator_manipulation/press_solution.hpp"
#include "piper_elevator_manipulation/press_task_builder.hpp"

#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/task_constructor/stage.h>
#include <moveit/task_constructor/task.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <exception>
#include <optional>
#include <string>
#include <utility>
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

namespace {

//: The outcome of one look-again-then-correct attempt. Every failure is
//: reported through `accepted` plus a reason instead of thrown: the refined
//: observation is an improvement to the aim, so losing it has to cost accuracy,
//: never the press.
struct RefinementOutcome {
  bool accepted{false};
  Eigen::Vector3d position;
  double correction_m{0.0};  // |refined - coarse|
  double angle_deg{0.0};     // between the two normals
  double applied_m{0.0};     // how far the fused target actually moves
  std::string reason;
};

// One round: wait for a refined observation, check it, fuse it.
RefinementOutcome refineOnce(const rclcpp::Node::SharedPtr& node,
                             const PressTaskParameters& parameters,
                             TargetSource& source,
                             const RefinementParams& refinement, int round) {
  RefinementOutcome outcome;
  outcome.position = parameters.target_position;

  RCLCPP_INFO(node->get_logger(),
              "Refine round %d/%d: waiting up to %.1f s on '%s'...",
              round + 1, refinement.rounds, refinement.wait_timeout_s,
              source.topic().c_str());
  const auto observation = source.waitForTarget(std::chrono::milliseconds(
      static_cast<int64_t>(std::llround(refinement.wait_timeout_s * 1000.0))));
  if (!observation.has_value()) {
    outcome.reason = "no refined observation arrived in time";
    return outcome;
  }

  outcome.correction_m =
      (observation->position - parameters.target_position).norm();
  // The normal is watched, never averaged. A single marker offers four coplanar
  // corners and no plane to average over, so its normal can swing a long way --
  // and once normalised, a wrong one looks exactly like a right one.
  const double cosine = std::clamp(
      parameters.press_direction.dot(observation->press_direction), -1.0, 1.0);
  outcome.angle_deg = std::acos(cosine) * 180.0 / M_PI;

  if (outcome.angle_deg > refinement.max_angle_deg) {
    outcome.reason = "the refined normal disagrees by more than the angle gate";
    return outcome;
  }
  if (outcome.correction_m > refinement.max_correction_m) {
    outcome.reason =
        "the refined position is further from the coarse one than the "
        "correction gate allows";
    return outcome;
  }

  // Position only, and deliberately not all of it. The two observations measure
  // the same physical point with different errors, and the errors they share --
  // intrinsics, hand-eye -- average out to nothing. A partial weight is the
  // honest default while "the second one is better" is still unmeasured.
  outcome.position =
      (1.0 - refinement.fusion_weight) * parameters.target_position +
      refinement.fusion_weight * observation->position;
  outcome.applied_m = (outcome.position - parameters.target_position).norm();
  outcome.accepted = true;
  return outcome;
}

}  // namespace

int runSinglePress(const rclcpp::Node::SharedPtr& node,
                   const RunSettings& settings,
                   const PressTaskParameters& parameters,
                   JointStateCache& joint_states,
                   TargetSource* refine_source,
                   const RefinementParams& refinement) {
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
  // The aim this press will actually use. Equal to the requested one until a
  // refined observation moves it, which is why the code below reads this rather
  // than `parameters`.
  PressTaskParameters active = parameters;
  try {
    auto planning_task = createTask(node, active, *start_positions);
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

      // ── the observation and the correction ───────────────────────────────
      // Outside the `settings.execute` gate on purpose: the plumbing is what a
      // dry run exists to check, and a refined rebuild is as inspectable as the
      // coarse one. With `rounds == 0` -- the regression baseline -- this whole
      // block disappears and the run is exactly what it was before.
      if (refine_source != nullptr && refinement.rounds > 0) {
        if (settings.execute) {
          exit_code = runToPrePress(node, planning_task, joint_states);
        }
        if (exit_code == 0) {
          const auto outcome =
              refineOnce(node, active, *refine_source, refinement, 0);
          if (!outcome.accepted) {
            RCLCPP_WARN(node->get_logger(),
                        "Refine round 1/%d: keeping the coarse target (%s).",
                        refinement.rounds, outcome.reason.c_str());
          } else {
            RCLCPP_INFO(node->get_logger(),
                        "Refine round 1/%d: dP=%.1f mm, dtheta=%.2f deg, "
                        "weight %.2f -> aim moves %.1f mm",
                        refinement.rounds, outcome.correction_m * 1000.0,
                        outcome.angle_deg, refinement.fusion_weight,
                        outcome.applied_m * 1000.0);
            PressTaskParameters refined = active;
            refined.target_position = outcome.position;
            // The build below has two ways to fail, and neither of them may end
            // the press. The correction is a gain, not a dependency: if the
            // refined aim cannot be turned into a task, the coarse one is still
            // standing there and is still perfectly pressable. Stopping instead
            // would leave the arm parked at the approach pose for no reason.
            //
            // It fails for two different reasons. `createTask` throws when the
            // corrected waypoints have no IK at all -- which is a normal
            // outcome rather than a fault, because a correction that moves the
            // aim outwards can push it past what the arm can reach. Planning
            // can also come back empty even though every waypoint solved.
            const auto live = joint_states.latestArmPositions();
            std::optional<PlanningTask> rebuilt;
            try {
              // No prerecorded route: the arm already stands at the old
              // approach pose, so replaying a route recorded from elsewhere
              // would drag it back to that route's start and forward again.
              // The live pose is handed over as the IK seed for the same
              // reason: dropping the route also drops its end pose, and the
              // captured start state is then metres away from the target.
              rebuilt.emplace(createTask(node, refined, *start_positions, false,
                                         live ? &*live : nullptr));
              if (rebuilt->task->plan(1) !=
                      moveit::core::MoveItErrorCode::SUCCESS ||
                  rebuilt->task->solutions().empty()) {
                RCLCPP_ERROR(node->get_logger(),
                             "Re-planning against the refined target failed; "
                             "keeping the coarse target.");
                rebuilt.reset();
              }
            } catch (const std::exception& error) {
              RCLCPP_ERROR(node->get_logger(),
                           "The refined target could not be built into a task "
                           "(%s); keeping the coarse target.",
                           error.what());
            }
            if (rebuilt.has_value()) {
              active = refined;
              planning_task = std::move(*rebuilt);
              // Walk to the *new* approach pose before anything else. The
              // rebuilt task's free-motion segment runs from the live state --
              // which is the old approach pose -- to the new one, so it is the
              // correction's own length: a couple of centimetres.
              //
              // Leaving it out is not an optimisation, it is a lurch. The arm
              // would stand at the old start while the approach trajectory
              // began at the new one, and a trajectory controller starts at its
              // first point: a first point a centimetre away from the measured
              // state is a step command, and the arm snaps to it at whatever
              // speed the servo can manage.
              //
              // It also keeps the retreat valid. That stage is built as a line
              // along the panel normal from the actual press stop back to the
              // approach waypoint; with the arm left at the old waypoint the
              // two ends are no longer on a normal line, and the recovery
              // builder refuses with "TCP endpoints do not define a
              // panel-normal line".
              if (settings.execute) {
                const int moved =
                    runToPrePress(node, planning_task, joint_states);
                if (moved != 0) {
                  RCLCPP_ERROR(node->get_logger(),
                               "Refined target accepted, but moving to its "
                               "approach pose failed; aborting this press.");
                  exit_code = moved;
                }
              }
            }
          }
        }
      } else if (settings.execute) {
        exit_code = runToPrePress(node, planning_task, joint_states);
      }

      if (settings.execute && exit_code == 0) {
        if (settings.execution_mode == "simulation") {
          if (!settings.simulated_contact || !settings.use_sim_time) {
            RCLCPP_ERROR(node->get_logger(),
                         "Execution disabled: simulation mode requires "
                         "simulated_contact:=true and use_sim_time:=true.");
            exit_code = 1;
          } else {
            exit_code = executeFromPrePress(node, active, planning_task,
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
              executeFromPrePress(node, active, planning_task,
                                   joint_states, false);
        }
      } else if (!settings.execute) {
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
