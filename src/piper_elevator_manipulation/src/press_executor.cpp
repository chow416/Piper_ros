#include "piper_elevator_manipulation/press_executor.hpp"

#include "piper_elevator_manipulation/contact_monitor.hpp"
#include "piper_elevator_manipulation/press_constants.hpp"
#include "piper_elevator_manipulation/press_solution.hpp"
#include "piper_elevator_manipulation/press_travel_monitor.hpp"
#include "piper_elevator_manipulation/trajectory_executor.hpp"

#include <algorithm>
#include <chrono>
#include <stdexcept>
#include <thread>
#include <vector>

namespace piper_elevator_manipulation {

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
                "axial offset from target %.4f m; against a commanded standoff "
                "of %.1f mm",
                tcp->x(), tcp->y(), tcp->z(),
                (*tcp - parameters.target_position).dot(
                    parameters.press_direction),
                parameters.contact_standoff * 1000.0);
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
  // The press phase starts where the approach stopped and travels exactly
  // `press_depth`, so the cap sits just above that. (It used to be
  // standoff + depth: back then the press ran from the standoff all the way to
  // `target + depth`, which is also why this line's "planned travel" used to
  // disagree with the motion it was describing.)
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

}  // namespace piper_elevator_manipulation
