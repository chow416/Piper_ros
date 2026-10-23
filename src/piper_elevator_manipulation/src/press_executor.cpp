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

namespace {

//: Phase names as they appear in the MTC solution; `press_solution.cpp` is what
//: turns the stage tree into that ordered list. Named rather than written out
//: because the split below is defined by two of them, and a typo in one of the
//: two comparisons would silently move the boundary instead of failing.
constexpr char kAdvanceStage[] = "advance to button contact";
constexpr char kPressStage[] = "press button";
constexpr char kRetreatStage[] = "retreat to pre-press waypoint";
constexpr char kReturnStage[] = "return to captured start state";

}  // namespace

int runToPrePress(const rclcpp::Node::SharedPtr& node,
                  const PlanningTask& planning_task,
                  JointStateCache& joint_states) {
  std::vector<PlannedMotion> motions;
  collectMotions(*planning_task.task->solutions().front(), &motions);
  piper_elevator_manipulation::TrajectoryExecutor executor(node);

  const PlannedMotion* last_executed = nullptr;
  for (const auto& motion : motions) {
    // Stops at the approach, not at the press. If the aim is going to be
    // corrected, the correction has to land before the arm commits to the
    // contact line -- and it has to be observed while the camera can still see
    // anything, which is this pose and no later one.
    if (motion.stage_name == kAdvanceStage ||
        motion.stage_name == kPressStage) {
      break;
    }
    if (motion.stage_name == kRetreatStage ||
        motion.stage_name == kReturnStage) {
      throw std::runtime_error("MTC solution has an unexpected phase order");
    }
    RCLCPP_INFO(node->get_logger(), "Executing phase: %s (planned %.2f s)",
                motion.stage_name.c_str(), plannedSeconds(motion.trajectory));
    const auto phase_started = std::chrono::steady_clock::now();
    const auto result = executor.execute(
        motion.trajectory, motionTimeout(motion.trajectory));
    // 实测耗时：调速度参数时唯一能验证"到底快了多少"的数字。
    // planned 是时间参数化算出来的，实际会被控制器跟踪误差 / 固件限速拉开差距，
    // 两者的差就是"参数改了但没变快"的信号。
    const double actual_s = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - phase_started).count();
    RCLCPP_INFO(node->get_logger(), "  -> %s took %.2f s (planned %.2f s)",
                motion.stage_name.c_str(), actual_s,
                plannedSeconds(motion.trajectory));
    if (result.status !=
        piper_elevator_manipulation::ExecutionStatus::completed) {
      RCLCPP_ERROR(node->get_logger(), "CONTROLLER_FAILED at %s: %s",
                   motion.stage_name.c_str(), result.detail.c_str());
      return 1;
    }
    last_executed = &motion;
  }

  // Hand back a settled arm. A second observation taken while the arm is still
  // moving carries the motion as pose error, and with no hardware capture
  // timestamp there is nothing later that can take it back out.
  if (last_executed != nullptr) {
    waitForSettledJoints(node, joint_states, last_executed->trajectory,
                         "pre-press");
  }
  return 0;
}

int executeFromPrePress(const rclcpp::Node::SharedPtr& node,
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
  bool reached_approach = false;
  const PlannedMotion* press_motion = nullptr;
  const PlannedMotion* approach_motion = nullptr;
  for (const auto& motion : motions) {
    if (motion.stage_name == kPressStage) {
      press_motion = &motion;
      break;
    }
    if (motion.stage_name == kRetreatStage ||
        motion.stage_name == kReturnStage) {
      throw std::runtime_error("MTC solution has an unexpected phase order");
    }
    // The free-motion phases are behind us -- `runToPrePress` already ran them
    // and the arm is standing at the approach pose. Skipping rather than
    // repeating them is what makes the split a split: re-running would drag the
    // arm back to the start of the route and forward again.
    if (!reached_approach && motion.stage_name != kAdvanceStage) {
      continue;
    }
    reached_approach = true;
    RCLCPP_INFO(node->get_logger(), "Executing phase: %s (planned %.2f s)",
                motion.stage_name.c_str(), plannedSeconds(motion.trajectory));
    const auto phase_started = std::chrono::steady_clock::now();
    const auto result = executor.execute(
        motion.trajectory, motionTimeout(motion.trajectory));
    const double actual_s = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - phase_started).count();
    RCLCPP_INFO(node->get_logger(), "  -> %s took %.2f s (planned %.2f s)",
                motion.stage_name.c_str(), actual_s,
                plannedSeconds(motion.trajectory));
    if (result.status !=
        piper_elevator_manipulation::ExecutionStatus::completed) {
      RCLCPP_ERROR(node->get_logger(), "CONTROLLER_FAILED at %s: %s",
                   motion.stage_name.c_str(), result.detail.c_str());
      return 1;
    }
    approach_completed |= motion.stage_name == kAdvanceStage;
    if (motion.stage_name == kAdvanceStage) {
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
