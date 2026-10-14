#pragma once

#include "piper_elevator_manipulation/joint_state_cache.hpp"
#include "piper_elevator_manipulation/press_parameters.hpp"
#include "piper_elevator_manipulation/press_task_builder.hpp"

#include <rclcpp/rclcpp.hpp>

namespace piper_elevator_manipulation {

// Executes the validated solution one phase at a time, in two pieces split at
// the approach pose.
//
// The split is what makes an observation *between* two phases possible at all.
// Correcting the aim means re-planning the approach and the press against a new
// target, and that can only be done while the arm is standing still at the
// approach pose -- which is exactly where the two halves meet.

// Executes the free-motion phases and stops at the approach pose.
//
// Deliberately does not include the approach: if the aim is going to be
// corrected, the correction has to happen before the arm commits to the contact
// line. Ends with the joints settled, because a second observation taken from a
// moving arm carries the motion as pose error and this camera has no hardware
// capture timestamp to correct it with afterwards.
int runToPrePress(
    const rclcpp::Node::SharedPtr& node,
    const PlanningTask& planning_task,
    JointStateCache& joint_states);

// Executes the approach, the press and the recovery, in that order.
//
// The free-motion phases are skipped rather than re-run: the arm is already at
// the approach pose, and repeating them would drag it back to the start of the
// route and forward again.
//
// `require_contact` selects the stop condition for the press phase:
//   true  - simulation. The press is cancelled on stable virtual contact and
//           completing the full travel is treated as a failure.
//   false - hardware without a contact sensor. The planned trajectory defines
//           the travel, and the independently measured TCP travel is still
//           enforced but as a safety cap above the plan, so a correct press
//           ends because the trajectory completed, never because the cap hit.
int executeFromPrePress(
    const rclcpp::Node::SharedPtr& node,
    const PressTaskParameters& parameters,
    const PlanningTask& planning_task,
    JointStateCache& joint_states,
    bool require_contact);

}  // namespace piper_elevator_manipulation
