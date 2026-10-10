#pragma once

#include "piper_elevator_manipulation/joint_state_cache.hpp"
#include "piper_elevator_manipulation/press_parameters.hpp"
#include "piper_elevator_manipulation/press_task_builder.hpp"

#include <rclcpp/rclcpp.hpp>

namespace piper_elevator_manipulation {

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
    bool require_contact);

}  // namespace piper_elevator_manipulation
