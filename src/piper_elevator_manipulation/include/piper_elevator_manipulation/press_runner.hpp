#pragma once

#include "piper_elevator_manipulation/joint_state_cache.hpp"
#include "piper_elevator_manipulation/press_parameters.hpp"
#include "piper_elevator_manipulation/target_source.hpp"

#include <rclcpp/rclcpp.hpp>

#include <optional>
#include <string>

namespace piper_elevator_manipulation {

// The gates that decide whether a planned press is allowed to move anything:
// which execution mode, and the two independent switches that have to be on
// before a real arm is commanded.
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
    piper_elevator_manipulation::TargetSource& source, double timeout_seconds);

// Plans and, when the gates allow it, runs exactly one press. Returns 0 when
// the press completed and the arm returned to where it started.
int runSinglePress(const rclcpp::Node::SharedPtr& node,
                   const RunSettings& settings,
                   const PressTaskParameters& parameters,
                   JointStateCache& joint_states);

}  // namespace piper_elevator_manipulation
