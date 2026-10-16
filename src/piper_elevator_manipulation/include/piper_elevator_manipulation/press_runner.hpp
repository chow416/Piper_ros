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

// Parameters of the look-again-then-correct step.
//
// `rounds == 0` turns the whole feature off, and that is a first-class setting
// rather than a debug flag: it is the regression baseline the newly split
// executor has to reproduce exactly. Nothing here defaults to "on and
// moving" -- zero rounds, and a wait timeout that is deliberately short,
// because a broken camera must degrade the aim rather than hang the task.
struct RefinementParams {
  int rounds{1};
  std::string topic{"/press/refine_pose"};
  //: Must be non-zero. A vision failure has to cost a worse aim, never a task
  //: that never proceeds.
  double wait_timeout_s{2.0};
  //: How much of the correction to believe. Deliberately not 1: the second
  //: observation is closer but has no redundancy at all, and "closer is more
  //: accurate" has not been measured. See the fusion note in the design.
  double fusion_weight{0.5};
  //: Rejects a refined position this far from the coarse one. With a single
  //: marker this is the main safety gate -- a mirrored or otherwise wrong solve
  //: lands somewhere else entirely, and this is what stops it.
  double max_correction_m{0.020};
  //: Rejects a refined *normal* this far off. The normal is never fused, only
  //: watched: it is one number that says whether the solve went wild, and a
  //: single marker can produce a wild one while its position still looks sane.
  double max_angle_deg{10.0};
  //: Minimum age of a refined observation relative to the moment the arm
  //: settled. The camera has no hardware exposure stamp, so the locator's
  //: publish stamp stands in for the capture time; the guard has to cover the
  //: capture-to-publish pipeline delay, or a frame shot while the arm was
  //: still moving would be consumed after it stopped and carry the motion as
  //: pose error.
  double static_guard_s{0.3};
};

// Waits for one accepted target, reporting rejected messages as they pile up so
// that a silent wait can be told apart from a publisher that is being refused.
std::optional<piper_elevator_manipulation::PressTarget> waitForTarget(
    const rclcpp::Node::SharedPtr& node,
    piper_elevator_manipulation::TargetSource& source, double timeout_seconds);

// Plans and, when the gates allow it, runs exactly one press. Returns 0 when
// the press completed and the arm returned to where it started.
//
// `refine_source` may be null, which is the single-board deployment: the press
// then runs exactly as it did before the refinement step existed. When it is
// present the refined observation is used to move the aim -- and any failure to
// get one degrades to the coarse target with a warning, because vision is a
// gain here, not a dependency.
int runSinglePress(const rclcpp::Node::SharedPtr& node,
                   const RunSettings& settings,
                   const PressTaskParameters& parameters,
                   JointStateCache& joint_states,
                   TargetSource* refine_source = nullptr,
                   const RefinementParams& refinement = {});

}  // namespace piper_elevator_manipulation
