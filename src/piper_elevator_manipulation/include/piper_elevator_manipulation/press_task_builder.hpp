#pragma once

#include "piper_elevator_manipulation/press_parameters.hpp"

#include <Eigen/Core>
#include <moveit/task_constructor/container.h>
#include <moveit/task_constructor/task.h>
#include <rclcpp/rclcpp.hpp>

#include <map>
#include <memory>
#include <string>

namespace piper_elevator_manipulation {

namespace mtc = moveit::task_constructor;

// A task that has been built but not planned, together with the pieces the
// executor and the recovery step need afterwards.
struct PlanningTask {
  std::unique_ptr<mtc::Task> task;
  const mtc::SerialContainer* prerecorded_free_motion{nullptr};
  std::map<std::string, double> pre_press_joints;
  std::map<std::string, double> return_joints;
};

// Builds the free-motion, approach, press, retreat and return stages.
//
// Nothing is planned here: this solves the three waypoints and assembles the
// stage tree. The planning happens in the caller's `task->plan(1)`, so a dry run
// can build and inspect the tree without commanding anything.
//
// `use_prerecorded_route` selects how the free-motion segment is planned. True
// is the normal case and replays the recorded route. False makes that segment
// fall back to the sampling planner, which is what the *second* build after a
// refined observation needs: by then the arm already stands at the old approach
// pose, so replaying a route recorded from elsewhere would drag it back to that
// route's start and forward again.
//
// `live_positions`, when given, is tried as an IK seed ahead of every other.
// The second build passes it: with the route dropped above, the only seed left
// is the captured start state, which is a pose from before the press began --
// metres from the target -- while the arm itself is standing centimetres from
// the answer. A five-axis arm's pose for a given position and direction is not
// unique, and the seed is what picks the branch.
PlanningTask createTask(
    const rclcpp::Node::SharedPtr& node,
    const PressTaskParameters& parameters,
    const std::map<std::string, double>& start_positions,
    bool use_prerecorded_route = true,
    const std::map<std::string, double>* live_positions = nullptr);

// Builds a two-stage task that retreats along the panel normal and returns to
// the recorded start state, both replanned from wherever the arm actually is.
//
// The plan's own retreat and return stages are deliberately not used after a
// press: a contact-cancelled goal stops somewhere in the middle, so a retreat
// that assumes full travel would start from the wrong place.
std::unique_ptr<mtc::Task> createRecoveryTask(
    const rclcpp::Node::SharedPtr& node,
    const Eigen::Vector3d& panel_normal,
    const std::map<std::string, double>& pre_press_joints,
    const std::map<std::string, double>& return_joints);

}  // namespace piper_elevator_manipulation
