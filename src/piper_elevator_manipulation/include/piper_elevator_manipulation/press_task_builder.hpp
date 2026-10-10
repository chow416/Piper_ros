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
PlanningTask createTask(
    const rclcpp::Node::SharedPtr& node,
    const PressTaskParameters& parameters,
    const std::map<std::string, double>& start_positions);

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
