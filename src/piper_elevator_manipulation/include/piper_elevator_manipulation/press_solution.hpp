#pragma once

#include <moveit/task_constructor/storage.h>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include <chrono>
#include <string>
#include <vector>

namespace piper_elevator_manipulation {

namespace mtc = moveit::task_constructor;

// One planned stage reduced to the single trajectory it contributes, tagged
// with the name of the stage that produced it.
struct PlannedMotion {
  std::string stage_name;
  trajectory_msgs::msg::JointTrajectory trajectory;
};

// Flattens an MTC solution tree into an ordered list of motions. A segment
// carrying fewer than two waypoints describes no motion and is dropped, which
// is why a phase can disappear from the list entirely when it collapses.
void collectMotions(const mtc::SolutionBase& solution,
                    std::vector<PlannedMotion>* motions);

// True when `sought_stage` contributed to this solution, at any nesting depth.
bool solutionUsesStage(const mtc::SolutionBase& solution,
                       const mtc::Stage* sought_stage);

// Duration of a planned trajectory, in seconds.
double plannedSeconds(const trajectory_msgs::msg::JointTrajectory& trajectory);

// How long to allow a trajectory to execute before treating it as stuck.
std::chrono::milliseconds motionTimeout(
    const trajectory_msgs::msg::JointTrajectory& trajectory);

}  // namespace piper_elevator_manipulation
