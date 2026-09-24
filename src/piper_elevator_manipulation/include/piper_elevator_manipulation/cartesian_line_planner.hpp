#pragma once

#include <moveit/task_constructor/solvers/planner_interface.h>

#include <Eigen/Core>

#include <string>

namespace piper_elevator_manipulation {

struct CartesianLineOptions {
  double max_translation_step_m{0.002};
  double max_lateral_error_m{0.003};
  double max_joint_jump_rad{0.20};
  double max_start_bound_error_rad{0.001};
  double position_tolerance_m{0.002};
  double direction_tolerance_rad{0.05};
};

// Interpolates TCP position along one world-frame axis and solves the 5D IK
// at each sample. Unlike joint interpolation, it checks the full Cartesian
// line, joint jumps, joint limits, and the current planning scene.
class CartesianLinePlanner final
    : public moveit::task_constructor::solvers::PlannerInterface {
 public:
  CartesianLinePlanner(std::string tip_link, Eigen::Vector3d panel_normal,
                       CartesianLineOptions options = {});

  void init(const moveit::core::RobotModelConstPtr& robot_model) override;

  Result plan(const planning_scene::PlanningSceneConstPtr& from,
              const planning_scene::PlanningSceneConstPtr& to,
              const moveit::core::JointModelGroup* group, double timeout,
              robot_trajectory::RobotTrajectoryPtr& result,
              const moveit_msgs::msg::Constraints& constraints) override;

  Result plan(const planning_scene::PlanningSceneConstPtr& from,
              const moveit::core::LinkModel& link,
              const Eigen::Isometry3d& offset,
              const Eigen::Isometry3d& target,
              const moveit::core::JointModelGroup* group, double timeout,
              robot_trajectory::RobotTrajectoryPtr& result,
              const moveit_msgs::msg::Constraints& constraints) override;

  std::string getPlannerId() const override { return "FiveDofCartesianLine"; }

 private:
  std::string tip_link_;
  Eigen::Vector3d panel_normal_;
  CartesianLineOptions options_;
};

}  // namespace piper_elevator_manipulation
