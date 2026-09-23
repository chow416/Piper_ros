#pragma once

#include <Eigen/Core>
#include <moveit/robot_model/robot_model.h>

#include <map>
#include <optional>
#include <string>

namespace piper_elevator_manipulation {

struct FiveDofIkOptions {
  int max_iterations{250};
  int random_restarts{32};
  double finite_difference_step{1.0e-5};
  double damping{2.0e-3};
  double max_joint_step{0.15};
  double direction_weight{0.20};
  double position_tolerance{1.5e-3};
  double direction_tolerance_rad{0.03490658503988659};
};

// Solves the task-relevant five dimensions: TCP position (x, y, z) and the
// direction of the press axis (two independent angular dimensions). Rotation
// about the cylindrical press rod is deliberately unconstrained.
class FiveDofIkSolver {
public:
  explicit FiveDofIkSolver(FiveDofIkOptions options = {});

  std::optional<std::map<std::string, double>> solve(
      const moveit::core::RobotModelConstPtr& robot_model,
      const std::string& group_name,
      const std::string& tip_link,
      const std::map<std::string, double>& seed,
      const Eigen::Vector3d& target_position,
      const Eigen::Vector3d& target_direction,
      std::string* failure_reason = nullptr) const;

private:
  FiveDofIkOptions options_;
};

}  // namespace piper_elevator_manipulation
