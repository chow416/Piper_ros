#include "piper_elevator_manipulation/five_dof_ik.hpp"

#include <Eigen/Cholesky>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_state/robot_state.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <random>
#include <sstream>
#include <utility>
#include <vector>

namespace piper_elevator_manipulation {
namespace {

struct TaskError {
  Eigen::Matrix<double, 6, 1> value;
  double position_norm;
  double direction_angle;
};

TaskError computeTaskError(moveit::core::RobotState& state,
                           const std::string& tip_link,
                           const Eigen::Vector3d& target_position,
                           const Eigen::Vector3d& target_direction,
                           double direction_weight) {
  state.updateLinkTransforms();
  const Eigen::Isometry3d& tcp = state.getGlobalLinkTransform(tip_link);
  const Eigen::Vector3d current_direction = tcp.linear().col(2).normalized();
  const double dot = std::clamp(current_direction.dot(target_direction), -1.0, 1.0);

  TaskError error;
  error.value.head<3>() = target_position - tcp.translation();
  error.value.tail<3>() = direction_weight * (target_direction - current_direction);
  error.position_norm = error.value.head<3>().norm();
  error.direction_angle = std::acos(dot);
  return error;
}

double seedDistance(const Eigen::VectorXd& values, const Eigen::VectorXd& seed,
                    const moveit::core::JointModelGroup* group) {
  double result = 0.0;
  const auto& names = group->getVariableNames();
  for (Eigen::Index i = 0; i < values.size(); ++i) {
    const auto& bounds = group->getParentModel().getVariableBounds(names[static_cast<std::size_t>(i)]);
    const double range = std::max(1.0e-6, bounds.max_position_ - bounds.min_position_);
    const double normalized = (values[i] - seed[i]) / range;
    result += normalized * normalized;
  }
  return result;
}

}  // namespace

FiveDofIkSolver::FiveDofIkSolver(FiveDofIkOptions options)
    : options_(std::move(options)) {}

std::optional<std::map<std::string, double>> FiveDofIkSolver::solve(
    const moveit::core::RobotModelConstPtr& robot_model,
    const std::string& group_name,
    const std::string& tip_link,
    const std::map<std::string, double>& seed,
    const Eigen::Vector3d& target_position,
    const Eigen::Vector3d& target_direction,
    std::string* failure_reason) const {
  const auto* group = robot_model->getJointModelGroup(group_name);
  if (group == nullptr) {
    if (failure_reason != nullptr) {
      *failure_reason = "unknown joint group '" + group_name + "'";
    }
    return std::nullopt;
  }
  if (robot_model->getLinkModel(tip_link) == nullptr) {
    if (failure_reason != nullptr) {
      *failure_reason = "unknown TCP link '" + tip_link + "'";
    }
    return std::nullopt;
  }
  const auto& variable_names = group->getVariableNames();
  if (variable_names.size() != 5) {
    if (failure_reason != nullptr) {
      *failure_reason = "five-dimensional solver requires exactly five active joints";
    }
    return std::nullopt;
  }
  if (target_direction.norm() < 1.0e-9) {
    if (failure_reason != nullptr) {
      *failure_reason = "press direction has zero length";
    }
    return std::nullopt;
  }
  const Eigen::Vector3d direction = target_direction.normalized();

  moveit::core::RobotState seed_state(robot_model);
  seed_state.setToDefaultValues();
  seed_state.setVariablePositions(seed);
  seed_state.enforceBounds(group);
  seed_state.update();

  Eigen::VectorXd seed_values;
  seed_state.copyJointGroupPositions(group, seed_values);

  std::vector<Eigen::VectorXd> starts;
  starts.push_back(seed_values);

  Eigen::VectorXd midpoint(seed_values.size());
  for (std::size_t i = 0; i < variable_names.size(); ++i) {
    const auto& bounds = robot_model->getVariableBounds(variable_names[i]);
    midpoint[static_cast<Eigen::Index>(i)] = 0.5 * (bounds.min_position_ + bounds.max_position_);
  }
  starts.push_back(midpoint);

  std::mt19937 generator(20260922U);
  for (int restart = 0; restart < options_.random_restarts; ++restart) {
    Eigen::VectorXd sample(seed_values.size());
    for (std::size_t i = 0; i < variable_names.size(); ++i) {
      const auto& bounds = robot_model->getVariableBounds(variable_names[i]);
      std::uniform_real_distribution<double> distribution(bounds.min_position_, bounds.max_position_);
      sample[static_cast<Eigen::Index>(i)] = distribution(generator);
    }
    starts.push_back(sample);
  }

  planning_scene::PlanningScene self_collision_scene(robot_model);
  std::optional<Eigen::VectorXd> best_solution;
  double best_cost = std::numeric_limits<double>::infinity();
  double best_position_error = std::numeric_limits<double>::infinity();
  double best_direction_error = std::numeric_limits<double>::infinity();

  for (Eigen::VectorXd values : starts) {
    moveit::core::RobotState state(robot_model);
    state.setToDefaultValues();

    for (int iteration = 0; iteration < options_.max_iterations; ++iteration) {
      state.setJointGroupPositions(group, values);
      state.enforceBounds(group);
      state.copyJointGroupPositions(group, values);
      state.update();

      const TaskError error = computeTaskError(
          state, tip_link, target_position, direction, options_.direction_weight);
      best_position_error = std::min(best_position_error, error.position_norm);
      best_direction_error = std::min(best_direction_error, error.direction_angle);

      if (error.position_norm <= options_.position_tolerance &&
          error.direction_angle <= options_.direction_tolerance_rad) {
        if (!self_collision_scene.isStateColliding(state, group_name)) {
          const double cost = seedDistance(values, seed_values, group);
          if (cost < best_cost) {
            best_cost = cost;
            best_solution = values;
          }
        }
        break;
      }

      Eigen::MatrixXd jacobian(6, values.size());
      const Eigen::Isometry3d base_tcp = state.getGlobalLinkTransform(tip_link);
      const Eigen::Vector3d base_direction = base_tcp.linear().col(2).normalized();

      for (Eigen::Index column = 0; column < values.size(); ++column) {
        Eigen::VectorXd perturbed = values;
        const auto& bounds = robot_model->getVariableBounds(
            variable_names[static_cast<std::size_t>(column)]);
        double delta = options_.finite_difference_step;
        if (perturbed[column] + delta > bounds.max_position_) {
          delta = -options_.finite_difference_step;
        }
        perturbed[column] = std::clamp(
            perturbed[column] + delta, bounds.min_position_, bounds.max_position_);
        const double actual_delta = perturbed[column] - values[column];
        if (std::abs(actual_delta) < 1.0e-12) {
          jacobian.col(column).setZero();
          continue;
        }

        moveit::core::RobotState shifted(state);
        shifted.setJointGroupPositions(group, perturbed);
        shifted.update();
        const Eigen::Isometry3d shifted_tcp = shifted.getGlobalLinkTransform(tip_link);
        const Eigen::Vector3d shifted_direction = shifted_tcp.linear().col(2).normalized();
        jacobian.block<3, 1>(0, column) =
            (shifted_tcp.translation() - base_tcp.translation()) / actual_delta;
        jacobian.block<3, 1>(3, column) = options_.direction_weight *
            (shifted_direction - base_direction) / actual_delta;
      }

      const Eigen::MatrixXd normal = jacobian.transpose() * jacobian +
          options_.damping * options_.damping *
              Eigen::MatrixXd::Identity(values.size(), values.size());
      Eigen::VectorXd step = normal.ldlt().solve(jacobian.transpose() * error.value);
      if (!step.allFinite()) {
        break;
      }
      for (Eigen::Index i = 0; i < step.size(); ++i) {
        step[i] = std::clamp(step[i], -options_.max_joint_step, options_.max_joint_step);
      }
      values += step;
    }
  }

  if (!best_solution.has_value()) {
    if (failure_reason != nullptr) {
      std::ostringstream message;
      message << "no collision-free 5D IK solution; best position error="
              << best_position_error << " m, best direction error="
              << best_direction_error * 57.29577951308232 << " deg";
      *failure_reason = message.str();
    }
    return std::nullopt;
  }

  std::map<std::string, double> result;
  for (std::size_t i = 0; i < variable_names.size(); ++i) {
    result[variable_names[i]] = (*best_solution)[static_cast<Eigen::Index>(i)];
  }
  return result;
}

}  // namespace piper_elevator_manipulation
