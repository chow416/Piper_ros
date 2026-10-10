#include "piper_elevator_manipulation/press_parameters.hpp"

#include <yaml-cpp/yaml.h>

#include <cmath>
#include <cstdint>
#include <fstream>
#include <stdexcept>

namespace piper_elevator_manipulation {

double speedScaling(const rclcpp::Node::SharedPtr& node,
                    const std::string& key, double default_value) {
  const double value = getOrDeclareParameter(node, key, default_value);
  if (!std::isfinite(value) || value <= 0.0 || value > 0.2) {
    throw std::invalid_argument(key + " must be in (0, 0.2]");
  }
  return value;
}

// Same contract as free_motion_route.cpp's readPositiveScaling, so the OMPL
// fallback and the prerecorded route cannot drift apart. Those two must agree:
// they are alternative branches of the same free-motion segment, and a
// different speed between them would be visible as a lurch whenever the
// recorded route is rejected.
double freeMotionScaling(const rclcpp::Node::SharedPtr& node,
                         const std::string& key, double default_value) {
  const double value = getOrDeclareParameter(node, key, default_value);
  if (!std::isfinite(value) || value <= 0.0 || value > 1.0) {
    throw std::invalid_argument(key + " must be finite and in (0, 1]");
  }
  return value;
}

Eigen::Vector3d vector3Parameter(const rclcpp::Node::SharedPtr& node,
                                 const std::string& name,
                                 const Eigen::Vector3d& default_value) {
  const std::vector<double> default_list{default_value.x(), default_value.y(),
                                         default_value.z()};
  const auto values = getOrDeclareParameter(node, name, default_list);
  if (values.size() != 3) {
    throw std::invalid_argument(name + " must list exactly three numbers");
  }
  return Eigen::Vector3d(values[0], values[1], values[2]);
}

std::chrono::milliseconds pressTimeout(const rclcpp::Node::SharedPtr& node) {
  const double seconds = getOrDeclareParameter(node, "press_timeout_s", 15.0);
  if (!std::isfinite(seconds) || seconds < 1.0 || seconds > 30.0) {
    throw std::invalid_argument("press_timeout_s must be in [1, 30]");
  }
  return std::chrono::milliseconds(
      static_cast<int64_t>(seconds * 1000.0));
}

// Reads the editable file. Returns false -- and says why -- when it is absent or
// unusable. The caller then falls back to the ROS parameters, which is exactly
// the behaviour that existed before this file did: a broken file degrades to
// "same as yesterday", not to "refuses to press".
bool readPressParamsFile(const rclcpp::Node::SharedPtr& node,
                         const std::string& path,
                         EditablePressParams* out) {
  std::ifstream stream(path);
  if (!stream) {
    return false;  // absent is normal when launch pointed us at nothing
  }

  const auto number = [&](const YAML::Node& root, const char* key,
                          double fallback) {
    const YAML::Node value = root[key];
    if (!value) {
      return fallback;
    }
    try {
      return value.as<double>();
    } catch (const YAML::Exception& exception) {
      RCLCPP_ERROR(node->get_logger(), "%s: %s: %s", path.c_str(), key,
                   exception.what());
      return fallback;
    }
  };
  try {
    const YAML::Node root = YAML::Load(stream);
    if (!root.IsMap()) {
      RCLCPP_ERROR(node->get_logger(),
                   "%s: top level must be a mapping of parameter names to "
                   "values",
                   path.c_str());
      return false;
    }
    out->approach_distance =
        number(root, "approach_distance", out->approach_distance);
    out->contact_standoff =
        number(root, "contact_standoff_m", out->contact_standoff);
    out->press_depth = number(root, "press_depth", out->press_depth);
  } catch (const YAML::Exception& exception) {
    RCLCPP_ERROR(node->get_logger(), "%s: %s", path.c_str(), exception.what());
    return false;
  }
  return true;
}

bool validatePressParameters(const rclcpp::Node::SharedPtr& node,
                             PressTaskParameters& parameters) {
  if (!parameters.target_position.allFinite() ||
      !parameters.press_direction.allFinite() ||
      parameters.press_direction.norm() < 1.0e-6) {
    RCLCPP_ERROR(node->get_logger(),
                 "Target position and non-zero press direction must be finite.");
    return false;
  }
  if (!std::isfinite(parameters.approach_distance) ||
      parameters.approach_distance < 0.01 ||
      parameters.approach_distance > 0.15) {
    RCLCPP_ERROR(node->get_logger(),
                 "approach_distance must be in [0.01, 0.15] m");
    return false;
  }
  if (!std::isfinite(parameters.contact_standoff) ||
      parameters.contact_standoff < 0.0 ||
      parameters.contact_standoff > 0.05) {
    RCLCPP_ERROR(node->get_logger(),
                 "contact_standoff_m must be in [0, 0.05] m");
    return false;
  }
  if (parameters.contact_standoff >= parameters.approach_distance) {
    RCLCPP_ERROR(node->get_logger(),
                 "contact_standoff_m (%.4f) must be smaller than "
                 "approach_distance (%.4f), or the approach would already be "
                 "past the contact point",
                 parameters.contact_standoff, parameters.approach_distance);
    return false;
  }
  if (!std::isfinite(parameters.press_depth) ||
      parameters.press_depth <= 0.0 || parameters.press_depth > 0.015) {
    RCLCPP_ERROR(node->get_logger(), "press_depth must be in (0, 0.015] m");
    return false;
  }
  parameters.press_direction.normalize();
  return true;
}

}  // namespace piper_elevator_manipulation
