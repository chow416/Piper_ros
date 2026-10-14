#pragma once

#include <Eigen/Core>
#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <string>
#include <vector>

namespace piper_elevator_manipulation {

// What to press: the validated request that createTask turns into a motion.
struct PressTaskParameters {
  Eigen::Vector3d target_position;
  Eigen::Vector3d press_direction;
  double approach_distance;
  //: How far short of `target_position` the approach stops -- which is also
  //: where the press phase starts.
  //:
  //: It exists because the aim point is only as good as the calibration behind
  //: it, and anything that puts it inside the real surface turns an approach
  //: into a collision. With a standoff the same error eats a visible gap
  //: instead, and that gap is something an operator can put a shim into and
  //: measure.
  double contact_standoff;
  //: How much further to push from where the approach stopped. The press phase
  //: travels exactly this, so this is the real penetration.
  //:
  //: It is measured from the *contact waypoint*, not from `target_position`.
  //: That matters: measured from the target, the standoff would move where the
  //: press starts but not where it ends, so tuning the standoff could not stop
  //: the rod from driving past an aim point that sits inside the real surface.
  //: Anchored to the contact waypoint, the standoff shifts the whole press --
  //: approach end and press end together -- and `press_depth` keeps meaning
  //: "how deep I push", independent of how wrong the calibration is.
  double press_depth;
};

// Settings that describe how to run a press, as opposed to what to press.
// ------------------------------------------------------- 可编辑的按压参数文件
// An operator changes these by editing one file, instead of passing launch
// arguments that need the whole service restarted.
//
// Why a file and not `ros2 param set`: these numbers belong next to each other,
// with their allowed ranges written down beside them, and they should survive a
// restart. A one-off param set does neither.
//
// It is re-read before every press, so an edit takes effect on the next press.
// That matters here because the standoff is the one value that gets tuned by
// repeated trial -- every restart costs a planning-scene handshake and a route,
// which is exactly the kind of friction that stops people from tuning at all.
//
// ⚠️ Deliberately NOT in this file: `execute` and `hardware_enable`. Those two
// gates have to be typed on the command line for every session. A saved `true`
// in a file that outlives the session is exactly the "one careless flag apart"
// that this launch profile is built to avoid.
struct EditablePressParams {
  double approach_distance{0.030};
  double contact_standoff{0.005};
  double press_depth{0.004};

  // The look-again-then-correct step. These sit here for the same reason the
  // three above do: they get tuned by repeated trial, and a tunable that needs
  // a restart is a tunable nobody touches. They are seeded from the launch
  // parameters before the file is read, so a key the file does not mention
  // keeps whatever the launch line said rather than snapping to a code default.
  //
  // `refine_rounds` doubles as the feature switch, so zero still restores the
  // pre-refinement behaviour exactly.
  int refine_rounds{1};
  double refine_wait_timeout_s{2.0};
  double refine_fusion_weight{0.5};
  double refine_max_correction_m{0.020};
  double refine_max_angle_deg{10.0};
};

// Reads a ROS parameter, declaring it with `default_value` when it does not
// exist yet. A parameter of the wrong type is reported and the default is used,
// so a launch-file typo cannot take the node down.
//
// A template, so the body has to live in the header.
template <typename T>
T getOrDeclareParameter(const rclcpp::Node::SharedPtr& node,
                        const std::string& name, const T& default_value) {
  if (!node->has_parameter(name)) {
    return node->declare_parameter<T>(name, default_value);
  }
  T value{};
  if (!node->get_parameter(name, value)) {
    RCLCPP_WARN(node->get_logger(),
                "Parameter '%s' has an unexpected type; using its default.",
                name.c_str());
    return default_value;
  }
  return value;
}

// Velocity/acceleration scaling for the Cartesian approach and retreat, which
// have a lower ceiling than free motion because a straight line has no room to
// coast.
double speedScaling(const rclcpp::Node::SharedPtr& node,
                    const std::string& key, double default_value);

// Same contract as free_motion_route.cpp's readPositiveScaling, so the OMPL
// fallback and the prerecorded route cannot drift apart. Those two must agree:
// they are alternative branches of the same free-motion segment, and a
// different speed between them would be visible as a lurch whenever the
// recorded route is rejected.
double freeMotionScaling(const rclcpp::Node::SharedPtr& node,
                         const std::string& key, double default_value);

// Reads a ROS parameter that has to hold exactly three numbers.
Eigen::Vector3d vector3Parameter(const rclcpp::Node::SharedPtr& node,
                                 const std::string& name,
                                 const Eigen::Vector3d& default_value);

// The validated value of `press_timeout_s`.
std::chrono::milliseconds pressTimeout(const rclcpp::Node::SharedPtr& node);

// Reads the editable file. Returns false -- and says why -- when it is absent or
// unusable. The caller then falls back to the ROS parameters, which is exactly
// the behaviour that existed before this file did: a broken file degrades to
// "same as yesterday", not to "refuses to press".
bool readPressParamsFile(const rclcpp::Node::SharedPtr& node,
                         const std::string& path, EditablePressParams* out);

// Checks a request against the ranges the parameters document, and normalises
// the press direction in place.
bool validatePressParameters(const rclcpp::Node::SharedPtr& node,
                             PressTaskParameters& parameters);

}  // namespace piper_elevator_manipulation
