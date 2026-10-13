#include "piper_elevator_manipulation/joint_state_cache.hpp"
#include "piper_elevator_manipulation/press_parameters.hpp"
#include "piper_elevator_manipulation/press_runner.hpp"
#include "piper_elevator_manipulation/target_source.hpp"

#include <Eigen/Core>
#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

using piper_elevator_manipulation::EditablePressParams;
using piper_elevator_manipulation::JointStateCache;
using piper_elevator_manipulation::PressTaskParameters;
using piper_elevator_manipulation::RunSettings;
using piper_elevator_manipulation::getOrDeclareParameter;
using piper_elevator_manipulation::readPressParamsFile;
using piper_elevator_manipulation::runSinglePress;
using piper_elevator_manipulation::validatePressParameters;
using piper_elevator_manipulation::vector3Parameter;
using piper_elevator_manipulation::waitForTarget;

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  auto node = rclcpp::Node::make_shared("piper_press_button_demo", options);
  const auto logger = node->get_logger();

  RunSettings settings;
  settings.execute = getOrDeclareParameter(node, "execute", false);
  settings.execution_mode =
      getOrDeclareParameter(node, "execution_mode", std::string("simulation"));
  // A second, explicit gate for real hardware. Reading the target from launch
  // arguments and commanding a real arm must never be one careless flag apart.
  settings.hardware_enable =
      getOrDeclareParameter(node, "hardware_enable", false);
  settings.simulated_contact =
      getOrDeclareParameter(node, "simulated_contact", false);
  settings.use_sim_time = getOrDeclareParameter(node, "use_sim_time", false);
  settings.travel_margin =
      getOrDeclareParameter(node, "hardware_travel_margin_m", 0.003);
  if (settings.execution_mode != "simulation" &&
      settings.execution_mode != "hardware_fixed_travel") {
    RCLCPP_ERROR(logger,
                 "execution_mode must be 'simulation' or 'hardware_fixed_travel'");
    rclcpp::shutdown();
    return 2;
  }

  // Where the target comes from. 'parameters' is a fixed point read from the
  // launch arguments; 'topic' is the perception interface, where each accepted
  // message asks for one press.
  const std::string target_source_name =
      getOrDeclareParameter(node, "target_source", std::string("parameters"));
  const bool targets_from_topic = target_source_name == "topic";
  if (!targets_from_topic && target_source_name != "parameters") {
    RCLCPP_ERROR(logger, "target_source must be 'parameters' or 'topic', not '%s'",
                 target_source_name.c_str());
    rclcpp::shutdown();
    return 2;
  }
  const int64_t max_presses = getOrDeclareParameter<int64_t>(node, "max_presses", 1);
  if (max_presses < 0) {
    RCLCPP_ERROR(logger, "max_presses must be 0 (unlimited) or a positive count");
    rclcpp::shutdown();
    return 2;
  }

  const double approach_distance =
      getOrDeclareParameter(node, "approach_distance", 0.030);
  // The standoff is what makes the approach stop visibly clear of the target on
  // hardware, which is also what makes the resulting gap measurable. The value
  // in use, and the range it has to satisfy, live in config/press_params.yaml.
  const double contact_standoff =
      getOrDeclareParameter(node, "contact_standoff_m", 0.005);
  const double press_depth = getOrDeclareParameter(node, "press_depth", 0.004);

  // Optional operator-editable file. Empty means "use the ROS parameters", which
  // is what every launch except press_button_hardware keeps doing; that one
  // points it at config/press_params.yaml, where the numbers sit next to the
  // ranges they have to satisfy.
  const std::string press_params_file =
      getOrDeclareParameter(node, "press_params_file", std::string(""));

  std::unique_ptr<piper_elevator_manipulation::TargetSource> target_channel;
  try {
    if (targets_from_topic) {
      piper_elevator_manipulation::TargetSourceOptions source_options;
      source_options.topic = getOrDeclareParameter(
          node, "target_topic", std::string("/press/target_pose"));
      source_options.direction_sign =
          getOrDeclareParameter(node, "press_direction_sign", 1.0);
      source_options.max_age = std::chrono::milliseconds(static_cast<int64_t>(
          1000.0 * getOrDeclareParameter(node, "target_max_age_s", 1.0)));
      source_options.workspace_min = vector3Parameter(
          node, "workspace_min", Eigen::Vector3d(-0.30, -0.50, -0.02));
      source_options.workspace_max = vector3Parameter(
          node, "workspace_max", Eigen::Vector3d(0.65, 0.50, 0.80));
      source_options.accepted_frames = getOrDeclareParameter(
          node, "target_accepted_frames",
          std::vector<std::string>{"world", "base_link"});
      if (source_options.direction_sign != 1.0 &&
          source_options.direction_sign != -1.0) {
        RCLCPP_ERROR(logger, "press_direction_sign must be +1 or -1");
        rclcpp::shutdown();
        return 2;
      }
      target_channel = std::make_unique<piper_elevator_manipulation::TargetSource>(
          node, source_options);
    }
  } catch (const std::exception& exception) {
    RCLCPP_ERROR(logger, "%s", exception.what());
    rclcpp::shutdown();
    return 2;
  }

  JointStateCache joint_states(node);
  std::thread spin_thread([node] { rclcpp::spin(node); });

  int exit_code = 0;
  int64_t press_count = 0;
  std::string reported_params;
  bool overlap_warned = false;
  while (rclcpp::ok() && (max_presses == 0 || press_count < max_presses)) {
    // Re-read the editable file every press so an edit lands on the next one
    // without restarting the service. The ROS parameters stay as the fallback,
    // so a missing or broken file behaves exactly as it did before the file
    // existed.
    double approach = approach_distance;
    double standoff = contact_standoff;
    double depth = press_depth;
    std::string params_source = "ROS parameters";
    if (!press_params_file.empty()) {
      EditablePressParams edited;
      edited.approach_distance = approach_distance;
      edited.contact_standoff = contact_standoff;
      edited.press_depth = press_depth;
      if (readPressParamsFile(node, press_params_file, &edited)) {
        approach = edited.approach_distance;
        standoff = edited.contact_standoff;
        depth = edited.press_depth;
        params_source = press_params_file;
      }
    }
    // Say what will be used, but only when it changes. An operator editing the
    // file wants to see the new values land; a line per press would just be
    // noise, and this log is read while watching the arm.
    char summary[256];
    std::snprintf(summary, sizeof(summary),
                  "approach %.1f mm, contact standoff %.1f mm, press depth "
                  "%.1f mm",
                  approach * 1000.0, standoff * 1000.0, depth * 1000.0);
    if (summary != reported_params) {
      reported_params = summary;
      RCLCPP_INFO(logger, "Press parameters (from %s): %s",
                  params_source.c_str(), summary);
    }
    // The file wins over the launch arguments, because "one knob" is the whole
    // point of it. But winning silently would mean `contact_standoff_m:=0.02` on
    // the launch line doing nothing with no explanation, which is the worse
    // failure of the two. So say it, once.
    //
    // This is only reachable by launching the node directly now: the
    // `--standoff` / `--depth` flags that used to be the way in are refused
    // outright by tools/press_board.sh, precisely because of the failure this
    // warning describes.
    if (!overlap_warned && params_source != "ROS parameters" &&
        (std::abs(approach - approach_distance) > 1e-9 ||
         std::abs(standoff - contact_standoff) > 1e-9 ||
         std::abs(depth - press_depth) > 1e-9)) {
      overlap_warned = true;
      RCLCPP_WARN(logger,
                  "%s overrides the launch arguments for this press. Move the "
                  "file away, or pass press_params_file:='', if you meant to "
                  "drive these from the command line.",
                  press_params_file.c_str());
    }

    PressTaskParameters parameters;
    if (targets_from_topic) {
      // Drop anything that arrived while the arm was moving. A target observed
      // behind the arm is not worth reusing, and this keeps each press tied to
      // a fresh observation instead of to whatever the queue happened to hold.
      target_channel->discardPendingTarget();
      const auto target = waitForTarget(
          node, *target_channel,
          getOrDeclareParameter(node, "target_wait_timeout_s", 0.0));
      if (!target.has_value()) {
        exit_code = 1;
        break;
      }
      parameters.target_position = target->position;
      parameters.press_direction = target->press_direction;
      parameters.approach_distance = approach;
      parameters.contact_standoff = standoff;
      parameters.press_depth = depth;
    } else {
      parameters =
          PressTaskParameters{
              Eigen::Vector3d(
                  getOrDeclareParameter(node, "button_x", 0.300),
                  getOrDeclareParameter(node, "button_y", 0.000),
                  getOrDeclareParameter(node, "button_z", 0.240)),
              Eigen::Vector3d(
                  getOrDeclareParameter(node, "press_direction_x", 1.0),
                  getOrDeclareParameter(node, "press_direction_y", 0.0),
                  getOrDeclareParameter(node, "press_direction_z", 0.0)),
              approach, standoff, depth};
    }

    if (!validatePressParameters(node, parameters)) {
      exit_code = 2;
      break;
    }
    exit_code = runSinglePress(node, settings, parameters, joint_states);
    ++press_count;
    if (targets_from_topic) {
      RCLCPP_INFO(logger, "Press %ld finished with exit code %d.",
                  static_cast<long>(press_count), exit_code);
    }
  }

  if (target_channel) {
    RCLCPP_INFO(logger, "Target source '%s': %zu message(s) rejected in total.",
                target_channel->topic().c_str(), target_channel->rejectedCount());
  }

  rclcpp::shutdown();
  spin_thread.join();
  return exit_code;
}
