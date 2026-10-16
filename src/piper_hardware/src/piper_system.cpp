#include "piper_hardware/piper_system.hpp"

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/logging.hpp>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <thread>
#include <utility>

namespace piper_hardware {
namespace {

constexpr char kLoggerName[] = "PiperSystem";

// Piper CAN identifiers. Values were verified against piper_sdk and against a
// live arm on can0: 0x2A5/0x2A6/0x2A7 carry joint angles, 0x2A2..0x2A4 carry
// the end-effector pose, and the 0x155..0x157 family carries joint commands.
constexpr uint32_t kCanIdMotionCtrl1 = 0x150;    // emergency-stop / teach control
constexpr uint32_t kCanIdMotionCtrl2 = 0x151;    // mode, enable, speed ratio
constexpr uint32_t kCanIdJointCtrl12 = 0x155;    // joint1, joint2
constexpr uint32_t kCanIdJointCtrl34 = 0x156;    // joint3, joint4
constexpr uint32_t kCanIdJointCtrl56 = 0x157;    // joint5, joint6
constexpr uint32_t kCanIdMotorEnable = 0x471;    // per-motor enable/disable
constexpr uint32_t kCanIdStatusFeedback = 0x2A1;
constexpr uint32_t kCanIdJointFeedback12 = 0x2A5;
constexpr uint32_t kCanIdJointFeedback34 = 0x2A6;
constexpr uint32_t kCanIdJointFeedback56 = 0x2A7;
constexpr uint32_t kCanIdGripperFeedback = 0x2A8;
constexpr uint32_t kCanIdLowSpeedFeedbackFirst = 0x261;
constexpr uint32_t kCanIdLowSpeedFeedbackLast = 0x266;

constexpr uint8_t kCtrlModeCanCommand = 0x01;
constexpr uint8_t kMoveModeJoint = 0x01;
constexpr uint8_t kMitModePositionVelocity = 0x00;
// The installed Piper V2 SDK's EnablePiper() handshake repeatedly sends 0x07
// and the live arm confirms every joint enabled. Keep this firmware-specific
// selector rather than trusting the contradictory generic message comment.
constexpr uint8_t kMotorAllJoints = 0x07;
constexpr uint8_t kEnableFlagDisable = 0x01;
constexpr uint8_t kEnableFlagEnable = 0x02;
constexpr uint8_t kEmergencyStopResume = 0x02;
constexpr uint8_t kDriverEnableStatusBit = 0x40;

constexpr double kRadToMilliDeg = 1000.0 * 180.0 / M_PI;
constexpr double kMilliDegToRad = M_PI / (1000.0 * 180.0);

double radiansToDegrees(double radians) { return radians * 180.0 / M_PI; }

// Wall-clock budget spent waiting for the first complete feedback frame during
// activation. Activation runs outside the real-time loop, so a short block is
// acceptable; without this the first command would be sent blind.
constexpr double kActivationFeedbackTimeoutSeconds = 3.0;
constexpr double kMotorEnableTimeoutSeconds = 2.0;

// The Piper protocol is big-endian on the wire. piper_sdk encodes commands with
// struct.pack(">i", value) and decodes feedback with int.from_bytes(..., "big"),
// so both directions must use `>` order. Verified against a live arm: the frame
// 2A6 = 00 00 07 86 00 00 07 68 decodes to joint3 = 1926, joint4 = 1896.
int32_t readInt32(const PiperCan::Frame& frame, std::size_t offset) {
  uint32_t raw = 0;
  for (std::size_t byte = 0; byte < 4; ++byte) {
    raw = (raw << 8) | static_cast<uint32_t>(frame.data[offset + byte]);
  }
  return static_cast<int32_t>(raw);
}

void writeInt32(PiperCan::Frame* frame, std::size_t offset, int32_t value) {
  const uint32_t raw = static_cast<uint32_t>(value);
  for (std::size_t byte = 0; byte < 4; ++byte) {
    frame->data[offset + byte] =
        static_cast<uint8_t>((raw >> (8 * (3 - byte))) & 0xFFu);
  }
}

bool parseBoolean(const std::string& text, bool fallback) {
  if (text == "true" || text == "True" || text == "1") {
    return true;
  }
  if (text == "false" || text == "False" || text == "0") {
    return false;
  }
  return fallback;
}

}  // namespace

double PiperSystem::rawToRadians(int32_t raw_thousandths_of_degree) {
  return static_cast<double>(raw_thousandths_of_degree) * kMilliDegToRad;
}

int32_t PiperSystem::radiansToRaw(double radians) {
  return static_cast<int32_t>(std::lround(radians * kRadToMilliDeg));
}

void PiperSystem::loadParameters() {
  const auto& parameters = info_.hardware_parameters;

  const auto stringParameter = [&parameters](const char* name,
                                             const std::string& fallback) {
    const auto found = parameters.find(name);
    return found == parameters.end() ? fallback : found->second;
  };

  can_interface_ = stringParameter("can_interface", "can0");
  enable_motors_on_activate_ =
      parseBoolean(stringParameter("enable_motors_on_activate", "true"), true);
  disable_motors_on_deactivate_ =
      parseBoolean(stringParameter("disable_motors_on_deactivate", "true"), true);
  clamp_to_protocol_limits_ =
      parseBoolean(stringParameter("clamp_to_protocol_limits", "true"), true);

  try {
    feedback_timeout_ = std::stod(stringParameter("feedback_timeout", "0.5"));
  } catch (const std::exception&) {
    feedback_timeout_ = 0.5;
  }
  try {
    command_speed_ratio_ = std::stoi(stringParameter("command_speed_ratio", "100"));
  } catch (const std::exception&) {
    command_speed_ratio_ = 100;
  }
  command_speed_ratio_ = std::clamp(command_speed_ratio_, 1, 100);
}

hardware_interface::CallbackReturn PiperSystem::on_init(
    const hardware_interface::HardwareInfo& info) {
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  loadParameters();

  bus_index_.fill(-1);
  declared_joint_count_ = info_.joints.size();
  if (declared_joint_count_ == 0 || declared_joint_count_ > kPiperJointCount) {
    RCLCPP_ERROR(rclcpp::get_logger(kLoggerName),
                 "Expected between 1 and %zu joints, the description declares %zu.",
                 kPiperJointCount, declared_joint_count_);
    return hardware_interface::CallbackReturn::ERROR;
  }

  for (std::size_t index = 0; index < declared_joint_count_; ++index) {
    const auto& joint = info_.joints[index];

    int bus_index = -1;
    for (std::size_t candidate = 0; candidate < kPiperJointCount; ++candidate) {
      if (joint.name == kJointNames[candidate]) {
        bus_index = static_cast<int>(candidate);
        break;
      }
    }
    if (bus_index < 0) {
      RCLCPP_ERROR(rclcpp::get_logger(kLoggerName),
                   "Joint '%s' is not part of the Piper arm (expected joint1..joint6).",
                   joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    bus_index_[static_cast<std::size_t>(bus_index)] = static_cast<int>(index);

    if (joint.state_interfaces.size() != 2 ||
        joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION ||
        joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY) {
      RCLCPP_ERROR(rclcpp::get_logger(kLoggerName),
                   "Joint '%s' must declare state interfaces [position, velocity].",
                   joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (joint.command_interfaces.size() != 1 ||
        joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
      RCLCPP_ERROR(rclcpp::get_logger(kLoggerName),
                   "Joint '%s' must declare the 'position' command interface.",
                   joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  state_position_.assign(declared_joint_count_, 0.0);
  state_velocity_.assign(declared_joint_count_, 0.0);
  command_position_.assign(declared_joint_count_, 0.0);
  measured_valid_.fill(false);
  feedback_healthy_ = false;
  feedback_warned_ = false;

  std::string undeclared;
  for (std::size_t bus = 0; bus < kPiperJointCount; ++bus) {
    if (bus_index_[bus] < 0) {
      if (!undeclared.empty()) {
        undeclared += ", ";
      }
      undeclared += kJointNames[bus];
    }
  }
  if (!undeclared.empty()) {
    RCLCPP_INFO(rclcpp::get_logger(kLoggerName),
                "Joints not declared by the description (%s) will be commanded to "
                "hold their measured position.",
                undeclared.c_str());
  }

  RCLCPP_INFO(rclcpp::get_logger(kLoggerName),
              "Configured '%s' on %s: enable_on_activate=%s, feedback_timeout=%.2f s, "
              "speed_ratio=%d%%.",
              info_.name.c_str(), can_interface_.c_str(),
              enable_motors_on_activate_ ? "true" : "false", feedback_timeout_,
              command_speed_ratio_);
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn PiperSystem::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  return hardware_interface::CallbackReturn::SUCCESS;
}

bool PiperSystem::startBus(std::string* error) {
  if (!can_.open(can_interface_, error)) {
    return false;
  }
  bus_open_ = true;
  last_feedback_time_ = std::chrono::steady_clock::time_point{};
  return true;
}

void PiperSystem::stopBus() {
  can_.close();
  bus_open_ = false;
  feedback_healthy_ = false;
  motor_enabled_feedback_.fill(false);
}

void PiperSystem::pollFeedback() {
  PiperCan::Frame frame;
  std::string error;
  while (can_.tryRead(&frame, &error)) {
    switch (frame.id) {
      case kCanIdJointFeedback12:
      case kCanIdJointFeedback34:
      case kCanIdJointFeedback56: {
        const std::size_t first = frame.id == kCanIdJointFeedback12 ? 0u
                                 : frame.id == kCanIdJointFeedback34 ? 2u : 4u;
        measured_radians_[first] = rawToRadians(readInt32(frame, 0));
        measured_radians_[first + 1] = rawToRadians(readInt32(frame, 4));
        measured_valid_[first] = true;
        measured_valid_[first + 1] = true;
        last_feedback_time_ = std::chrono::steady_clock::now();
        break;
      }
      case kCanIdStatusFeedback:
      case kCanIdGripperFeedback:
        // Status and gripper frames prove the arm is alive but carry no joint
        // angle, so they only refresh the liveness timestamp.
        last_feedback_time_ = std::chrono::steady_clock::now();
        break;
      default:
        if (frame.id >= kCanIdLowSpeedFeedbackFirst &&
            frame.id <= kCanIdLowSpeedFeedbackLast && frame.length >= 6) {
          const auto motor = static_cast<std::size_t>(
              frame.id - kCanIdLowSpeedFeedbackFirst);
          motor_enabled_feedback_[motor] =
              (frame.data[5] & kDriverEnableStatusBit) != 0;
          last_feedback_time_ = std::chrono::steady_clock::now();
        }
        break;
    }
  }
  if (!error.empty()) {
    last_error_ = error;
    RCLCPP_ERROR_THROTTLE(rclcpp::get_logger(kLoggerName), *clock_,
                          2000, "CAN read failed: %s", error.c_str());
  }
}

double PiperSystem::secondsSinceLastFeedback() const {
  if (last_feedback_time_.time_since_epoch().count() == 0) {
    return std::numeric_limits<double>::infinity();
  }
  return std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                       last_feedback_time_)
      .count();
}

bool PiperSystem::seedFromHardware(std::string* error) {
  const auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::duration<double>(kActivationFeedbackTimeoutSeconds);
  while (std::chrono::steady_clock::now() < deadline) {
    pollFeedback();
    if (std::all_of(measured_valid_.begin(), measured_valid_.end(),
                    [](bool valid) { return valid; })) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  *error = "no complete joint feedback from the arm within " +
           std::to_string(kActivationFeedbackTimeoutSeconds) +
           " s; check that the arm is powered and " + can_interface_ + " is up";
  return false;
}

bool PiperSystem::setMotorsEnabled(bool enabled, std::string* error) {
  PiperCan::Frame frame;
  frame.id = kCanIdMotorEnable;
  // Piper's decoder requires the fixed eight-byte control-frame DLC; unused
  // bytes remain zero from PiperCan::Frame's value initialization.
  frame.length = 8;
  frame.data[0] = kMotorAllJoints;
  frame.data[1] = enabled ? kEnableFlagEnable : kEnableFlagDisable;
  if (!can_.write(frame, error)) {
    return false;
  }
  if (!enabled) {
    motors_enabled_ = false;
  }
  return true;
}

bool PiperSystem::sendEmergencyStopResume(std::string* error) {
  PiperCan::Frame frame;
  frame.id = kCanIdMotionCtrl1;
  frame.length = 8;
  frame.data[0] = kEmergencyStopResume;
  return can_.write(frame, error);
}

bool PiperSystem::motorsAllEnabled() const {
  return std::all_of(motor_enabled_feedback_.begin(), motor_enabled_feedback_.end(),
                     [](bool enabled) { return enabled; });
}

bool PiperSystem::enableMotorsAndConfirm(
    const std::array<double, kPiperJointCount>& hold,
    std::string* error) {
  const auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::duration<double>(kMotorEnableTimeoutSeconds);
  while (std::chrono::steady_clock::now() < deadline) {
    // This firmware revision resets its active target to all-zero whenever an
    // enable request arrives. Restore MOVE_J and the measured-pose target
    // immediately after every request, before waiting for feedback.
    if (!setMotorsEnabled(true, error)) {
      return false;
    }
    if (!sendMotionMode(error) || !sendJointCommand(hold, error)) {
      return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    pollFeedback();
  }
  if (motorsAllEnabled()) {
    motors_enabled_ = true;
    return true;
  }
  *error = "the Piper firmware did not confirm all joint motors enabled within " +
           std::to_string(kMotorEnableTimeoutSeconds) + " s";
  return false;
}

bool PiperSystem::sendMotionMode(std::string* error) {
  PiperCan::Frame frame;
  frame.id = kCanIdMotionCtrl2;
  // Match the vendor SDK's zero-padded eight-byte 0x151 frame.
  frame.length = 8;
  frame.data[0] = kCtrlModeCanCommand;
  frame.data[1] = kMoveModeJoint;
  frame.data[2] = static_cast<uint8_t>(command_speed_ratio_);
  frame.data[3] = kMitModePositionVelocity;
  frame.data[4] = 0;  // Offline-trajectory hold time: unused in CAN mode.
  frame.data[5] = 0;  // Installation position: 0 = not configured.
  return can_.write(frame, error);
}

bool PiperSystem::sendJointCommand(
    const std::array<double, kPiperJointCount>& radians, std::string* error) {
  const uint32_t ids[3] = {kCanIdJointCtrl12, kCanIdJointCtrl34, kCanIdJointCtrl56};
  for (std::size_t frame_index = 0; frame_index < 3; ++frame_index) {
    PiperCan::Frame frame;
    frame.id = ids[frame_index];
    frame.length = 8;
    writeInt32(&frame, 0, radiansToRaw(radians[2 * frame_index]));
    writeInt32(&frame, 4, radiansToRaw(radians[2 * frame_index + 1]));
    if (!can_.write(frame, error)) {
      return false;
    }
  }
  return true;
}

hardware_interface::CallbackReturn PiperSystem::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  std::string error;
  if (!startBus(&error)) {
    RCLCPP_ERROR(rclcpp::get_logger(kLoggerName), "Failed to open CAN: %s",
                 error.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // The arm must report its real pose before any command is written. Otherwise
  // the first write() would drive the arm towards whatever the command buffer
  // happened to contain (a zero-initialised buffer means "go to zero").
  if (!seedFromHardware(&error)) {
    RCLCPP_ERROR(rclcpp::get_logger(kLoggerName), "Activation aborted: %s", error.c_str());
    stopBus();
    return hardware_interface::CallbackReturn::ERROR;
  }
  has_previous_measurement_ = false;
  for (std::size_t bus = 0; bus < kPiperJointCount; ++bus) {
    previous_measured_radians_[bus] = measured_radians_[bus];
  }

  for (std::size_t index = 0; index < declared_joint_count_; ++index) {
    state_position_[index] = 0.0;
    state_velocity_[index] = 0.0;
    command_position_[index] = 0.0;
  }
  for (std::size_t bus = 0; bus < kPiperJointCount; ++bus) {
    const int index = bus_index_[bus];
    if (index >= 0) {
      state_position_[static_cast<std::size_t>(index)] = measured_radians_[bus];
      // Seed the command with the measured position so the controller starts by
      // holding the arm exactly where it is.
      command_position_[static_cast<std::size_t>(index)] = measured_radians_[bus];
    }
  }

  const std::array<double, kPiperJointCount> hold = measured_radians_;

  RCLCPP_INFO(rclcpp::get_logger(kLoggerName),
              "Arm feedback received. Current joint angles (deg): %.3f %.3f %.3f %.3f %.3f %.3f",
              radiansToDegrees(measured_radians_[0]), radiansToDegrees(measured_radians_[1]),
              radiansToDegrees(measured_radians_[2]), radiansToDegrees(measured_radians_[3]),
              radiansToDegrees(measured_radians_[4]), radiansToDegrees(measured_radians_[5]));

  // A pose outside the documented limits means the model and the hardware
  // disagree. This is reported rather than corrected: it usually points at a
  // missing zero calibration, and silently shifting the frame would hide it.
  for (std::size_t bus = 0; bus < kPiperJointCount; ++bus) {
    if (measured_radians_[bus] < kJointMinRad[bus] - 1.0e-6 ||
        measured_radians_[bus] > kJointMaxRad[bus] + 1.0e-6) {
      RCLCPP_WARN(rclcpp::get_logger(kLoggerName),
                  "%s reads %.3f deg, outside the documented range [%.3f, %.3f] deg. "
                  "Holding is still allowed; check the arm zero calibration.",
                  kJointNames[bus], radiansToDegrees(measured_radians_[bus]),
                  radiansToDegrees(kJointMinRad[bus]), radiansToDegrees(kJointMaxRad[bus]));
    }
  }

  if (!sendEmergencyStopResume(&error)) {
    RCLCPP_ERROR(rclcpp::get_logger(kLoggerName),
                 "Failed to request emergency-stop resume: %s", error.c_str());
    stopBus();
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Preload the current-pose target before enabling. Some firmware revisions
  // immediately act on their default zero target as soon as the drives enable.
  if (!sendMotionMode(&error) || !sendJointCommand(hold, &error)) {
    RCLCPP_ERROR(rclcpp::get_logger(kLoggerName),
                 "Failed to preload the current-pose hold: %s", error.c_str());
    stopBus();
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (enable_motors_on_activate_) {
    if (!enableMotorsAndConfirm(hold, &error)) {
      RCLCPP_ERROR(rclcpp::get_logger(kLoggerName),
                   "Failed to enable and confirm the motors: %s", error.c_str());
      stopBus();
      return hardware_interface::CallbackReturn::ERROR;
    }
    RCLCPP_WARN(rclcpp::get_logger(kLoggerName),
                "Motors enabled: the arm now holds its position under power. "
                "Keep the emergency stop within reach.");
  } else {
    RCLCPP_WARN(rclcpp::get_logger(kLoggerName),
                "enable_motors_on_activate is false: commands will be sent but the "
                "arm stays limp.");
  }

  // The final enable request may reset the firmware's command mode. Restore
  // both mode and hold once more before ros2_control starts its update loop.
  if (!sendMotionMode(&error) || !sendJointCommand(hold, &error)) {
    RCLCPP_ERROR(rclcpp::get_logger(kLoggerName),
                 "Failed to send the initial hold command: %s", error.c_str());
    stopBus();
    return hardware_interface::CallbackReturn::ERROR;
  }

  feedback_healthy_ = true;
  feedback_warned_ = false;
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn PiperSystem::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  std::string error;
  if (bus_open_ && motors_enabled_ && disable_motors_on_deactivate_) {
    if (!setMotorsEnabled(false, &error)) {
      RCLCPP_WARN(rclcpp::get_logger(kLoggerName),
                  "Failed to disable the motors on deactivate: %s", error.c_str());
    } else {
      RCLCPP_WARN(rclcpp::get_logger(kLoggerName),
                  "Motors disabled: the arm is limp and will sag under gravity.");
    }
  }
  stopBus();
  motors_enabled_ = false;
  has_previous_measurement_ = false;
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> PiperSystem::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> interfaces;
  interfaces.reserve(2 * declared_joint_count_);
  for (std::size_t index = 0; index < declared_joint_count_; ++index) {
    const auto& name = info_.joints[index].name;
    interfaces.emplace_back(name, hardware_interface::HW_IF_POSITION,
                            &state_position_[index]);
    interfaces.emplace_back(name, hardware_interface::HW_IF_VELOCITY,
                            &state_velocity_[index]);
  }
  return interfaces;
}

std::vector<hardware_interface::CommandInterface>
PiperSystem::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> interfaces;
  interfaces.reserve(declared_joint_count_);
  for (std::size_t index = 0; index < declared_joint_count_; ++index) {
    interfaces.emplace_back(info_.joints[index].name,
                            hardware_interface::HW_IF_POSITION,
                            &command_position_[index]);
  }
  return interfaces;
}

hardware_interface::return_type PiperSystem::read(const rclcpp::Time& /*time*/,
                                                  const rclcpp::Duration& period) {
  if (!bus_open_) {
    return hardware_interface::return_type::ERROR;
  }

  pollFeedback();

  if (!feedback_healthy_) {
    return hardware_interface::return_type::ERROR;
  }

  const double since_feedback = secondsSinceLastFeedback();
  if (since_feedback > feedback_timeout_) {
    if (!feedback_warned_) {
      RCLCPP_ERROR(rclcpp::get_logger(kLoggerName),
                   "No arm feedback for %.3f s on %s; reporting a hardware failure so "
                   "the controllers stop commanding motion.",
                   since_feedback, can_interface_.c_str());
      feedback_warned_ = true;
    }
    return hardware_interface::return_type::ERROR;
  }
  feedback_warned_ = false;

  const double dt = period.seconds();
  for (std::size_t bus = 0; bus < kPiperJointCount; ++bus) {
    const int index = bus_index_[bus];
    if (index < 0) {
      continue;
    }
    const auto slot = static_cast<std::size_t>(index);
    const double position = measured_radians_[bus];
    double velocity = 0.0;
    if (has_previous_measurement_ && dt > 1.0e-9) {
      velocity = (position - previous_measured_radians_[bus]) / dt;
    }
    state_position_[slot] = position;
    state_velocity_[slot] = velocity;
  }
  for (std::size_t bus = 0; bus < kPiperJointCount; ++bus) {
    previous_measured_radians_[bus] = measured_radians_[bus];
  }
  has_previous_measurement_ = true;
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type PiperSystem::write(const rclcpp::Time& /*time*/,
                                                   const rclcpp::Duration& /*period*/) {
  if (!bus_open_) {
    return hardware_interface::return_type::ERROR;
  }

  std::array<double, kPiperJointCount> target = measured_radians_;

  for (std::size_t bus = 0; bus < kPiperJointCount; ++bus) {
    const int index = bus_index_[bus];
    if (index < 0) {
      // Undeclared joints stay at their last measured angle. This is what keeps
      // joint6 from drifting while the five-axis press configuration runs.
      continue;
    }
    double value = command_position_[static_cast<std::size_t>(index)];
    if (!std::isfinite(value)) {
      RCLCPP_ERROR_THROTTLE(rclcpp::get_logger(kLoggerName),
                            *clock_, 1000,
                            "Non-finite command for %s; holding the measured angle.",
                            kJointNames[bus]);
      value = measured_radians_[bus];
    }
    if (clamp_to_protocol_limits_) {
      // A live arm was observed reporting joint2 = -2.3 deg against a documented
      // lower bound of 0 deg, so the measured pose can legitimately sit outside
      // the protocol range. Holding that pose must never be clamped, otherwise
      // activation itself would command a jump. The window is therefore widened
      // to include the measured angle; commands beyond both windows are still
      // cut off.
      const double low = std::min(kJointMinRad[bus], measured_radians_[bus]);
      const double high = std::max(kJointMaxRad[bus], measured_radians_[bus]);
      const double clamped = std::clamp(value, low, high);
      if (std::abs(clamped - value) > 1.0e-6) {
        RCLCPP_WARN_THROTTLE(rclcpp::get_logger(kLoggerName), *clock_, 2000,
                             "%s command %.4f rad is outside the arm limit [%.4f, %.4f]; "
                             "clamped.",
                             kJointNames[bus], value, low, high);
        value = clamped;
      }
    }
    target[bus] = value;
  }

  std::string error;

  // The mode frame is resent every cycle on purpose. The vendor's own control
  // node issues MotionCtrl_2 immediately before every JointCtrl, and a live arm
  // reproduced the reason: with a single 0x151 at activation, the arm reported
  // CAN_CTRL, reported all motors enabled, held its position under power, and
  // still ignored joint commands entirely. Repeating the mode frame makes the
  // commands take effect.
  if (!sendMotionMode(&error)) {
    last_error_ = error;
    RCLCPP_ERROR_THROTTLE(rclcpp::get_logger(kLoggerName), *clock_, 2000,
                          "CAN mode write failed: %s", error.c_str());
    return hardware_interface::return_type::ERROR;
  }

  if (!sendJointCommand(target, &error)) {
    last_error_ = error;
    RCLCPP_ERROR_THROTTLE(rclcpp::get_logger(kLoggerName), *clock_, 2000,
                          "CAN write failed: %s", error.c_str());
    return hardware_interface::return_type::ERROR;
  }
  return hardware_interface::return_type::OK;
}

}  // namespace piper_hardware

PLUGINLIB_EXPORT_CLASS(piper_hardware::PiperSystem, hardware_interface::SystemInterface)
