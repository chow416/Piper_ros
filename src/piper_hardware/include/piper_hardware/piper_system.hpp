#pragma once

#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include <array>
#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "piper_hardware/piper_can.hpp"

namespace piper_hardware {

// ros2_control hardware interface for a physical Piper arm on SocketCAN.
//
// The arm always reports and accepts six joint angles, but this interface only
// exports the joints declared in the ros2_control block of the robot
// description. Joints that are not declared are not left free: the driver keeps
// commanding their last measured angle, which holds them still. That is what
// keeps the six-axis arm consistent with the five-axis press configuration,
// where joint6 is replaced by the rigid press tool.
class PiperSystem final : public hardware_interface::SystemInterface {
 public:
  static constexpr std::size_t kPiperJointCount = 6;

  // Piper protocol joint order, indexed 0..5 for joint1..joint6.
  static constexpr std::array<const char*, kPiperJointCount> kJointNames{
      "joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};

  // Hard limits reported by the vendor SDK, in radians. They are used as a last
  // resort clamp in write() so that a bad trajectory cannot be forwarded to the
  // motors even if the URDF bounds are wrong.
  static constexpr std::array<double, kPiperJointCount> kJointMinRad{
      -2.6179, 0.0, -2.967, -1.745, -1.22, -2.09439};
  static constexpr std::array<double, kPiperJointCount> kJointMaxRad{
      2.6179, 3.14, 0.0, 1.745, 1.22, 2.09439};

  hardware_interface::CallbackReturn on_init(
      const hardware_interface::HardwareInfo& info) override;

  hardware_interface::CallbackReturn on_configure(
      const rclcpp_lifecycle::State& previous_state) override;

  hardware_interface::CallbackReturn on_activate(
      const rclcpp_lifecycle::State& previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State& previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces()
      override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces()
      override;

  hardware_interface::return_type read(const rclcpp::Time& time,
                                       const rclcpp::Duration& period) override;

  hardware_interface::return_type write(const rclcpp::Time& time,
                                        const rclcpp::Duration& period) override;

  // Exposed for the unit tests and for the standalone bring-up tooling.
  [[nodiscard]] bool feedbackHealthy() const { return feedback_healthy_; }
  [[nodiscard]] double secondsSinceLastFeedback() const;
  [[nodiscard]] const std::string& lastError() const { return last_error_; }

 private:
  // Configuration read from the <hardware> parameters of the robot description.
  void loadParameters();
  bool startBus(std::string* error);
  void stopBus();

  bool seedFromHardware(std::string* error);
  bool setMotorsEnabled(bool enabled, std::string* error);
  bool sendEmergencyStopResume(std::string* error);
  bool enableMotorsAndConfirm(
      const std::array<double, kPiperJointCount>& hold,
      std::string* error);
  [[nodiscard]] bool motorsAllEnabled() const;
  bool sendMotionMode(std::string* error);
  bool sendJointCommand(const std::array<double, kPiperJointCount>& radians,
                        std::string* error);

  // Drains every pending CAN frame and refreshes the cached feedback.
  void pollFeedback();

  static double rawToRadians(int32_t raw_thousandths_of_degree);
  static int32_t radiansToRaw(double radians);

  // Configuration.
  std::string can_interface_{"can0"};
  bool enable_motors_on_activate_{true};
  bool disable_motors_on_deactivate_{true};
  double feedback_timeout_{0.5};
  int command_speed_ratio_{100};
  bool clamp_to_protocol_limits_{true};

  // Joint bookkeeping: bus_index_ maps a Piper protocol joint to its position in
  // the declared joint list, or -1 when the description does not declare it.
  std::array<int, kPiperJointCount> bus_index_{};
  std::size_t declared_joint_count_{0};

  // Interface storage owned by the handles exported to the controller manager.
  std::vector<double> state_position_;
  std::vector<double> state_velocity_;
  std::vector<double> command_position_;

  // Cached arm feedback, always indexed by Piper protocol joint order.
  std::array<double, kPiperJointCount> measured_radians_{};
  std::array<double, kPiperJointCount> previous_measured_radians_{};
  std::array<bool, kPiperJointCount> measured_valid_{};
  std::array<bool, kPiperJointCount> motor_enabled_feedback_{};
  bool has_previous_measurement_{false};

  PiperCan can_;
  bool bus_open_{false};
  bool motors_enabled_{false};
  bool feedback_healthy_{false};
  bool feedback_warned_{false};
  std::chrono::steady_clock::time_point last_feedback_time_{};
  std::string last_error_;
  // 发送队列一时满（SocketCAN 返回 ENOBUFS/EAGAIN）是正常的，但不能一直满。
  // 连续这么多个控制周期都发不出去才算真故障。
  int transient_write_failures_{0};
  static constexpr int kMaxTransientWriteFailures = 40;

  // A steady clock is used for the throttled diagnostics because ROS time can
  // be paused or jumped, which would break throttling on a hardware driver.
  rclcpp::Clock::SharedPtr clock_{std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME)};
};

}  // namespace piper_hardware
