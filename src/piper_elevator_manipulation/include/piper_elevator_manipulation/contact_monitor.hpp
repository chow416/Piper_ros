#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

#include <chrono>
#include <cstddef>
#include <memory>
#include <mutex>

namespace piper_elevator_manipulation {

// Source-independent contact signal. A simulation adapter and a future real
// force/current/switch adapter can publish the same topic. False or stale
// readings never count as successful contact.
class ContactMonitor {
 public:
  ContactMonitor(const rclcpp::Node::SharedPtr& node,
                 const std::string& topic,
                 std::size_t stable_samples,
                 std::chrono::milliseconds max_age);

  void reset();
  bool isFresh() const;
  bool isContactActive() const;
  bool hasStableContact() const;

 private:
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subscription_;
  std::size_t stable_samples_;
  std::chrono::milliseconds max_age_;
  mutable std::mutex mutex_;
  std::size_t consecutive_contact_{0};
  bool latest_contact_{false};
  std::chrono::steady_clock::time_point last_sample_{};
};

}  // namespace piper_elevator_manipulation
