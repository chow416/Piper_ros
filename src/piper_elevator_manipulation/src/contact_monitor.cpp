#include "piper_elevator_manipulation/contact_monitor.hpp"

#include <stdexcept>

namespace piper_elevator_manipulation {

ContactMonitor::ContactMonitor(const rclcpp::Node::SharedPtr& node,
                               const std::string& topic,
                               std::size_t stable_samples,
                               std::chrono::milliseconds max_age)
    : stable_samples_(stable_samples), max_age_(max_age) {
  if (topic.empty() || stable_samples == 0 || max_age.count() <= 0) {
    throw std::invalid_argument("Invalid contact monitor parameters");
  }
  subscription_ = node->create_subscription<std_msgs::msg::Bool>(
      topic, rclcpp::SensorDataQoS(),
      [this](std_msgs::msg::Bool::ConstSharedPtr message) {
        std::lock_guard<std::mutex> lock(mutex_);
        last_sample_ = std::chrono::steady_clock::now();
        latest_contact_ = message->data;
        consecutive_contact_ = message->data ? consecutive_contact_ + 1 : 0;
      });
}

void ContactMonitor::reset() {
  std::lock_guard<std::mutex> lock(mutex_);
  consecutive_contact_ = 0;
}

bool ContactMonitor::isFresh() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return last_sample_ != std::chrono::steady_clock::time_point{} &&
         std::chrono::steady_clock::now() - last_sample_ <= max_age_;
}

bool ContactMonitor::isContactActive() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return last_sample_ != std::chrono::steady_clock::time_point{} &&
         std::chrono::steady_clock::now() - last_sample_ <= max_age_ &&
         latest_contact_;
}

bool ContactMonitor::hasStableContact() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return last_sample_ != std::chrono::steady_clock::time_point{} &&
         std::chrono::steady_clock::now() - last_sample_ <= max_age_ &&
         consecutive_contact_ >= stable_samples_;
}

}  // namespace piper_elevator_manipulation
