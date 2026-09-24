#include "piper_elevator_manipulation/target_source.hpp"

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <utility>

namespace piper_elevator_manipulation {
namespace {

constexpr double kMinQuaternionNorm = 0.5;
constexpr double kQuaternionNormTolerance = 1.0e-3;
// A stamp this far in the future means the publisher and this node disagree
// about the clock. Warn but keep going when it is small; the freshness check
// below is the one that decides.
constexpr double kClockSkewToleranceSeconds = 0.5;

}  // namespace

TargetSource::TargetSource(const rclcpp::Node::SharedPtr& node,
                           TargetSourceOptions options)
    : node_(node), options_(std::move(options)) {
  // Reliable and shallow: a target is a discrete command, not a stream, and
  // losing the newest one costs a press. Depth 1 is enough because only the
  // most recent observation is ever useful.
  subscription_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
      options_.topic, rclcpp::QoS(rclcpp::KeepLast(1)).reliable(),
      [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr message) {
        onMessage(message);
      });
  std::string accepted_frames;
  for (const auto& frame : options_.accepted_frames) {
    accepted_frames += (accepted_frames.empty() ? "" : ", ") + frame;
  }
  RCLCPP_INFO(node_->get_logger(),
              "Target source: topic '%s', accepted frames %s, max age %.3f s, "
              "direction sign %+.0f.",
              options_.topic.c_str(),
              accepted_frames.empty() ? "(any)" : accepted_frames.c_str(),
              static_cast<double>(options_.max_age.count()) / 1000.0,
              options_.direction_sign);
}

void TargetSource::onMessage(
    const geometry_msgs::msg::PoseStamped::ConstSharedPtr& message) {
  PressTarget target;
  std::string reason;
  if (!validate(*message, &target, &reason)) {
    std::size_t rejected = 0;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      last_rejection_ = reason;
      rejected = ++rejected_count_;
    }
    // Only the first rejection is reported, so a 5 Hz publisher cannot flood
    // the log while the cause stays visible.
    if (rejected == 1) {
      RCLCPP_WARN(node_->get_logger(),
                  "Rejected a target on '%s': %s. Further rejections are "
                  "counted and reported at the end of the run.",
                  options_.topic.c_str(), reason.c_str());
    } else {
      RCLCPP_DEBUG(node_->get_logger(), "Rejected a target: %s", reason.c_str());
    }
    return;
  }

  // A 5 Hz detector would otherwise print five identical lines per second. The
  // first target and every target that actually moved are worth seeing; the
  // rest stay at debug level.
  const bool moved =
      !last_logged_position_.has_value() ||
      (target.position - *last_logged_position_).norm() > 0.001 ||
      (target.press_direction - *last_logged_direction_).norm() > 0.017;
  last_logged_position_ = target.position;
  last_logged_direction_ = target.press_direction;

  if (moved) {
    RCLCPP_INFO(node_->get_logger(),
                "Accepted target in '%s': press rod tip at (%.4f, %.4f, %.4f), "
                "press direction (%+.3f, %+.3f, %+.3f).",
                target.frame_id.c_str(), target.position.x(), target.position.y(),
                target.position.z(), target.press_direction.x(),
                target.press_direction.y(), target.press_direction.z());
  } else {
    RCLCPP_DEBUG(node_->get_logger(),
                 "Accepted an unchanged target at (%.4f, %.4f, %.4f).",
                 target.position.x(), target.position.y(), target.position.z());
  }

  std::lock_guard<std::mutex> lock(mutex_);
  pending_ = target;
  condition_.notify_all();
}

bool TargetSource::validate(const geometry_msgs::msg::PoseStamped& message,
                            PressTarget* target, std::string* reason) const {
  const std::string& frame_id = message.header.frame_id;
  if (frame_id.empty()) {
    *reason = "header.frame_id is empty; publish in 'world' or 'base_link'";
    return false;
  }
  if (!options_.accepted_frames.empty() &&
      std::find(options_.accepted_frames.begin(), options_.accepted_frames.end(),
                frame_id) == options_.accepted_frames.end()) {
    *reason = "unknown frame '" + frame_id + "'; no transform path is available "
              "yet, so publish in 'world' or 'base_link'";
    return false;
  }

  const rclcpp::Time stamp(message.header.stamp);
  if (stamp.nanoseconds() == 0) {
    *reason = "header.stamp is zero; set it to the observation time";
    return false;
  }
  const rclcpp::Time now = node_->now();
  const double age_seconds = (now - stamp).seconds();
  if (age_seconds > static_cast<double>(options_.max_age.count()) / 1000.0) {
    *reason = "target is " + std::to_string(age_seconds) +
              " s old, older than the accepted maximum; publish it again";
    return false;
  }
  if (age_seconds < -kClockSkewToleranceSeconds) {
    RCLCPP_WARN(node_->get_logger(),
                "Target stamp is %.3f s in the future; check the clocks.",
                -age_seconds);
  }

  const auto& position = message.pose.position;
  if (!std::isfinite(position.x) || !std::isfinite(position.y) ||
      !std::isfinite(position.z)) {
    *reason = "position contains a non-finite number";
    return false;
  }

  const auto& orientation = message.pose.orientation;
  if (!std::isfinite(orientation.x) || !std::isfinite(orientation.y) ||
      !std::isfinite(orientation.z) || !std::isfinite(orientation.w)) {
    *reason = "orientation contains a non-finite number";
    return false;
  }
  const double quaternion_norm =
      std::sqrt(orientation.x * orientation.x + orientation.y * orientation.y +
                orientation.z * orientation.z + orientation.w * orientation.w);
  if (quaternion_norm < kMinQuaternionNorm) {
    *reason = "orientation is not a rotation; set the quaternion so that its "
              "+Z axis is the press direction";
    return false;
  }
  if (std::abs(quaternion_norm - 1.0) > kQuaternionNormTolerance) {
    RCLCPP_WARN(node_->get_logger(),
                "Target quaternion norm is %.6f; normalising it. Publishers "
                "should send a unit quaternion.",
                quaternion_norm);
  }

  const Eigen::Quaterniond rotation(orientation.w, orientation.x,
                                    orientation.y, orientation.z);
  // The tool model defines +Z as the forward, pressing axis, so the pose
  // orientation carries the direction without a separate field.
  const Eigen::Vector3d direction =
      options_.direction_sign * (rotation.normalized() * Eigen::Vector3d::UnitZ());
  if (!direction.allFinite() || direction.norm() < 1.0e-6) {
    *reason = "the orientation's +Z axis is degenerate";
    return false;
  }

  const Eigen::Vector3d point(position.x, position.y, position.z);
  if (!point.allFinite()) {
    *reason = "position is not finite";
    return false;
  }
  for (int axis = 0; axis < 3; ++axis) {
    if (point[axis] < options_.workspace_min[axis] ||
        point[axis] > options_.workspace_max[axis]) {
      *reason = "point (" + std::to_string(point.x()) + ", " +
                std::to_string(point.y()) + ", " + std::to_string(point.z()) +
                ") is outside the accepted workspace box [" +
                std::to_string(options_.workspace_min.x()) + ", " +
                std::to_string(options_.workspace_min.y()) + ", " +
                std::to_string(options_.workspace_min.z()) + "] to [" +
                std::to_string(options_.workspace_max.x()) + ", " +
                std::to_string(options_.workspace_max.y()) + ", " +
                std::to_string(options_.workspace_max.z()) + "]";
      return false;
    }
  }

  target->position = point;
  target->press_direction = direction.normalized();
  target->stamp = stamp;
  target->frame_id = frame_id;
  return true;
}

std::optional<PressTarget> TargetSource::waitForTarget(
    std::chrono::milliseconds timeout) {
  std::unique_lock<std::mutex> lock(mutex_);
  if (!condition_.wait_for(lock, timeout,
                           [this] { return pending_.has_value(); })) {
    return std::nullopt;
  }
  std::optional<PressTarget> target = pending_;
  pending_.reset();
  return target;
}

void TargetSource::discardPendingTarget() {
  std::lock_guard<std::mutex> lock(mutex_);
  pending_.reset();
}

std::size_t TargetSource::rejectedCount() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return rejected_count_;
}

std::string TargetSource::lastRejectionReason() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return last_rejection_;
}

const std::string& TargetSource::topic() const { return options_.topic; }

}  // namespace piper_elevator_manipulation
