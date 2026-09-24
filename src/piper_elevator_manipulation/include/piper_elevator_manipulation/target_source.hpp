#pragma once

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

#include <Eigen/Core>

#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

namespace piper_elevator_manipulation {

// A target that survived validation, already converted into planning terms.
struct PressTarget {
  Eigen::Vector3d position;         // press rod tip, planning frame, metres
  Eigen::Vector3d press_direction;  // unit vector, rod axis, points into the surface
  rclcpp::Time stamp;               // stamp of the accepted observation
  std::string frame_id;             // frame as declared by the publisher
};

struct TargetSourceOptions {
  std::string topic{"/press/target_pose"};
  // Frames we can accept without a transform. "world" is the planning frame and
  // "base_link" is the same physical frame: the URDF links them with a fixed
  // joint that has no origin, so the transform is the identity.
  std::vector<std::string> accepted_frames{"world", "base_link"};
  // Rejects a target that was observed too long ago, which matters because a
  // latched publisher can replay an old observation to a late subscriber.
  std::chrono::milliseconds max_age{1000};
  // Sanity envelope, not a collision check. Keeps an obviously wrong point out
  // of the planner and out of a trajectory.
  Eigen::Vector3d workspace_min{-0.30, -0.50, -0.02};
  Eigen::Vector3d workspace_max{0.65, 0.50, 0.80};
  // +1 takes the pose orientation's +Z axis as the press direction, which is
  // what the tool model means by "forward". -1 flips it, for publishers that
  // compute an outward surface normal instead.
  double direction_sign{1.0};
};

// Target input for the press task.
//
// The task can take its target from launch parameters (a fixed point on the
// bench) or from a topic, which is the interface the perception layer uses.
// Every way the topic path can go wrong is checked here and rejected with a
// reason, so the planner never sees a target that is stale, expressed in a
// frame we cannot transform from, numerically broken, or outside the declared
// workspace. Rejections are counted and the last reason is kept, because
// "nothing happened" is the worst possible failure mode for an interface.
class TargetSource {
 public:
  TargetSource(const rclcpp::Node::SharedPtr& node, TargetSourceOptions options);

  TargetSource(const TargetSource&) = delete;
  TargetSource& operator=(const TargetSource&) = delete;

  // Blocks until a valid target arrives and consumes it. The node must be
  // spinning on another thread for this to make progress.
  std::optional<PressTarget> waitForTarget(std::chrono::milliseconds timeout);

  // Forgets the target currently held. Called at the start of a cycle so the
  // message that produced this cycle cannot also produce the next one.
  void discardPendingTarget();

  std::size_t rejectedCount() const;
  std::string lastRejectionReason() const;
  const std::string& topic() const;

 private:
  void onMessage(const geometry_msgs::msg::PoseStamped::ConstSharedPtr& message);
  bool validate(const geometry_msgs::msg::PoseStamped& message,
                PressTarget* target, std::string* reason) const;

  rclcpp::Node::SharedPtr node_;
  TargetSourceOptions options_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_;

  mutable std::mutex mutex_;
  std::condition_variable condition_;
  std::optional<PressTarget> pending_;
  std::size_t rejected_count_{0};
  std::string last_rejection_;
  // Used only to keep the accept log readable when the publisher runs faster
  // than the task; not part of the validation state.
  std::optional<Eigen::Vector3d> last_logged_position_;
  std::optional<Eigen::Vector3d> last_logged_direction_;
};

}  // namespace piper_elevator_manipulation
