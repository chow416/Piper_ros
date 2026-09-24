#!/usr/bin/env python3
"""Publish one press target for the Piper button press task.

This is the reference publisher for the interface documented in
`docs/perception_interface.md`. It exists because the task needs a point and a
press direction, while the wire format carries a point and a quaternion, and
building that quaternion by hand is the easiest step to get wrong.

    ros2 run piper_elevator_manipulation send_press_target.py \\
        --point 0.300,0.000,0.240 --direction 1,0,0

The direction becomes the +Z axis of the published orientation, which is what
the task reads. Any vector works, it does not have to be a unit vector.

WHY IT PUBLISHES THE TARGET SEVERAL TIMES
-----------------------------------------
A press target is a discrete command sent over a one-way topic. If a sample is
lost, nothing tells the sender: there is no acknowledgement, and the task just
keeps waiting. That is exactly how "I sent it and nothing happened" happens.

Measured on 2026-09-24: six single-shot sends over the same interface reached
the task node three times. Late discovery, a subscription queue of depth 1, and
the task discarding whatever arrived while the arm was moving can all drop a
one-off sample.

So the default is three publishes 0.3 s apart. Repeating is harmless: the task
takes the first one, and any that arrive during the press are discarded before
it waits again, so this still produces exactly one press.
"""

import argparse
import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

DEFAULT_TOPIC = "/press/target_pose"
DEFAULT_FRAME = "world"


def normalise(vector):
    norm = math.sqrt(sum(component * component for component in vector))
    if norm < 1.0e-9:
        raise ValueError("direction vector is zero")
    return [component / norm for component in vector]


def cross(left, right):
    return [
        left[1] * right[2] - left[2] * right[1],
        left[2] * right[0] - left[0] * right[2],
        left[0] * right[1] - left[1] * right[0],
    ]


def rotation_with_z_axis(direction):
    """Build a rotation matrix whose third column is `direction`.

    The remaining two axes are unconstrained, so any valid completion will do:
    this picks one deterministically so repeated runs publish the same target.
    """
    # Any axis that is not parallel to the direction works as the first guess.
    helper = [1.0, 0.0, 0.0] if abs(direction[0]) < 0.9 else [0.0, 1.0, 0.0]
    x_axis = normalise(cross(helper, direction))
    y_axis = cross(direction, x_axis)
    return [
        [x_axis[0], y_axis[0], direction[0]],
        [x_axis[1], y_axis[1], direction[1]],
        [x_axis[2], y_axis[2], direction[2]],
    ]


def quaternion_from_matrix(matrix):
    """Convert a rotation matrix to (x, y, z, w), Shepperd's method."""
    m = matrix
    trace = m[0][0] + m[1][1] + m[2][2]
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (m[2][1] - m[1][2]) / s
        y = (m[0][2] - m[2][0]) / s
        z = (m[1][0] - m[0][1]) / s
    elif m[0][0] > m[1][1] and m[0][0] > m[2][2]:
        s = math.sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2.0
        w = (m[2][1] - m[1][2]) / s
        x = 0.25 * s
        y = (m[0][1] + m[1][0]) / s
        z = (m[0][2] + m[2][0]) / s
    elif m[1][1] > m[2][2]:
        s = math.sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2.0
        w = (m[0][2] - m[2][0]) / s
        x = (m[0][1] + m[1][0]) / s
        y = 0.25 * s
        z = (m[1][2] + m[2][1]) / s
    else:
        s = math.sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2.0
        w = (m[1][0] - m[0][1]) / s
        x = (m[0][2] + m[2][0]) / s
        y = (m[1][2] + m[2][1]) / s
        z = 0.25 * s
    return (x, y, z, w)


def parse_triple(text, label):
    parts = text.replace(" ", "").split(",")
    if len(parts) != 3:
        raise argparse.ArgumentTypeError(
            f"{label} must be three comma-separated numbers, got '{text}'"
        )
    try:
        values = [float(part) for part in parts]
    except ValueError as error:
        raise argparse.ArgumentTypeError(
            f"{label} must contain only numbers, got '{text}'"
        ) from error
    if not all(math.isfinite(value) for value in values):
        raise argparse.ArgumentTypeError(f"{label} must be finite, got '{text}'")
    return values


class TargetPublisher(Node):
    def __init__(self, topic, frame, point, direction):
        super().__init__("press_target_publisher")
        self._frame = frame
        self._point = point
        direction = normalise(direction)
        matrix = rotation_with_z_axis(direction)
        self._quaternion = quaternion_from_matrix(matrix)

        # Cross-check the construction: rotating +Z by this quaternion must
        # return the requested direction. Kept in the tool because getting this
        # wrong silently aims the arm somewhere else.
        axis = self._quaternion_to_z_axis(self._quaternion)
        error = max(abs(axis[i] - direction[i]) for i in range(3))
        if error > 1.0e-6:
            raise RuntimeError(
                f"internal error: quaternion maps +Z to {axis}, not {direction}"
            )
        self._direction = direction

        # Matches the subscription in the task: reliable, shallow, volatile.
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._publisher = self.create_publisher(PoseStamped, topic, qos)

    @staticmethod
    def _quaternion_to_z_axis(quaternion):
        x, y, z, w = quaternion
        # Third column of the rotation matrix, written out to avoid a
        # dependency on a transform library.
        return [
            2.0 * (x * z + w * y),
            2.0 * (y * z - w * x),
            1.0 - 2.0 * (x * x + y * y),
        ]

    def wait_for_subscriber(self, timeout_sec):
        """Block until the task node is actually listening.

        A single publish loses the message when DDS discovery has not finished
        yet: the publisher has no matched reader, so the sample is dropped and
        the sender never notices. That is the whole reason `--count 1` can look
        like it did nothing, so wait for a match before the first publish.
        """
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            if self._publisher.get_subscription_count() > 0:
                return True
            rclpy.spin_once(self, timeout_sec=0.05)
        return self._publisher.get_subscription_count() > 0

    def publish_once(self):
        message = PoseStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self._frame
        message.pose.position.x = self._point[0]
        message.pose.position.y = self._point[1]
        message.pose.position.z = self._point[2]
        (
            message.pose.orientation.x,
            message.pose.orientation.y,
            message.pose.orientation.z,
            message.pose.orientation.w,
        ) = self._quaternion
        self._publisher.publish(message)


def main():
    parser = argparse.ArgumentParser(
        description="Publish a press target as geometry_msgs/PoseStamped.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "--point",
        required=True,
        type=lambda text: parse_triple(text, "--point"),
        help="press rod tip target as x,y,z in metres",
    )
    parser.add_argument(
        "--direction",
        required=True,
        type=lambda text: parse_triple(text, "--direction"),
        help="press direction, pointing into the button; becomes the pose +Z axis",
    )
    parser.add_argument("--frame", default=DEFAULT_FRAME, help="header.frame_id")
    parser.add_argument("--topic", default=DEFAULT_TOPIC, help="target topic")
    parser.add_argument(
        "--count", type=int, default=3,
        help=(
            "how many times to publish; 0 repeats forever. Defaults to 3 on "
            "purpose, see the note on delivery in the module docstring"
        ),
    )
    parser.add_argument(
        "--period", type=float, default=0.3, help="seconds between messages"
    )
    parser.add_argument(
        "--subscriber-timeout", type=float, default=15.0,
        help=(
            "wait this long for the press task to subscribe before publishing; "
            "a message sent earlier is dropped without any error"
        ),
    )
    arguments = parser.parse_args()

    if arguments.count < 0:
        parser.error("--count must be zero or positive")
    if arguments.period <= 0.0:
        parser.error("--period must be positive")

    rclpy.init()
    try:
        node = TargetPublisher(
            arguments.topic, arguments.frame, arguments.point, arguments.direction
        )
    except (ValueError, RuntimeError) as error:
        print(f"error: {error}", file=sys.stderr)
        rclpy.shutdown()
        return 2

    print(f"topic      : {arguments.topic}")
    print(f"frame      : {arguments.frame}")
    print(f"point      : {arguments.point[0]:.4f}, {arguments.point[1]:.4f}, "
          f"{arguments.point[2]:.4f} m")
    print(f"direction  : {node._direction[0]:+.4f}, {node._direction[1]:+.4f}, "
          f"{node._direction[2]:+.4f}")
    print(f"quaternion : {node._quaternion[0]:+.6f}, {node._quaternion[1]:+.6f}, "
          f"{node._quaternion[2]:+.6f}, {node._quaternion[3]:+.6f}  (x, y, z, w)")

    print(f"waiting up to {arguments.subscriber_timeout:.0f} s for a subscriber on "
          f"{arguments.topic} ...")
    if not node.wait_for_subscriber(arguments.subscriber_timeout):
        print(f"error: nothing subscribed to {arguments.topic}. The message would "
              "be dropped silently.", file=sys.stderr)
        print("  Is the press task running? Start it and wait until it prints",
              file=sys.stderr)
        print("    Waiting for a press target on '/press/target_pose'",
              file=sys.stderr)
        print("  then run this command again.", file=sys.stderr)
        node.destroy_node()
        rclpy.shutdown()
        return 3
    print("subscriber found, publishing")
    if arguments.count != 1:
        print(f"publishing {arguments.count} copies "
              f"({arguments.period:.2f} s apart) so one lost sample cannot stall "
              "the task")

    remaining = arguments.count
    try:
        while rclpy.ok():
            node.publish_once()
            rclpy.spin_once(node, timeout_sec=0.0)
            if arguments.count != 0:
                remaining -= 1
                if remaining <= 0:
                    break
                print(f"published, {remaining} to go")
            else:
                print("published")
            time.sleep(arguments.period)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
