"""Publish validated J1–J5 positions from the real Piper CAN bus.

This module has no command subscribers and never calls arm enable or motion APIs.
It deliberately skips SDK initialization queries to keep bring-up passive.
"""

import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from .feedback import JOINT_NAMES, InvalidFeedback, decode_joint_feedback


class JointStateReader(Node):
    def __init__(self):
        super().__init__('piper_press_joint_state_reader')
        self.declare_parameter('can_port', 'can0')
        self.declare_parameter('joint_state_topic', 'hardware_joint_states')
        self.declare_parameter('publish_rate_hz', 20.0)
        self.declare_parameter('max_feedback_age_sec', 0.25)
        self.declare_parameter('max_abs_joint_angle_rad', 3.5)

        can_port = self.get_parameter('can_port').value
        topic = self.get_parameter('joint_state_topic').value
        publish_rate = float(self.get_parameter('publish_rate_hz').value)
        self._max_age = float(self.get_parameter('max_feedback_age_sec').value)
        self._max_angle = float(self.get_parameter('max_abs_joint_angle_rad').value)
        if not can_port or not topic:
            raise ValueError('can_port and joint_state_topic must not be empty')
        if not 1.0 <= publish_rate <= 100.0:
            raise ValueError('publish_rate_hz must be between 1 and 100')
        if not 0.01 <= self._max_age <= 2.0:
            raise ValueError('max_feedback_age_sec must be between 0.01 and 2')
        if not 0.1 <= self._max_angle <= 6.3:
            raise ValueError('max_abs_joint_angle_rad must be between 0.1 and 6.3')

        # Import only when the node starts; pure feedback tests need no SDK.
        from piper_sdk import C_PiperInterface

        self._sdk = C_PiperInterface(can_name=can_port)
        self._sdk.ConnectPort(piper_init=False)
        self._publisher = self.create_publisher(JointState, topic, 10)
        self._healthy = None
        self.create_timer(1.0 / publish_rate, self._publish_if_fresh)
        self.get_logger().info(
            f'Read-only CAN feedback on {can_port} -> {topic}; J1-J5 only, '
            'no enable or motion commands')

    def _set_health(self, healthy: bool, reason: str = ''):
        if healthy == self._healthy:
            return
        self._healthy = healthy
        if healthy:
            self.get_logger().info('Fresh Piper joint feedback received')
        else:
            self.get_logger().warn(f'Joint state publication paused: {reason}')

    def _publish_if_fresh(self):
        if not self._sdk.isOk():
            self._set_health(False, 'SDK CAN receive thread is not healthy')
            return

        try:
            sample = decode_joint_feedback(
                self._sdk.GetArmJointMsgs(),
                now_sec=time.time(),
                max_age_sec=self._max_age,
                max_abs_angle_rad=self._max_angle,
            )
        except (InvalidFeedback, AttributeError, TypeError, ValueError) as error:
            self._set_health(False, str(error))
            return

        message = JointState()
        stamp_seconds = int(sample.timestamp_sec)
        message.header.stamp.sec = stamp_seconds
        message.header.stamp.nanosec = int(
            (sample.timestamp_sec - stamp_seconds) * 1_000_000_000)
        message.name = list(JOINT_NAMES)
        message.position = list(sample.positions_rad)
        # Velocity and effort are intentionally empty until their SDK units
        # and hardware meaning have been validated independently.
        self._publisher.publish(message)
        self._set_health(True)

    def close(self):
        self._sdk.DisconnectPort()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = JointStateReader()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        rclpy.shutdown()
