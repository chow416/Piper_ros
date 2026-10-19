#!/usr/bin/env python3
"""ROS-topic contract test for the semantic button target selector."""

import importlib.util
import time
import unittest
import uuid
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from vision_msgs.msg import Detection3D, Detection3DArray, ObjectHypothesisWithPose


SCRIPT = Path(__file__).parents[1] / "scripts" / "button_target_selector.py"


def load_selector_module():
    spec = importlib.util.spec_from_file_location("button_target_selector", SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def target_array(stamp_ns, semantic="floor_4", target_id="track-4"):
    message = Detection3DArray()
    message.header.stamp.sec = stamp_ns // 1_000_000_000
    message.header.stamp.nanosec = stamp_ns % 1_000_000_000
    message.header.frame_id = "base_link"
    detection = Detection3D()
    detection.id = target_id
    result = ObjectHypothesisWithPose()
    result.hypothesis.class_id = semantic
    result.hypothesis.score = 0.9
    result.pose.pose.position.x = 0.30
    result.pose.pose.position.y = 0.01
    result.pose.pose.position.z = 0.24
    result.pose.pose.orientation.w = 1.0
    detection.results.append(result)
    message.detections.append(detection)
    return message


class ButtonTargetSelectorRosTest(unittest.TestCase):
    """The public adapter contract is topics, not calls into the press node."""

    def setUp(self):
        rclpy.init()
        suffix = uuid.uuid4().hex
        self.input_topic = f"/test/semantic_targets_{suffix}"
        self.coarse_topic = f"/test/coarse_target_{suffix}"
        self.refine_topic = f"/test/refine_target_{suffix}"
        module = load_selector_module()
        self.selector = module.ButtonTargetSelectorNode(
            parameter_overrides=[
                Parameter("input_topic", value=self.input_topic),
                Parameter("coarse_target_topic", value=self.coarse_topic),
                Parameter("refine_target_topic", value=self.refine_topic),
                Parameter("target_semantic", value="floor_4"),
                Parameter("max_age_s", value=1.0),
            ]
        )
        self.observer = Node(f"selector_contract_observer_{suffix}")
        self.coarse = []
        self.refine = []
        self.observer.create_subscription(
            PoseStamped, self.coarse_topic, self.coarse.append, 10
        )
        self.observer.create_subscription(
            PoseStamped, self.refine_topic, self.refine.append, 10
        )
        self.publisher = self.observer.create_publisher(
            Detection3DArray, self.input_topic, 10
        )
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.selector)
        self.executor.add_node(self.observer)

    def tearDown(self):
        self.executor.shutdown()
        self.selector.destroy_node()
        self.observer.destroy_node()
        rclpy.shutdown()

    def spin_until(self, predicate, timeout_s=2.0):
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            self.executor.spin_once(timeout_sec=0.05)
            if predicate():
                return True
        return predicate()

    def test_emits_one_coarse_then_only_newer_refinement(self):
        self.assertTrue(
            self.spin_until(lambda: self.publisher.get_subscription_count() == 1),
            "selector subscription did not discover the perception publisher",
        )

        first_stamp = self.selector.get_clock().now().nanoseconds
        self.publisher.publish(target_array(first_stamp))
        self.assertTrue(self.spin_until(lambda: len(self.coarse) == 1))
        self.assertEqual(len(self.refine), 0)
        self.assertEqual(self.coarse[0].header.frame_id, "base_link")

        second_stamp = self.selector.get_clock().now().nanoseconds
        self.publisher.publish(target_array(second_stamp))
        self.assertTrue(self.spin_until(lambda: len(self.refine) == 1))
        self.assertEqual(len(self.coarse), 1)
        self.assertGreater(
            self.refine[0].header.stamp.sec * 1_000_000_000
            + self.refine[0].header.stamp.nanosec,
            first_stamp,
        )


if __name__ == "__main__":
    unittest.main()
