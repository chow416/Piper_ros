#!/usr/bin/env python3
"""Behavior tests for the fixed-semantic perception target selector.

These tests deliberately load the installed-style script directly.  The
selector is a thin ROS adapter, but its selection state must stay testable
without a camera, TF tree, MoveIt, or a real arm.
"""

import importlib.util
import math
import unittest
from pathlib import Path

from vision_msgs.msg import Detection3D, Detection3DArray, ObjectHypothesisWithPose


SCRIPT = Path(__file__).parents[1] / "scripts" / "button_target_selector.py"


def load_selector_module():
    spec = importlib.util.spec_from_file_location("button_target_selector", SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def target_array(stamp_ns=10_000_000_000, frame_id="base_link", detections=None):
    message = Detection3DArray()
    message.header.stamp.sec = stamp_ns // 1_000_000_000
    message.header.stamp.nanosec = stamp_ns % 1_000_000_000
    message.header.frame_id = frame_id
    if detections:
        message.detections = detections
    return message


def detection(
    semantic="floor_4",
    target_id="track-4",
    score=0.9,
    position=(0.30, 0.01, 0.24),
):
    item = Detection3D()
    item.id = target_id
    hypothesis = ObjectHypothesisWithPose()
    hypothesis.hypothesis.class_id = semantic
    hypothesis.hypothesis.score = score
    hypothesis.pose.pose.position.x = position[0]
    hypothesis.pose.pose.position.y = position[1]
    hypothesis.pose.pose.position.z = position[2]
    hypothesis.pose.pose.orientation.w = 1.0
    item.results.append(hypothesis)
    return item


def local_z_axis(quaternion):
    """Return the world direction of a pose's local +Z axis."""
    x, y, z, w = quaternion.x, quaternion.y, quaternion.z, quaternion.w
    return (
        2.0 * (x * z + y * w),
        2.0 * (y * z - x * w),
        1.0 - 2.0 * (x * x + y * y),
    )


class FixedSemanticSelectionValidationTest(unittest.TestCase):
    """Inputs outside the published perception contract must never move an arm."""

    def selector(self):
        module = load_selector_module()
        return module.FixedSemanticSelection(
            target_semantic="floor_4",
            expected_frame="base_link",
            max_age_s=0.5,
            future_tolerance_s=0.05,
            reacquire_distance_m=0.04,
        )

    def test_requires_nonempty_target_semantic(self):
        """An empty task choice must fail at configuration time, not select all buttons."""
        module = load_selector_module()

        with self.assertRaises(ValueError):
            module.FixedSemanticSelection("", "base_link", 0.5, 0.05, 0.04)

    def test_requires_a_finite_nonzero_fixed_press_direction(self):
        """Position-only perception must not create an arbitrary press axis."""
        module = load_selector_module()

        for direction in ((0.0, 0.0, 0.0), (math.nan, 0.0, 1.0)):
            with self.subTest(direction=direction):
                with self.assertRaises(ValueError):
                    module.FixedSemanticSelection(
                        "floor_4", "base_link", 0.5, 0.05, 0.04, direction
                    )

    def test_rejects_wrong_frame(self):
        """A camera-frame point must not be mistaken for a base-frame press point."""
        decision = self.selector().process(
            target_array(frame_id="camera_color_optical_frame", detections=[detection()]),
            now_ns=10_000_000_000,
            coarse_subscriber_ready=True,
        )

        self.assertIsNone(decision.coarse)
        self.assertIsNone(decision.refine)

    def test_rejects_zero_stale_and_future_stamp(self):
        """A timestamp error must not become a motion command."""
        selector = self.selector()
        cases = (
            target_array(stamp_ns=0, detections=[detection()]),
            target_array(stamp_ns=9_499_999_999, detections=[detection()]),
            target_array(stamp_ns=10_050_000_001, detections=[detection()]),
        )

        for message in cases:
            with self.subTest(stamp=(message.header.stamp.sec, message.header.stamp.nanosec)):
                decision = selector.process(
                    message, now_ns=10_000_000_000, coarse_subscriber_ready=True
                )
                self.assertIsNone(decision.coarse)
                self.assertIsNone(decision.refine)

    def test_empty_list_clears_cached_candidate(self):
        """A later empty perception frame invalidates a candidate cached before subscribe."""
        selector = self.selector()
        cached = selector.process(
            target_array(detections=[detection()]),
            now_ns=10_000_000_000,
            coarse_subscriber_ready=False,
        )
        cleared = selector.process(
            target_array(stamp_ns=10_100_000_000, detections=[]),
            now_ns=10_100_000_000,
            coarse_subscriber_ready=True,
        )

        self.assertIsNone(cached.coarse)
        self.assertIsNone(cleared.coarse)
        self.assertIsNone(cleared.refine)

    def test_ignores_detection_without_results(self):
        """A malformed detection cannot become a default-origin target."""
        decision = self.selector().process(
            target_array(detections=[Detection3D()]),
            now_ns=10_000_000_000,
            coarse_subscriber_ready=True,
        )

        self.assertIsNone(decision.coarse)
        self.assertIsNone(decision.refine)


class FixedSemanticSelectionLifecycleTest(unittest.TestCase):
    """A continuous perception stream must trigger exactly one press cycle."""

    def selector(self):
        module = load_selector_module()
        return module.FixedSemanticSelection(
            target_semantic="floor_4",
            expected_frame="base_link",
            max_age_s=0.5,
            future_tolerance_s=0.05,
            reacquire_distance_m=0.04,
        )

    def test_selects_only_the_requested_unique_semantic(self):
        """A visible but unrequested floor must never win target selection."""
        decision = self.selector().process(
            target_array(detections=[detection("floor_3"), detection("floor_4")]),
            now_ns=10_000_000_000,
            coarse_subscriber_ready=True,
        )

        self.assertIsNotNone(decision.coarse)
        self.assertEqual(decision.coarse.pose.position.x, 0.30)
        self.assertIsNone(decision.refine)

    def test_rejects_ambiguous_initial_semantic(self):
        """Two equally named buttons are not enough evidence to choose one to press."""
        decision = self.selector().process(
            target_array(
                detections=[
                    detection(target_id="left"),
                    detection(target_id="right", position=(0.32, 0.01, 0.24)),
                ]
            ),
            now_ns=10_000_000_000,
            coarse_subscriber_ready=True,
        )

        self.assertIsNone(decision.coarse)
        self.assertIsNone(decision.refine)

    def test_waits_for_subscriber_then_emits_cached_target_once(self):
        """The only coarse trigger is retained until its volatile subscriber exists."""
        selector = self.selector()
        message = target_array(detections=[detection()])

        before_subscribe = selector.process(
            message, now_ns=10_000_000_000, coarse_subscriber_ready=False
        )
        after_subscribe = selector.process(
            message, now_ns=10_000_000_000, coarse_subscriber_ready=True
        )

        self.assertIsNone(before_subscribe.coarse)
        self.assertIsNotNone(after_subscribe.coarse)
        self.assertIsNone(after_subscribe.refine)

    def test_later_frame_emits_refine_without_second_coarse(self):
        """Continuous detections after planning begins refine the one chosen target."""
        selector = self.selector()
        selector.process(
            target_array(detections=[detection()]),
            now_ns=10_000_000_000,
            coarse_subscriber_ready=True,
        )

        decision = selector.process(
            target_array(
                stamp_ns=10_100_000_000,
                detections=[detection(position=(0.301, 0.01, 0.24))],
            ),
            now_ns=10_100_000_000,
            coarse_subscriber_ready=True,
        )

        self.assertIsNone(decision.coarse)
        self.assertIsNotNone(decision.refine)
        self.assertEqual(decision.refine.pose.position.x, 0.301)

    def test_duplicate_stamp_is_not_forwarded_as_refinement(self):
        """The same camera frame cannot be reused as a later stop-time observation."""
        selector = self.selector()
        message = target_array(detections=[detection()])
        first = selector.process(
            message, now_ns=10_000_000_000, coarse_subscriber_ready=True
        )
        self.assertIsNotNone(first.coarse)

        decision = selector.process(
            message, now_ns=10_000_000_000, coarse_subscriber_ready=True
        )

        self.assertIsNone(decision.coarse)
        self.assertIsNone(decision.refine)

    def test_preserves_position_and_stamp_but_replaces_perception_orientation(self):
        """Position-only perception uses the configured panel-normal direction."""
        selector = self.selector()
        source = target_array(
            stamp_ns=10_000_000_000,
            detections=[detection(position=(0.31, -0.02, 0.26))],
        )
        source.detections[0].results[0].pose.pose.orientation.w = 0.0
        coarse = selector.process(
            source, now_ns=10_000_000_000, coarse_subscriber_ready=True
        ).coarse

        self.assertEqual(coarse.header.frame_id, "base_link")
        self.assertEqual((coarse.header.stamp.sec, coarse.header.stamp.nanosec), (10, 0))
        self.assertEqual(
            (coarse.pose.position.x, coarse.pose.position.y, coarse.pose.position.z),
            (0.31, -0.02, 0.26),
        )
        self.assertAlmostEqual(local_z_axis(coarse.pose.orientation)[0], 1.0)
        self.assertAlmostEqual(local_z_axis(coarse.pose.orientation)[1], 0.0)
        self.assertAlmostEqual(local_z_axis(coarse.pose.orientation)[2], 0.0)

    def test_prefers_locked_tracking_id_over_a_higher_score_duplicate(self):
        """Once coarse planning starts, a confidence change cannot switch buttons."""
        selector = self.selector()
        selector.process(
            target_array(detections=[detection(target_id="locked", score=0.3)]),
            now_ns=10_000_000_000,
            coarse_subscriber_ready=True,
        )

        decision = selector.process(
            target_array(
                stamp_ns=10_100_000_000,
                detections=[
                    detection(target_id="locked", score=0.1, position=(0.301, 0.01, 0.24)),
                    detection(target_id="other", score=0.99, position=(0.302, 0.01, 0.24)),
                ],
            ),
            now_ns=10_100_000_000,
            coarse_subscriber_ready=True,
        )

        self.assertIsNotNone(decision.refine)
        self.assertEqual(decision.refine.pose.position.x, 0.301)

    def test_reacquires_one_nearby_same_semantic_target(self):
        """A tracker reset may recover the same button only inside the stated gate."""
        selector = self.selector()
        selector.process(
            target_array(detections=[detection(target_id="old")]),
            now_ns=10_000_000_000,
            coarse_subscriber_ready=True,
        )

        decision = selector.process(
            target_array(
                stamp_ns=10_100_000_000,
                detections=[detection(target_id="new", position=(0.33, 0.01, 0.24))],
            ),
            now_ns=10_100_000_000,
            coarse_subscriber_ready=True,
        )

        self.assertIsNotNone(decision.refine)

    def test_rejects_distant_or_ambiguous_reacquisition(self):
        """A lost ID cannot silently retarget a different button."""
        distant_selector = self.selector()
        distant_selector.process(
            target_array(detections=[detection(target_id="old")]),
            now_ns=10_000_000_000,
            coarse_subscriber_ready=True,
        )
        distant = distant_selector.process(
            target_array(
                stamp_ns=10_100_000_000,
                detections=[detection(target_id="far", position=(0.35, 0.01, 0.24))],
            ),
            now_ns=10_100_000_000,
            coarse_subscriber_ready=True,
        )

        ambiguous_selector = self.selector()
        ambiguous_selector.process(
            target_array(detections=[detection(target_id="old")]),
            now_ns=10_000_000_000,
            coarse_subscriber_ready=True,
        )
        ambiguous = ambiguous_selector.process(
            target_array(
                stamp_ns=10_100_000_000,
                detections=[
                    detection(target_id="new-a", position=(0.31, 0.01, 0.24)),
                    detection(target_id="new-b", position=(0.32, 0.01, 0.24)),
                ],
            ),
            now_ns=10_100_000_000,
            coarse_subscriber_ready=True,
        )

        self.assertIsNone(distant.refine)
        self.assertIsNone(ambiguous.refine)


if __name__ == "__main__":
    unittest.main()
