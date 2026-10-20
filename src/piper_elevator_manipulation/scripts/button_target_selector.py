#!/usr/bin/env python3
"""Select one semantic button from perception without changing its geometry.

The perception package owns depth, hand-eye, and base-frame tracking.  This
module only validates that public contract and, in the next layer, adapts one
selected detection to the press task's existing PoseStamped topics.
"""

from copy import deepcopy
from dataclasses import dataclass
from math import dist, isfinite, sqrt
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from vision_msgs.msg import Detection3DArray


# The press task waits on a volatile, reliable topic.  Keeping that contract
# here avoids replaying an old press target whenever the task is restarted.
CONTRACT_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    durability=DurabilityPolicy.VOLATILE,
)


@dataclass(frozen=True)
class SelectionDecision:
    """Messages to publish after processing one perception array."""

    coarse: Optional[PoseStamped]
    refine: Optional[PoseStamped]
    reason: str


@dataclass(frozen=True)
class _Candidate:
    """One exact-semantic detection, represented without mutating camera output."""

    tracking_id: str
    pose: PoseStamped
    position: tuple[float, float, float]
    stamp_ns: int


class FixedSemanticSelection:
    """Choose one fixed semantic target and retain its identity for refinement.

    The input position is already in ``base_link`` according to the perception
    package contract.  This class copies that position and source timestamp
    without TF, offsets, or a second hand-eye calibration. Its orientation is
    constructed from the configured, fixed panel-normal direction so a
    position-only detector never supplies an invented press axis.
    """

    def __init__(
        self,
        target_semantic: str,
        expected_frame: str,
        max_age_s: float,
        future_tolerance_s: float,
        reacquire_distance_m: float,
        press_direction_source: str = "perception",
        fixed_press_direction: tuple[float, float, float] = (1.0, 0.0, 0.0),
    ):
        if not target_semantic:
            raise ValueError("target_semantic must be non-empty")
        if not expected_frame:
            raise ValueError("expected_frame must be non-empty")
        if max_age_s <= 0.0:
            raise ValueError("max_age_s must be greater than zero")
        if future_tolerance_s < 0.0:
            raise ValueError("future_tolerance_s must be non-negative")
        if reacquire_distance_m <= 0.0:
            raise ValueError("reacquire_distance_m must be greater than zero")
        if press_direction_source not in ("perception", "fixed"):
            raise ValueError("press_direction_source must be 'perception' or 'fixed'")

        self.target_semantic = target_semantic
        self.expected_frame = expected_frame
        self.max_age_ns = int(max_age_s * 1_000_000_000)
        self.future_tolerance_ns = int(future_tolerance_s * 1_000_000_000)
        self.reacquire_distance_m = reacquire_distance_m
        self.press_direction_source = press_direction_source
        self.fixed_press_direction = self._normalized(fixed_press_direction)
        self._pending: Optional[_Candidate] = None
        self._coarse_sent = False
        self._locked_tracking_id: Optional[str] = None
        self._anchor_position: Optional[tuple[float, float, float]] = None
        self._last_refine_stamp_ns: Optional[int] = None

    def process(
        self,
        message: Detection3DArray,
        now_ns: int,
        coarse_subscriber_ready: bool,
    ) -> SelectionDecision:
        """Return at most one coarse or one later refinement pose.

        ``coarse_subscriber_ready`` prevents the sole coarse trigger from
        disappearing before the press-task subscriber exists.  Once coarse
        has been emitted, only newer observations for the locked tracking ID
        (or one unambiguous nearby re-acquisition) are allowed through.
        """
        candidates, reason = self._extract_candidates(message, now_ns)
        if reason:
            if not self._coarse_sent:
                self._pending = None
            return SelectionDecision(None, None, reason)

        if not self._coarse_sent:
            if len(candidates) == 0:
                self._pending = None
                return SelectionDecision(None, None, "requested semantic is absent")
            if len(candidates) != 1:
                self._pending = None
                return SelectionDecision(None, None, "requested semantic is ambiguous")

            candidate = candidates[0]
            self._pending = candidate
            if not coarse_subscriber_ready:
                return SelectionDecision(None, None, "waiting for coarse target subscriber")
            return self._emit_coarse(candidate)

        candidate, reason = self._locked_candidate(candidates)
        if candidate is None:
            return SelectionDecision(None, None, reason)
        if candidate.stamp_ns <= self._last_refine_stamp_ns:
            return SelectionDecision(None, None, "not a newer observation")

        self._last_refine_stamp_ns = candidate.stamp_ns
        return SelectionDecision(None, deepcopy(candidate.pose), "refinement for locked target")

    def _emit_coarse(self, candidate: _Candidate) -> SelectionDecision:
        self._coarse_sent = True
        self._locked_tracking_id = candidate.tracking_id
        self._anchor_position = candidate.position
        self._last_refine_stamp_ns = candidate.stamp_ns
        self._pending = None
        return SelectionDecision(deepcopy(candidate.pose), None, "coarse target selected")

    def _extract_candidates(
        self, message: Detection3DArray, now_ns: int
    ) -> tuple[list[_Candidate], str]:
        stamp_ns = self._stamp_ns(message)
        if stamp_ns <= 0:
            return [], "source timestamp is zero"
        if message.header.frame_id != self.expected_frame:
            return [], "unexpected source frame"

        age_ns = now_ns - stamp_ns
        if age_ns > self.max_age_ns:
            return [], "source timestamp is stale"
        if age_ns < -self.future_tolerance_ns:
            return [], "source timestamp is too far in the future"
        if not message.detections:
            return [], "no current detections"
        if not any(detection.results for detection in message.detections):
            return [], "detections contain no hypotheses"

        candidates: list[_Candidate] = []
        for detection in message.detections:
            if not detection.id:
                continue
            for result in detection.results:
                if result.hypothesis.class_id != self.target_semantic:
                    continue
                if self.press_direction_source == "perception":
                    orientation = self._validated_orientation(result)
                    if orientation is None:
                        continue
                else:
                    orientation = None
                pose = self._pose_from(message, result, orientation)
                position = (pose.pose.position.x, pose.pose.position.y, pose.pose.position.z)
                if not all(isfinite(value) for value in position):
                    continue
                candidates.append(
                    _Candidate(
                        tracking_id=detection.id,
                        pose=pose,
                        position=position,
                        stamp_ns=stamp_ns,
                    )
                )
        return candidates, ""

    @staticmethod
    def _validated_orientation(result) -> Optional[tuple[float, float, float, float]]:
        """The perception orientation, finite and normalised, or None.

        The perception contract publishes the press normal as the local +Z of
        the pose; an invalid quaternion can only be dropped, never replaced by
        a fixed direction, so a vision failure is visible instead of silent.
        """
        quaternion = result.pose.pose.orientation
        values = (quaternion.x, quaternion.y, quaternion.z, quaternion.w)
        if not all(isfinite(value) for value in values):
            return None
        magnitude = sqrt(sum(value * value for value in values))
        if magnitude <= 1.0e-9:
            return None
        return tuple(value / magnitude for value in values)

    def _locked_candidate(
        self, candidates: list[_Candidate]
    ) -> tuple[Optional[_Candidate], str]:
        same_id = [
            candidate
            for candidate in candidates
            if candidate.tracking_id == self._locked_tracking_id
        ]
        if len(same_id) == 1:
            return same_id[0], ""
        if len(same_id) > 1:
            return None, "locked target has ambiguous hypotheses"

        nearby = [
            candidate
            for candidate in candidates
            if dist(candidate.position, self._anchor_position)
            <= self.reacquire_distance_m
        ]
        if len(nearby) == 1:
            self._locked_tracking_id = nearby[0].tracking_id
            return nearby[0], ""
        if len(nearby) > 1:
            return None, "target re-acquisition is ambiguous"
        return None, "locked target is absent"

    @staticmethod
    def _stamp_ns(message: Detection3DArray) -> int:
        return message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec

    def _pose_from(self, message: Detection3DArray, result,
                   orientation: Optional[tuple[float, float, float, float]]) -> PoseStamped:
        pose = PoseStamped()
        pose.header = deepcopy(message.header)
        pose.pose.position = deepcopy(result.pose.pose.position)
        if self.press_direction_source == "perception":
            # Keep the perception normal verbatim (already validated): no TF,
            # no second hand-eye, no re-quaternionising.
            (
                pose.pose.orientation.x,
                pose.pose.orientation.y,
                pose.pose.orientation.z,
                pose.pose.orientation.w,
            ) = orientation
        else:
            (
                pose.pose.orientation.x,
                pose.pose.orientation.y,
                pose.pose.orientation.z,
                pose.pose.orientation.w,
            ) = self._quaternion_with_local_z(self.fixed_press_direction)
        return pose

    @staticmethod
    def _normalized(vector) -> tuple[float, float, float]:
        try:
            values = tuple(float(value) for value in vector)
        except (TypeError, ValueError) as exc:
            raise ValueError("fixed_press_direction must contain three numbers") from exc
        if len(values) != 3 or not all(isfinite(value) for value in values):
            raise ValueError("fixed_press_direction must be three finite numbers")
        magnitude = sqrt(sum(value * value for value in values))
        if magnitude <= 1.0e-9:
            raise ValueError("fixed_press_direction must be non-zero")
        return tuple(value / magnitude for value in values)

    @classmethod
    def _quaternion_with_local_z(cls, direction) -> tuple[float, float, float, float]:
        """Build any rotation whose local +Z axis equals ``direction``.

        The press rod is rotationally symmetric, so the free roll component is
        chosen deterministically only to produce a valid quaternion.
        """
        z_axis = cls._normalized(direction)
        helper = (1.0, 0.0, 0.0) if abs(z_axis[0]) < 0.9 else (0.0, 1.0, 0.0)
        x_axis = cls._normalized(
            (
                helper[1] * z_axis[2] - helper[2] * z_axis[1],
                helper[2] * z_axis[0] - helper[0] * z_axis[2],
                helper[0] * z_axis[1] - helper[1] * z_axis[0],
            )
        )
        y_axis = (
            z_axis[1] * x_axis[2] - z_axis[2] * x_axis[1],
            z_axis[2] * x_axis[0] - z_axis[0] * x_axis[2],
            z_axis[0] * x_axis[1] - z_axis[1] * x_axis[0],
        )
        m00, m01, m02 = x_axis[0], y_axis[0], z_axis[0]
        m10, m11, m12 = x_axis[1], y_axis[1], z_axis[1]
        m20, m21, m22 = x_axis[2], y_axis[2], z_axis[2]
        trace = m00 + m11 + m22
        if trace > 0.0:
            scale = sqrt(trace + 1.0) * 2.0
            return (
                (m21 - m12) / scale,
                (m02 - m20) / scale,
                (m10 - m01) / scale,
                0.25 * scale,
            )
        if m00 > m11 and m00 > m22:
            scale = sqrt(1.0 + m00 - m11 - m22) * 2.0
            return (
                0.25 * scale,
                (m01 + m10) / scale,
                (m02 + m20) / scale,
                (m21 - m12) / scale,
            )
        if m11 > m22:
            scale = sqrt(1.0 + m11 - m00 - m22) * 2.0
            return (
                (m01 + m10) / scale,
                0.25 * scale,
                (m12 + m21) / scale,
                (m02 - m20) / scale,
            )
        scale = sqrt(1.0 + m22 - m00 - m11) * 2.0
        return (
            (m02 + m20) / scale,
            (m12 + m21) / scale,
            0.25 * scale,
            (m10 - m01) / scale,
        )


class ButtonTargetSelectorNode(Node):
    """ROS adapter from semantic base-frame detections to the press topics."""

    def __init__(self, **kwargs):
        super().__init__("button_target_selector", **kwargs)
        self.declare_parameter("input_topic", "/perception/button_targets_base")
        self.declare_parameter("coarse_target_topic", "/press/target_pose")
        self.declare_parameter("refine_target_topic", "/press/refine_pose")
        self.declare_parameter("target_semantic", "floor_4")
        self.declare_parameter("expected_frame", "base_link")
        self.declare_parameter("max_age_s", 0.5)
        self.declare_parameter("future_tolerance_s", 0.05)
        self.declare_parameter("reacquire_distance_m", 0.04)
        self.declare_parameter("press_direction_source", "perception")
        self.declare_parameter("fixed_press_direction_x", 1.0)
        self.declare_parameter("fixed_press_direction_y", 0.0)
        self.declare_parameter("fixed_press_direction_z", 0.0)

        parameter = lambda name: self.get_parameter(name).value
        self._processor = FixedSemanticSelection(
            target_semantic=str(parameter("target_semantic")),
            expected_frame=str(parameter("expected_frame")),
            max_age_s=float(parameter("max_age_s")),
            future_tolerance_s=float(parameter("future_tolerance_s")),
            reacquire_distance_m=float(parameter("reacquire_distance_m")),
            press_direction_source=str(parameter("press_direction_source")),
            fixed_press_direction=(
                float(parameter("fixed_press_direction_x")),
                float(parameter("fixed_press_direction_y")),
                float(parameter("fixed_press_direction_z")),
            ),
        )
        self._coarse_publisher = self.create_publisher(
            PoseStamped, str(parameter("coarse_target_topic")), CONTRACT_QOS
        )
        self._refine_publisher = self.create_publisher(
            PoseStamped, str(parameter("refine_target_topic")), CONTRACT_QOS
        )
        self._subscription = self.create_subscription(
            Detection3DArray,
            str(parameter("input_topic")),
            self._on_targets,
            CONTRACT_QOS,
        )
        self._cached_message: Optional[Detection3DArray] = None
        self._last_reason = ""
        self._retry_timer = self.create_timer(0.05, self._retry_cached_coarse)
        self.get_logger().info(
            "semantic selector ready: %s -> %s (refine: %s, semantic: %s)"
            % (
                parameter("input_topic"),
                parameter("coarse_target_topic"),
                parameter("refine_target_topic"),
                parameter("target_semantic"),
            )
        )

    def _on_targets(self, message: Detection3DArray) -> None:
        self._cached_message = deepcopy(message)
        self._process(message)

    def _retry_cached_coarse(self) -> None:
        if self._cached_message is None or self._processor._coarse_sent:
            return
        self._process(self._cached_message)

    def _process(self, message: Detection3DArray) -> None:
        decision = self._processor.process(
            message,
            now_ns=self.get_clock().now().nanoseconds,
            coarse_subscriber_ready=self._coarse_publisher.get_subscription_count() > 0,
        )
        if decision.coarse is not None:
            self._coarse_publisher.publish(decision.coarse)
            self.get_logger().info(
                f"published one coarse press target for {self._processor.target_semantic}"
            )
            return
        if decision.refine is not None:
            self._refine_publisher.publish(decision.refine)
            return
        if decision.reason != self._last_reason:
            self._last_reason = decision.reason
            self.get_logger().debug(f"target not forwarded: {decision.reason}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ButtonTargetSelectorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
