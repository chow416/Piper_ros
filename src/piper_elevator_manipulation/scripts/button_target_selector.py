#!/usr/bin/env python3
"""Select one semantic button from perception without changing its geometry.

The perception package owns depth, hand-eye, and base-frame tracking.  This
module only validates that public contract and, in the next layer, adapts one
selected detection to the press task's existing PoseStamped topics.
"""

from copy import deepcopy
from dataclasses import dataclass
from math import dist, isfinite
from typing import Optional

from geometry_msgs.msg import PoseStamped
from vision_msgs.msg import Detection3DArray


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

    The input pose is already in ``base_link`` according to the perception
    package contract.  This class intentionally copies it byte-for-byte into
    ``PoseStamped`` rather than applying TF, offsets, or a second hand-eye
    calibration.
    """

    def __init__(
        self,
        target_semantic: str,
        expected_frame: str,
        max_age_s: float,
        future_tolerance_s: float,
        reacquire_distance_m: float,
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

        self.target_semantic = target_semantic
        self.expected_frame = expected_frame
        self.max_age_ns = int(max_age_s * 1_000_000_000)
        self.future_tolerance_ns = int(future_tolerance_s * 1_000_000_000)
        self.reacquire_distance_m = reacquire_distance_m
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
                pose = self._pose_from(message, result)
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

    @staticmethod
    def _pose_from(message: Detection3DArray, result) -> PoseStamped:
        pose = PoseStamped()
        pose.header = deepcopy(message.header)
        pose.pose = deepcopy(result.pose.pose)
        return pose
