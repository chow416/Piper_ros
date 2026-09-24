"""Validate and convert Piper SDK joint feedback without ROS dependencies."""

from dataclasses import dataclass
import math
from typing import Tuple


JOINT_NAMES = ('joint1', 'joint2', 'joint3', 'joint4', 'joint5')
MILLIDEGREES_TO_RADIANS = math.pi / 180_000.0


class InvalidFeedback(ValueError):
    """The SDK feedback cannot be published as a current joint sample."""


@dataclass(frozen=True)
class JointSample:
    timestamp_sec: float
    positions_rad: Tuple[float, ...]


def decode_joint_feedback(
    sdk_message,
    now_sec: float,
    max_age_sec: float,
    max_abs_angle_rad: float,
) -> JointSample:
    """Return J1–J5 only; the stator-mounted tool does not actuate with J6.

    Piper SDK angles are integer millidegrees and timestamps use Unix time.
    A missing or stale sample must never be published as a valid robot state.
    """
    timestamp = float(sdk_message.time_stamp)
    if not math.isfinite(timestamp) or timestamp <= 0:
        raise InvalidFeedback('joint feedback has no valid timestamp')

    age_sec = now_sec - timestamp
    if not math.isfinite(age_sec) or age_sec < -0.05 or age_sec > max_age_sec:
        raise InvalidFeedback('joint feedback is stale or has a future timestamp')

    positions = []
    for joint_index in range(1, 6):
        raw_angle = float(getattr(sdk_message.joint_state, f'joint_{joint_index}'))
        position_rad = raw_angle * MILLIDEGREES_TO_RADIANS
        if not math.isfinite(position_rad) or abs(position_rad) > max_abs_angle_rad:
            raise InvalidFeedback(f'joint{joint_index} angle is invalid')
        positions.append(position_rad)

    return JointSample(timestamp, tuple(positions))
