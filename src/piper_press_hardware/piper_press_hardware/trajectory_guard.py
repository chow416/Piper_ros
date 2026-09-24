"""Pure, fail-closed validation for a future J1-J5 execution adapter.

This module has no ROS subscriber, SDK connection, arm-enable call, or CAN
transmit path. Passing validation is necessary, not sufficient, for motion.
"""

from dataclasses import dataclass
import math
from typing import Sequence

from .feedback import JOINT_NAMES, JointSample
from .model_limits import JointLimit, joint_limit_violations


class TrajectoryRejected(ValueError):
    """A proposed trajectory is not suitable for hardware execution."""

    def __init__(self, code: str, detail: str):
        super().__init__(f'{code}: {detail}')
        self.code = code
        self.detail = detail


@dataclass(frozen=True)
class TrajectoryPoint:
    time_from_start_sec: float
    positions_rad: tuple[float, ...]


@dataclass(frozen=True)
class GuardLimits:
    max_feedback_age_sec: float
    max_start_error_rad: float
    max_joint_step_rad: float
    max_joint_speed_rad_s: float
    max_duration_sec: float

    def __post_init__(self):
        for name, value in vars(self).items():
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f'{name} must be finite and positive')


@dataclass(frozen=True)
class TrajectorySummary:
    duration_sec: float
    point_count: int
    max_step_rad: float
    max_speed_rad_s: float


def validate_trajectory(
    joint_names: Sequence[str],
    points: Sequence[TrajectoryPoint],
    feedback: JointSample,
    now_sec: float,
    joint_limits: tuple[JointLimit, ...],
    guard: GuardLimits,
) -> TrajectorySummary:
    """Validate a candidate without accepting, scheduling, or sending it."""
    if tuple(joint_names) != JOINT_NAMES:
        raise TrajectoryRejected('JOINT_SET', 'expected joint1 through joint5 only, in order')
    if not math.isfinite(now_sec):
        raise TrajectoryRejected('CLOCK', 'current time is not finite')
    age_sec = now_sec - feedback.timestamp_sec
    if not math.isfinite(age_sec) or age_sec < -0.05 or age_sec > guard.max_feedback_age_sec:
        raise TrajectoryRejected('STALE_FEEDBACK', f'feedback age is {age_sec:.3f}s')
    if len(feedback.positions_rad) != len(JOINT_NAMES):
        raise TrajectoryRejected('FEEDBACK_SHAPE', 'feedback must contain five positions')
    try:
        current_violations = joint_limit_violations(feedback.positions_rad, joint_limits)
    except ValueError as error:
        raise TrajectoryRejected('MODEL', str(error)) from error
    if current_violations:
        raise TrajectoryRejected('CURRENT_LIMIT', '; '.join(current_violations))
    if not points:
        raise TrajectoryRejected('EMPTY', 'trajectory has no points')

    previous_time = 0.0
    previous_positions = feedback.positions_rad
    max_step = 0.0
    max_speed = 0.0
    for index, point in enumerate(points):
        when = point.time_from_start_sec
        if not math.isfinite(when) or when < 0 or when > guard.max_duration_sec:
            raise TrajectoryRejected('TIMING', f'point {index} has invalid time')
        if index > 0 and when <= previous_time:
            raise TrajectoryRejected('TIMING', 'point times must strictly increase')
        if len(point.positions_rad) != len(JOINT_NAMES):
            raise TrajectoryRejected('POINT_SHAPE', f'point {index} must contain five positions')
        violations = joint_limit_violations(point.positions_rad, joint_limits)
        if violations:
            raise TrajectoryRejected('JOINT_LIMIT', '; '.join(violations))

        steps = [abs(target - previous) for target, previous in
                 zip(point.positions_rad, previous_positions)]
        step = max(steps)
        if index == 0 and step > guard.max_start_error_rad:
            raise TrajectoryRejected('START_MISMATCH',
                                     f'first point differs from feedback by {step:.5f} rad')
        if step > guard.max_joint_step_rad:
            raise TrajectoryRejected('STEP_LIMIT',
                                     f'point {index} step is {step:.5f} rad')
        interval = when - previous_time
        if interval == 0 and step != 0:
            raise TrajectoryRejected('TIMING', 'motion at time zero is impossible')
        speed = step / interval if interval > 0 else 0.0
        if speed > guard.max_joint_speed_rad_s:
            raise TrajectoryRejected('SPEED_LIMIT',
                                     f'point {index} speed is {speed:.5f} rad/s')
        max_step = max(max_step, step)
        max_speed = max(max_speed, speed)
        previous_time = when
        previous_positions = point.positions_rad

    if previous_time <= 0:
        raise TrajectoryRejected('TIMING', 'trajectory must extend past time zero')
    return TrajectorySummary(previous_time, len(points), max_step, max_speed)
