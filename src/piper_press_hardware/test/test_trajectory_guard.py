from dataclasses import replace

import pytest

from piper_press_hardware.feedback import JOINT_NAMES, JointSample
from piper_press_hardware.model_limits import JointLimit
from piper_press_hardware.trajectory_guard import (
    GuardLimits,
    TrajectoryPoint,
    TrajectoryRejected,
    validate_trajectory,
)


LIMITS = (
    JointLimit(-2.618, 2.618),
    JointLimit(0.0, 3.14),
    JointLimit(-2.967, 0.0),
    JointLimit(-1.745, 1.745),
    JointLimit(-1.22, 1.22),
)
CURRENT = (0.0, 1.0, -1.0, 0.0, 0.0)
FEEDBACK = JointSample(100.0, CURRENT)
GUARD = GuardLimits(
    max_feedback_age_sec=0.25,
    max_start_error_rad=0.01,
    max_joint_step_rad=0.02,
    max_joint_speed_rad_s=0.05,
    max_duration_sec=10.0,
)
POINTS = (
    TrajectoryPoint(0.0, CURRENT),
    TrajectoryPoint(1.0, (0.01, 1.0, -1.0, 0.0, 0.0)),
)


def check(points=POINTS, feedback=FEEDBACK, names=JOINT_NAMES):
    return validate_trajectory(names, points, feedback, 100.1, LIMITS, GUARD)


def test_valid_bounded_trajectory_is_only_summarized():
    summary = check()
    assert summary.point_count == 2
    assert summary.max_speed_rad_s == pytest.approx(0.01)


@pytest.mark.parametrize('names', [
    ('joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6'),
    ('joint2', 'joint1', 'joint3', 'joint4', 'joint5'),
])
def test_no_joint6_or_reordered_joint_set(names):
    with pytest.raises(TrajectoryRejected, match='JOINT_SET'):
        check(names=names)


def test_stale_feedback_is_rejected():
    with pytest.raises(TrajectoryRejected, match='STALE_FEEDBACK'):
        check(feedback=replace(FEEDBACK, timestamp_sec=99.0))


def test_current_joint_outside_urdf_is_rejected():
    out_of_bounds = replace(FEEDBACK, positions_rad=(0.0, -0.038, 0.028, 0.0, 0.0))
    with pytest.raises(TrajectoryRejected, match='CURRENT_LIMIT'):
        check(feedback=out_of_bounds)


def test_initial_jump_is_rejected():
    points = (TrajectoryPoint(1.0, (0.015, 1.0, -1.0, 0.0, 0.0)),)
    with pytest.raises(TrajectoryRejected, match='START_MISMATCH'):
        check(points=points)


def test_later_speed_and_time_are_checked():
    too_fast = POINTS + (TrajectoryPoint(1.1, (0.02, 1.0, -1.0, 0.0, 0.0)),)
    with pytest.raises(TrajectoryRejected, match='SPEED_LIMIT'):
        check(points=too_fast)
    repeated_time = POINTS + (TrajectoryPoint(1.0, CURRENT),)
    with pytest.raises(TrajectoryRejected, match='TIMING'):
        check(points=repeated_time)


def test_invalid_point_shape_and_limit_are_checked():
    with pytest.raises(TrajectoryRejected, match='POINT_SHAPE'):
        check(points=(TrajectoryPoint(1.0, CURRENT[:4]),))
    invalid = POINTS + (TrajectoryPoint(2.0, (0.01, 3.2, -1.0, 0.0, 0.0)),)
    with pytest.raises(TrajectoryRejected, match='JOINT_LIMIT'):
        check(points=invalid)
