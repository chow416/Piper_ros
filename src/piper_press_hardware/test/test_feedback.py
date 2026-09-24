import math
from types import SimpleNamespace

import pytest

from piper_press_hardware.feedback import (
    JOINT_NAMES,
    InvalidFeedback,
    decode_joint_feedback,
)


def message(timestamp=100.0, angles=(-1719, -2187, 1618, 806, 20455, 0)):
    joints = {f'joint_{index}': angle for index, angle in enumerate(angles, 1)}
    return SimpleNamespace(
        time_stamp=timestamp,
        joint_state=SimpleNamespace(**joints),
    )


def decode(sample, now=100.05):
    return decode_joint_feedback(sample, now, max_age_sec=0.25,
                                 max_abs_angle_rad=3.5)


def test_converts_millidegrees_to_radians_and_omits_j6():
    sample = decode(message(angles=(1000, 0, 0, 0, -90000, 100000)))
    assert JOINT_NAMES == ('joint1', 'joint2', 'joint3', 'joint4', 'joint5')
    assert len(sample.positions_rad) == 5
    assert sample.positions_rad[0] == pytest.approx(math.pi / 180)
    assert sample.positions_rad[4] == pytest.approx(-math.pi / 2)


@pytest.mark.parametrize('timestamp,now', [
    (0.0, 100.0),
    (99.0, 100.0),
    (101.0, 100.0),
    (float('nan'), 100.0),
])
def test_rejects_missing_stale_or_future_feedback(timestamp, now):
    with pytest.raises(InvalidFeedback):
        decode(message(timestamp), now=now)


def test_rejects_impossible_angle():
    with pytest.raises(InvalidFeedback):
        decode(message(angles=(float('nan'), 0, 0, 0, 0, 0)))
    with pytest.raises(InvalidFeedback):
        decode(message(angles=(300000, 0, 0, 0, 0, 0)))
