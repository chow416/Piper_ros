import pytest

from piper_press_hardware.model_limits import (
    joint_limit_violations,
    parse_press_joint_limits,
)


def press_urdf(joint6_type='fixed'):
    joints = ''.join(
        f'<joint name="joint{index}" type="revolute">'
        f'<limit lower="{lower}" upper="{upper}"/></joint>'
        for index, (lower, upper) in enumerate(
            ((-2.618, 2.618), (0, 3.14), (-2.967, 0),
             (-1.745, 1.745), (-1.22, 1.22)), start=1)
    )
    return (f'<robot name="piper"><link name="press_tcp"/>{joints}'
            f'<joint name="joint6" type="{joint6_type}"/></robot>')


def test_parse_press_model_has_five_active_joint_limits():
    limits = parse_press_joint_limits(press_urdf())
    assert len(limits) == 5
    assert limits[1].lower_rad == 0
    assert limits[2].upper_rad == 0


def test_reject_model_with_actuated_joint6():
    with pytest.raises(ValueError, match='joint6'):
        parse_press_joint_limits(press_urdf('revolute'))


def test_report_observed_joint2_and_joint3_boundary_mismatch():
    limits = parse_press_joint_limits(press_urdf())
    violations = joint_limit_violations(
        (-0.0300, -0.03817, 0.02824, 0.01407, 0.3570), limits)
    assert len(violations) == 2
    assert violations[0].startswith('joint2:')
    assert violations[1].startswith('joint3:')
