"""Read the five active-joint limits from the expanded press-tool URDF."""

from dataclasses import dataclass
import math
import xml.etree.ElementTree as ET

from .feedback import JOINT_NAMES


@dataclass(frozen=True)
class JointLimit:
    lower_rad: float
    upper_rad: float


def parse_press_joint_limits(urdf_xml: str) -> tuple[JointLimit, ...]:
    """Fail if the model is not the five-axis, stator-mounted press variant."""
    root = ET.fromstring(urdf_xml)
    if root.tag != 'robot' or root.find("link[@name='press_tcp']") is None:
        raise ValueError('URDF must be the press-tool robot with press_tcp')

    joint6 = root.find("joint[@name='joint6']")
    if joint6 is not None and joint6.get('type') != 'fixed':
        raise ValueError('joint6 must not be actuated in the press-tool URDF')

    limits = []
    for name in JOINT_NAMES:
        joint = root.find(f"joint[@name='{name}']")
        if joint is None or joint.get('type') != 'revolute':
            raise ValueError(f'{name} must be a revolute joint')
        element = joint.find('limit')
        if element is None:
            raise ValueError(f'{name} has no position limits')
        try:
            lower = float(element.attrib['lower'])
            upper = float(element.attrib['upper'])
        except (KeyError, ValueError) as error:
            raise ValueError(f'{name} has invalid position limits') from error
        if not math.isfinite(lower) or not math.isfinite(upper) or lower >= upper:
            raise ValueError(f'{name} has invalid position limits')
        limits.append(JointLimit(lower, upper))
    return tuple(limits)


def joint_limit_violations(
    positions_rad: tuple[float, ...], limits: tuple[JointLimit, ...]
) -> tuple[str, ...]:
    """Report strict URDF violations; never silently widen a physical limit."""
    if len(positions_rad) != len(JOINT_NAMES) or len(limits) != len(JOINT_NAMES):
        raise ValueError('expected exactly J1-J5 positions and limits')
    violations = []
    for name, position, bound in zip(JOINT_NAMES, positions_rad, limits):
        if not math.isfinite(position):
            violations.append(f'{name}: non-finite position')
        elif position < bound.lower_rad:
            violations.append(
                f'{name}: {position:.5f} rad below URDF lower '
                f'{bound.lower_rad:.5f} rad')
        elif position > bound.upper_rad:
            violations.append(
                f'{name}: {position:.5f} rad above URDF upper '
                f'{bound.upper_rad:.5f} rad')
    return tuple(violations)
