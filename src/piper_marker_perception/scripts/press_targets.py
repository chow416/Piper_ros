#!/usr/bin/env python3
"""Where the button is, once the board has been found.

The board is a fiducial: it is not the thing being pressed, it is the thing that
can be seen. Everything downstream rests on one physical assumption, so it is
stated here rather than buried in a config file:

    the board and the button are rigidly fixed to the same panel

Given that, the button's pose follows from the board's pose by a *constant*
transform, fixed at the moment someone tapes the board down and unchanged
afterwards. This module owns that constant and applies it.

    T_base <- button_tip  =  T_base <- board  .  T_board <- button_tip

That transform is stored as two pieces rather than one matrix, because that is
how it can actually be measured:

    tip_in_board          where the rod tip should land, metres, board frame
    press_axis_in_board   which way is "in", unit vector, board frame

Position is a point and the axis is a direction, and a ruler measures the first
while a protractor measures the second. Storing them separately means neither has
to be invented to complete a 4x4.

When (0, 0, -1) is not the right axis
-------------------------------------
The default axis is the board's -Z, which is correct whenever the button sits
flat on the same panel as the board: their normals coincide by construction. It
stops being correct when the button travels along some other line -- a switch on
a raised housing, or a button whose travel axis is deliberately off-normal.
Pressing such a button off-axis jams it, so the axis is a real input, not a
formality.

Why the quaternion helper is a copy
-----------------------------------
`geometry_msgs/PoseStamped` carries a quaternion, the task reads only its +Z
axis, and the roll is free. `send_press_target.py` already picks one particular
completion, and a target published by either path has to be the same message, so
the same helper rule and the same matrix-to-quaternion method are used here. The
self-test checks the two agree bit for bit; if that test ever fails, one of the
two has drifted.
"""

import math

import numpy as np

#: into the board's front face, i.e. the panel normal when board and button are
#: coplanar. See the module docstring for when this is not the right answer.
DEFAULT_PRESS_AXIS = (0.0, 0.0, -1.0)


class ConfigError(ValueError):
    """The board/target config is wrong in a way the operator has to fix."""


def _triple(node, label):
    """Read {x, y, z} (or [x, y, z]) into a float64 vector."""
    if node is None:
        raise ConfigError(f"{label} is missing")
    if isinstance(node, dict):
        try:
            values = [node["x"], node["y"], node["z"]]
        except KeyError as error:
            raise ConfigError(f"{label} needs x, y and z (missing {error})") from error
    elif isinstance(node, (list, tuple)) and len(node) == 3:
        values = list(node)
    else:
        raise ConfigError(f"{label} must be {{x, y, z}} or a 3-element list")
    try:
        vector = np.array([float(v) for v in values], dtype=float)
    except (TypeError, ValueError) as error:
        raise ConfigError(f"{label} must be numbers, got {values}") from error
    if not np.all(np.isfinite(vector)):
        raise ConfigError(f"{label} contains a non-finite number: {values}")
    return vector


class PressTarget:
    """One button, described in the board frame."""

    def __init__(self, name, tip_in_board, press_axis_in_board=None,
                 taught=False, refine_tip_in_board=None, refine_taught=False):
        self.name = str(name)
        self.tip_in_board = _triple(tip_in_board, f"targets.{name}.tip_in_board")

        axis = (np.asarray(DEFAULT_PRESS_AXIS, dtype=float)
                if press_axis_in_board is None
                else _triple(press_axis_in_board,
                             f"targets.{name}.press_axis_in_board"))
        length = float(np.linalg.norm(axis))
        if length < 1e-9:
            raise ConfigError(f"targets.{name}.press_axis_in_board is zero")
        self.press_axis_in_board = axis / length

        #: Set by the teach script. Until then the numbers are guesses, and
        #: guessing here means driving a rod into an unknown spot.
        self.taught = bool(taught)

        #: The same button, measured from the *other* fiducial -- the single
        #: marker the refined observation looks at. None when the deployment
        #: has only one board, which is a supported configuration.
        self.refine_tip_in_board = (
            None if refine_tip_in_board is None
            else _triple(refine_tip_in_board,
                         f"targets.{name}.refine_tip_in_board"))
        self.refine_taught = bool(refine_taught)

    def resolve(self, T_base_board, direction_sign=1.0):
        """Return (position_m, direction_unit) both in the target frame.

        `T_base_board` is the 4x4 board pose. `direction_sign` flips the axis,
        matching the task node's own parameter, for the case where someone
        stores a surface *outward* normal instead of an inward one.
        """
        return self._resolve_from(T_base_board, self.tip_in_board,
                                  direction_sign)

    def resolve_refine(self, T_base_refine, direction_sign=1.0):
        """The same button, located from the refined board instead.

        Only the *position* is meant to be taken from this one: a single marker
        has four coplanar corners, so its normal is not an average of anything
        and is not worth using. The direction returned here still follows the
        button's constant axis, and the task discards it rather than fusing it.
        """
        if self.refine_tip_in_board is None:
            raise ConfigError(
                f"targets.{self.name} has no refine_tip_in_board, so the "
                f"refined board cannot locate it")
        return self._resolve_from(T_base_refine, self.refine_tip_in_board,
                                  direction_sign)

    def _resolve_from(self, T_base_board, tip_in_board, direction_sign):
        matrix = np.asarray(T_base_board, dtype=float)
        if matrix.shape != (4, 4):
            raise ValueError(f"board pose must be 4x4, got {matrix.shape}")
        rotation = matrix[:3, :3]
        translation = matrix[:3, 3]

        position = translation + rotation @ tip_in_board
        direction = rotation @ self.press_axis_in_board * float(direction_sign)
        return position, direction

    def describe(self):
        t = self.tip_in_board
        a = self.press_axis_in_board
        return (f"{self.name}: tip=({t[0]:+.4f}, {t[1]:+.4f}, {t[2]:+.4f}) m  "
                f"axis=({a[0]:+.3f}, {a[1]:+.3f}, {a[2]:+.3f})  "
                f"{'已教示' if self.taught else '⚠️ 未教示'}")

    def __repr__(self):
        return f"PressTarget({self.describe()})"


class PressTargetSet:
    """Every button on the panel, keyed by name."""

    def __init__(self, targets):
        if not targets:
            raise ConfigError("no targets defined")
        self.targets = dict(targets)

    def __getitem__(self, name):
        try:
            return self.targets[name]
        except KeyError:
            raise ConfigError(
                f"unknown target {name!r}; config has "
                f"{sorted(self.targets)}") from None

    def __contains__(self, name):
        return name in self.targets

    def __len__(self):
        return len(self.targets)

    def names(self):
        return sorted(self.targets)

    def untaught(self):
        return sorted(n for n, t in self.targets.items() if not t.taught)

    @classmethod
    def from_dict(cls, data):
        if not isinstance(data, dict):
            raise ConfigError("config root must be a mapping")
        node = data.get("targets")
        if not isinstance(node, dict) or not node:
            raise ConfigError(
                "config has no 'targets' section -- see the block at the "
                "bottom of board.yaml")
        targets = {}
        for name, entry in node.items():
            entry = entry or {}
            targets[name] = PressTarget(
                name,
                tip_in_board=entry.get("tip_in_board", {"x": 0, "y": 0, "z": 0}),
                press_axis_in_board=entry.get("press_axis_in_board"),
                taught=entry.get("taught", False),
                refine_tip_in_board=entry.get("refine_tip_in_board"),
                refine_taught=entry.get("refine_taught", False))
        return cls(targets)

    @classmethod
    def from_yaml(cls, path):
        import yaml
        with open(path, encoding="utf-8") as handle:
            return cls.from_dict(yaml.safe_load(handle) or {})


# -- orientation ---------------------------------------------------------

def rotation_with_z_axis(direction):
    """A rotation whose third column is `direction`, in the target frame.

    The completion is arbitrary, so one is chosen deterministically to keep
    repeated runs publishing identical messages. Same helper rule as
    `send_press_target.py` -- see the module docstring.
    """
    z_axis = np.asarray(direction, dtype=float)
    norm = float(np.linalg.norm(z_axis))
    if norm < 1e-9:
        raise ValueError("direction vector is zero")
    z_axis = z_axis / norm

    helper = (np.array([1.0, 0.0, 0.0]) if abs(z_axis[0]) < 0.9
              else np.array([0.0, 1.0, 0.0]))
    x_axis = np.cross(helper, z_axis)
    x_axis /= np.linalg.norm(x_axis)
    y_axis = np.cross(z_axis, x_axis)
    return np.column_stack([x_axis, y_axis, z_axis])


def quaternion_from_matrix(matrix):
    """(x, y, z, w) from a 3x3 rotation matrix, Shepperd's method.

    Chosen for numerical behaviour near 180 degrees, where the naive formula
    loses precision in w.
    """
    m = np.asarray(matrix, dtype=float)
    trace = m[0, 0] + m[1, 1] + m[2, 2]
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        return ((m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s,
                (m[1, 0] - m[0, 1]) / s, 0.25 * s)
    if m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2.0
        return (0.25 * s, (m[0, 1] + m[1, 0]) / s, (m[0, 2] + m[2, 0]) / s,
                (m[2, 1] - m[1, 2]) / s)
    if m[1, 1] > m[2, 2]:
        s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2.0
        return ((m[0, 1] + m[1, 0]) / s, 0.25 * s, (m[1, 2] + m[2, 1]) / s,
                (m[0, 2] - m[2, 0]) / s)
    s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2.0
    return ((m[0, 2] + m[2, 0]) / s, (m[1, 2] + m[2, 1]) / s, 0.25 * s,
            (m[1, 0] - m[0, 1]) / s)


def quaternion_for_direction(direction):
    """(x, y, z, w) whose +Z axis is `direction`."""
    return quaternion_from_matrix(rotation_with_z_axis(direction))


# -- pose helpers --------------------------------------------------------

def matrix_from_pose_stamped(message):
    """4x4 from a geometry_msgs/PoseStamped."""
    p = message.pose.position
    q = message.pose.orientation
    matrix = np.eye(4)
    matrix[:3, :3] = quaternion_to_matrix(q.x, q.y, q.z, q.w)
    matrix[:3, 3] = [p.x, p.y, p.z]
    return matrix


def quaternion_to_matrix(x, y, z, w):
    """3x3 rotation matrix from a quaternion, normalising first."""
    norm = x * x + y * y + z * z + w * w
    if norm < 1e-12:
        raise ValueError("zero-length quaternion")
    s = 2.0 / norm
    return np.array([
        [1 - s * (y * y + z * z), s * (x * y - z * w), s * (x * z + y * w)],
        [s * (x * y + z * w), 1 - s * (x * x + z * z), s * (y * z - x * w)],
        [s * (x * z - y * w), s * (y * z + x * w), 1 - s * (x * x + y * y)],
    ])


def transform_to_matrix(transform):
    """4x4 from a geometry_msgs/Transform.

    tf2_ros has no such helper, and `tf_transformations` would pull a whole
    package in for four lines of arithmetic.
    """
    translation = transform.translation
    rotation = transform.rotation
    matrix = np.eye(4)
    matrix[:3, :3] = quaternion_to_matrix(rotation.x, rotation.y, rotation.z,
                                          rotation.w)
    matrix[:3, 3] = [translation.x, translation.y, translation.z]
    return matrix


def yaml_block(target, indent="  "):
    """The target rendered as YAML, for pasting back into board.yaml.

    Printed rather than written: rewriting the file would drop the comments
    explaining what the numbers mean and where they came from, and those
    comments are most of the file's value.
    """
    t = target.tip_in_board
    a = target.press_axis_in_board
    return (f"{indent}{target.name}:\n"
            f"{indent}  taught: true\n"
            f"{indent}  tip_in_board: "
            f"{{x: {t[0]:+.6f}, y: {t[1]:+.6f}, z: {t[2]:+.6f}}}\n"
            f"{indent}  press_axis_in_board: "
            f"{{x: {a[0]:+.6f}, y: {a[1]:+.6f}, z: {a[2]:+.6f}}}")
