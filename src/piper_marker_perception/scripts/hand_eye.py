#!/usr/bin/env python3
"""Solve for where the camera sits on the tool, from a session of arm poses.

The last step in the chain, and the only one that needs the robot. Everything
before it -- detecting the board, solving its pose -- is in `aruco_board.py`,
and everything after it -- turning a board pose into a button pose -- is in
`press_targets.py`. This file is the one in between.

The problem
-----------
The camera is bolted to the tool. The board is bolted to the table. Move the arm
and both stay put relative to each other, so for every pose `i`:

    T_base_end[i] @ X @ T_cam_board[i] = T_base_board        (constant)

The two outer terms are measured: `T_base_end` from the encoders, `T_cam_board`
from solvePnP. `X` -- where the camera sits on the tool -- is the unknown, and
it is the same for every pose. That is `AX = XB`, and `cv2.calibrateHandEye` is
one call away.

So why this file exists at all
-----------------------------
Because the solver's own output cannot tell you whether it worked. It returns
the X that best fits the data it was given, and it returns one whatever the data
says -- including when the data is wrong. There is no residual to read, no
threshold to cross. A hand-eye calibration built on a bad intrinsic is a
self-consistent wrong answer, and it looks exactly like a right one.

What does tell you is the line above: the board does not move. Feed a candidate
X back in and every pose should agree on where the board is. If they do not, the
spread is the error, in metres and degrees, and it is the only honest figure in
the whole procedure. So this tool computes it for every X it hands back, and
prints it first.

The frame convention, which is where this goes wrong
----------------------------------------------------
`calibrateHandEye` solves in the **optical** frame, because that is what PnP
gave it. The URDF wants the **mechanical** frame that `wrist_depth_camera_joint`
connects. The two differ by the fixed rotation in `optical_joint`:

    T_tool<-link = T_tool<-optical @ T_optical<-link

Forgetting that second factor leaves a 90 degree error that produces a pose
which looks entirely reasonable. Both forms are printed, and the URDF one is
printed as a paste-ready `<origin>` line.

Usage
-----
    # The normal path: a capture_session directory, once the camera is mounted.
    hand_eye.py --session ~/capture/2026-09-30 \\
                --camera-info config/camera_info_orbbec.yaml

    # Re-solve without touching the images, for trying things out.
    hand_eye.py --pairs pairs.json --method park

    # Screen a session on the spot, before trusting it.
    hand_eye.py --session ~/capture/2026-09-30 --check-only
"""

import argparse
import glob
import json
import math
import os
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from aruco_board import (BoardSpec, CameraIntrinsics,      # noqa: E402
                         detect, resolve_config_path)
from press_targets import (quaternion_from_matrix,        # noqa: E402
                           quaternion_to_matrix)

#: The fixed rotation between the camera's mechanical frame and its optical
#: frame, copied from `optical_joint` in `wrist_depth_camera.xacro`. Not a
#: choice -- it is what the URDF says, and it is the same for both cameras.
OPTICAL_TO_LINK_RPY = (0.0, -1.57079632679, 0.0)

#: OpenCV's relative-pose solvers. They are five answers to one question, so
#: they should agree; when they do not, the data is the problem rather than the
#: method, and that is worth knowing before believing the number.
METHOD_NAMES = ("TSAI", "PARK", "HORAUD", "ANDREFF", "DANIILIDIS")

#: How far the board is allowed to wander, in base_link, between poses that
#: should agree on it exactly. These are report thresholds for real hardware,
#: where the encoders, the detection and the mounting all contribute.
GOOD_POSITION_MM = 2.0
OK_POSITION_MM = 10.0
GOOD_ROTATION_DEG = 0.2
OK_ROTATION_DEG = 1.0

#: Below this, the arm has not turned enough for the solver to separate the
#: camera's orientation from its position. Same figure the capture tool gates on.
MIN_ROTATION_SPREAD_DEG = 15.0

DEFAULT_BOARD = "config/board.yaml"


# -- small matrix helpers -------------------------------------------------

def invert(matrix):
    """Inverse of a rigid transform, from its structure rather than by solving.

    `np.linalg.inv` would give the same answer to within rounding; this gives it
    exactly, and cannot silently return something that is not a rigid transform.
    """
    rotation = matrix[:3, :3]
    out = np.eye(4)
    out[:3, :3] = rotation.T
    out[:3, 3] = -rotation.T @ matrix[:3, 3]
    return out


def rpy_to_matrix(rpy):
    """URDF `rpy`: fixed-axis roll, pitch, yaw, i.e. R = Rz(y) @ Ry(p) @ Rx(r).

    Done through a quaternion rather than three multiplied matrices because the
    optical joint sits at exactly -90 degrees of pitch -- the gimbal-lock case,
    where the matrix form has a singularity. The quaternion path has no special
    case, and a round-trip is asserted in `selftest_handeye.py`.
    """
    roll, pitch, yaw = (float(value) for value in rpy)
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    return quaternion_to_matrix(
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy)


def matrix_to_rpy(matrix):
    """The inverse of `rpy_to_matrix`, for printing a URDF `<origin>`.

    At the +/-90 degree singularities the angles are not unique: roll and yaw
    trade off and every combination describes the same rotation. The
    representative with roll = 0 is returned, because the obvious alternative --
    leave whatever the closed formula produces -- prints the wrist camera as if
    it were rolled fifty-five degrees when it is not. The matrix is the same
    either way; only the reading changes, and one of the two readings is scary.
    """
    rotation = np.asarray(matrix, dtype=float)[:3, :3]
    qx, qy, qz, qw = quaternion_from_matrix(rotation)
    sin_pitch = max(-1.0, min(1.0, 2.0 * (qw * qy - qz * qx)))

    if abs(sin_pitch) > 1.0 - 1e-9:
        if sin_pitch < 0.0:                    # pitch = -90: yaw from R[1]
            yaw = math.atan2(-rotation[1, 2], rotation[1, 1])
        else:                                  # pitch = +90: yaw from R[0]
            yaw = math.atan2(rotation[0, 1], rotation[1, 1])
        return (0.0, math.copysign(math.pi / 2.0, sin_pitch), yaw)

    roll = math.atan2(2.0 * (qw * qx + qy * qz), 1.0 - 2.0 * (qx * qx + qy * qy))
    yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
    return (roll, math.asin(sin_pitch), yaw)


def matrix_from_rotation(rotation, translation=(0.0, 0.0, 0.0)):
    """A 4x4 transform from a 3x3 rotation and a translation.

    `rpy_to_matrix` returns a rotation, because that is what a rotation is, and
    several places here want it as a transform. Writing `np.eye(4)` out by hand
    at each of them is how a 3x3 ends up multiplied against a 4x4.
    """
    out = np.eye(4)
    out[:3, :3] = np.asarray(rotation, dtype=float).reshape(3, 3)
    out[:3, 3] = translation
    return out


def rotation_angle_deg(a, b):
    """Angle between two rotations, in degrees."""
    cosine = (np.trace(np.asarray(a) @ np.asarray(b).T) - 1.0) / 2.0
    return math.degrees(math.acos(max(-1.0, min(1.0, cosine))))


def describe(matrix):
    """(position, rotation) of a 4x4, as a tuple and a 3x3."""
    return np.asarray(matrix, dtype=float)[:3, 3].copy(), \
        np.asarray(matrix, dtype=float)[:3, :3].copy()


# -- the data -------------------------------------------------------------

class PosePair:
    """One arm pose, and where the board was in the camera when it was there.

    `T_base_end` comes from the encoders, through forward kinematics, and is
    trustworthy. `T_cam_board` comes from solvePnP and is only as good as the
    intrinsics and the detection behind it. The whole method rests on the second
    one, which is why the consistency check exists rather than being optional.
    """

    def __init__(self, name, T_base_end, T_cam_board, markers=None,
                 rms_px=None):
        self.name = name
        self.T_base_end = np.asarray(T_base_end, dtype=float).reshape(4, 4)
        self.T_cam_board = np.asarray(T_cam_board, dtype=float).reshape(4, 4)
        self.markers = markers
        self.rms_px = rms_px

    def to_dict(self):
        return {"name": self.name,
                "T_base_end": self.T_base_end.tolist(),
                "T_cam_board": self.T_cam_board.tolist(),
                "markers": self.markers,
                "rms_px": self.rms_px}

    @classmethod
    def from_dict(cls, data):
        return cls(data.get("name", "?"), data["T_base_end"],
                   data["T_cam_board"], data.get("markers"),
                   data.get("rms_px"))

    def __repr__(self):
        extra = "" if self.rms_px is None else f", rms={self.rms_px:.2f}px"
        return f"PosePair({self.name}{extra})"


def load_session(directory):
    """Every `pose_*.json` in a `capture_session.py` directory, in order.

    Reads the per-pose files rather than the session header, because those are
    written one at a time and survive an interrupted session -- the header is
    rewritten on every capture and a Ctrl-C during that write is exactly the
    case `save_session` is atomic for.
    """
    paths = sorted(glob.glob(os.path.join(directory, "pose_*.json")))
    if not paths:
        raise ValueError(f"no pose_*.json in {directory}")

    samples = []
    for path in paths:
        with open(path, encoding="utf-8") as handle:
            samples.append(json.load(handle))
    return samples


def load_session_meta(directory):
    """The session header, when there is one -- frames and camera settings."""
    try:
        names = sorted(os.listdir(directory))
    except OSError:
        return {}
    for name in names:
        if not name.endswith(".json") or name.startswith("pose_"):
            continue
        try:
            with open(os.path.join(directory, name), encoding="utf-8") as handle:
                data = json.load(handle)
        except (OSError, ValueError):
            continue
        if isinstance(data, dict) and "samples" in data:
            return data
    return {}


def pairs_from_session(directory, spec, intrinsics, min_markers=6,
                       max_rms_px=3.0):
    """Run PnP on every image in the session, keeping the ones worth keeping.

    Returns `(pairs, skipped)`. Skipping is reported rather than silent: a pose
    that failed to detect is information about the session, and half a session
    quietly dropped is how a hand-eye calibration ends up solved from five poses
    nobody chose.
    """
    pairs, skipped = [], []
    for sample in load_session(directory):
        name = sample.get("name", "?")
        image_name = sample.get("image")
        if not image_name:
            skipped.append((name, "没有 image 字段"))
            continue
        image = cv2.imread(os.path.join(directory, image_name))
        if image is None:
            skipped.append((name, f"读不到 {image_name}"))
            continue

        result = detect(image, intrinsics.camera_matrix, intrinsics.dist_coeffs,
                        spec, min_markers=min_markers)
        if result is None:
            skipped.append((name, f"检出不足 {min_markers} 个 marker"))
            continue
        if result.reproj_rms_px > max_rms_px:
            skipped.append((name, f"重投影 {result.reproj_rms_px:.2f} px "
                                  f"> {max_rms_px}"))
            continue

        pairs.append(PosePair(name, sample["T_base_tool"], result.matrix,
                              len(result.marker_ids), result.reproj_rms_px))
    return pairs, skipped


def load_pairs(path):
    """Read a `--save-pairs` file back."""
    with open(path, encoding="utf-8") as handle:
        data = json.load(handle)
    pairs = [PosePair.from_dict(item) for item in data["pairs"]]
    return pairs, data


# -- the maths ------------------------------------------------------------

def solve(pairs, method="TSAI"):
    """`X = T_end <- camera_optical`, from the poses.

    `calibrateHandEye` wants the camera's pose in the gripper frame, the target's
    pose in the camera frame, and nothing else -- so `T_base_end` goes in as-is
    and `T_cam_board` as-is, and the frame names in the OpenCV documentation
    ("gripper", "target") line up with ours without a transpose anywhere.
    """
    if len(pairs) < 3:
        raise ValueError(f"手眼至少需要 3 个姿态，现在只有 {len(pairs)}")

    R_gripper2base = np.array([p.T_base_end[:3, :3] for p in pairs])
    t_gripper2base = np.array([p.T_base_end[:3, 3].reshape(3, 1)
                               for p in pairs])
    R_target2cam = np.array([p.T_cam_board[:3, :3] for p in pairs])
    t_target2cam = np.array([p.T_cam_board[:3, 3].reshape(3, 1)
                             for p in pairs])

    flag = getattr(cv2, "CALIB_HAND_EYE_" + method.upper())
    R, t = cv2.calibrateHandEye(R_gripper2base, t_gripper2base,
                                R_target2cam, t_target2cam, method=flag)
    X = np.eye(4)
    X[:3, :3] = R
    X[:3, 3] = np.asarray(t).ravel()
    return X


def board_in_base(pairs, X):
    """Where each pose says the board is, in `base_link`.

    Computed one pose at a time and never averaged, because the scatter is the
    measurement. Averaging it away would discard the only check this procedure
    has.
    """
    return [p.T_base_end @ X @ p.T_cam_board for p in pairs]


def scatter(transforms):
    """(max position deviation in mm, max rotation deviation in degrees).

    Both measured against the mean rather than the first pose, so one bad pose
    out of twenty moves the figure instead of defining it.
    """
    positions = np.array([T[:3, 3] for T in transforms])
    rotations = [T[:3, :3] for T in transforms]

    mean_position = positions.mean(axis=0)
    position_mm = float(np.max(np.linalg.norm(positions - mean_position,
                                              axis=1)) * 1000.0)

    mean_rotation = _mean_rotation(rotations)
    rotation_deg = max(rotation_angle_deg(mean_rotation, R) for R in rotations)
    return position_mm, rotation_deg, mean_position, mean_rotation


def _mean_rotation(rotations):
    """The rotation closest to a set of rotations, via the quaternion mean.

    Averaging the matrices entry by entry does not give a rotation, and
    projecting the result back onto SO(3) is a polar decomposition for no
    reason. Quaternions need one sign fix and one eigenvector.
    """
    quaternions = []
    reference = None
    for rotation in rotations:
        q = np.array(quaternion_from_matrix(rotation))
        if reference is None:
            reference = q
        elif float(np.dot(q, reference)) < 0.0:
            q = -q                     # q and -q are the same rotation
        quaternions.append(q)

    # Largest eigenvector of the outer-product sum == the mean direction.
    matrix = np.zeros((4, 4))
    for q in quaternions:
        matrix += np.outer(q, q)
    _, vectors = np.linalg.eigh(matrix)
    x, y, z, w = vectors[:, -1]
    return quaternion_to_matrix(x, y, z, w)


def rotation_spread(pairs):
    """How much the tool turned: `(span, axis)` in degrees, or (None, None).

    Two numbers, because hand-eye needs two different things and either of them
    can be satisfied by data that still cannot be solved.

    `span` is the largest angle between any two tool orientations. The *smallest*
    angle -- the obvious alternative, and what the capture tool uses one pose at
    a time to ask "does this pose add anything?" -- is useless as a batch figure:
    with twenty poses some pair is always close together, so a good session
    reports as a bad one. What matters here is that the set as a whole turned.

    `axis` is how far apart the axes of the relative rotations point, folded to
    one hemisphere because turning about +Z and about -Z is still turning about
    Z. It collapses to zero when every pose turns about a single axis, which is
    the other way this goes wrong and the one `span` cannot see.
    """
    if len(pairs) < 2:
        return None, None
    rotations = [p.T_base_end[:3, :3] for p in pairs]

    span = 0.0
    directions = []
    for i in range(len(rotations)):
        for j in range(i + 1, len(rotations)):
            span = max(span, rotation_angle_deg(rotations[i], rotations[j]))
            rvec = cv2.Rodrigues(rotations[i].T @ rotations[j])[0].ravel()
            angle = float(np.linalg.norm(rvec))
            if angle > math.radians(1.0):
                directions.append(rvec / angle)

    if len(directions) < 2:
        return span, 0.0
    reference = directions[0]
    folded = [axis if float(np.dot(axis, reference)) >= 0.0 else -axis
              for axis in directions]
    axis_spread = 0.0
    for i in range(len(folded)):
        for j in range(i + 1, len(folded)):
            cosine = float(np.clip(np.dot(folded[i], folded[j]), -1.0, 1.0))
            axis_spread = max(axis_spread, math.degrees(math.acos(cosine)))
    return span, axis_spread


def rotation_is_usable(pairs):
    """Whether the poses turned enough for the solver to have a unique answer.

    Returns `(ok, span, axis)`. This is the gate that has to come before
    believing any figure the solver produces -- see `trustworthy`.
    """
    span, axis = rotation_spread(pairs)
    if span is None:
        return False, None, None
    return (span >= MIN_ROTATION_SPREAD_DEG
            and axis >= MIN_ROTATION_SPREAD_DEG), span, axis


# -- reporting ------------------------------------------------------------

def verdict(position_mm, rotation_deg):
    if position_mm <= GOOD_POSITION_MM and rotation_deg <= GOOD_ROTATION_DEG:
        return "好", True
    if position_mm <= OK_POSITION_MM and rotation_deg <= OK_ROTATION_DEG:
        return "可用", True
    return "存疑", False


def trustworthy(pairs, position_mm, rotation_deg):
    """Whether the figures add up to something worth writing into a URDF.

    A small scatter is necessary for a usable X. It is not sufficient, and the
    gap between those two is not academic: with no rotation in the arm poses
    *every* X makes all the poses agree exactly, so the scatter comes out a
    flawless zero for a matrix that is off by sixteen million metres. The
    selftest builds that case on purpose. A check that can be talked into total
    confidence is worse than no check, so the rotation gate decides this too.
    """
    _, figures_ok = verdict(position_mm, rotation_deg)
    turned_enough, _, _ = rotation_is_usable(pairs)
    return figures_ok and turned_enough


def report_data(pairs, skipped):
    print(" ① 数据")
    print(f"     姿态      {len(pairs)} 组"
          + (f"，跳过 {len(skipped)} 组" if skipped else ""))
    rms = [p.rms_px for p in pairs if p.rms_px is not None]
    markers = [p.markers for p in pairs if p.markers is not None]
    if rms:
        print(f"     重投影    中位 {sorted(rms)[len(rms) // 2]:.2f} px，"
              f"最大 {max(rms):.2f} px")
    if markers:
        print(f"     marker    最少 {min(markers)} 个")
    spread = rotation_spread(pairs)
    if spread[0] is not None:
        turned_enough, span, axis = rotation_is_usable(pairs)
        print(f"     姿态跨度  最大两两相差 {span:.1f}°    "
              f"[{'够' if span >= MIN_ROTATION_SPREAD_DEG else '偏小'}]", )
        print(f"     转轴分散  {axis:.1f}°    "
              f"[{'够' if axis >= MIN_ROTATION_SPREAD_DEG else '都绕着一根轴转'}]")
    for name, reason in skipped:
        print(f"     ⊘ {name}  {reason}")
    print()


def report_consistency(pairs, X):
    """The headline check: does every pose agree on where the board is?

    Returns `(position_mm, rotation_deg, trustworthy)`, and the third value is
    deliberately not just the verdict on the first two -- see `trustworthy`.
    """
    transforms = board_in_base(pairs, X)
    position_mm, rotation_deg, mean_position, _ = scatter(transforms)
    label, _ = verdict(position_mm, rotation_deg)
    turned_enough, span, axis = rotation_is_usable(pairs)

    print(" ③ 板子在 base_link 下（应该处处相同）")
    print(f"     平均位置  ({mean_position[0]:+.4f}, {mean_position[1]:+.4f}, "
          f"{mean_position[2]:+.4f}) m")
    print(f"     最大偏离  {position_mm:.1f} mm      "
          f"[{'好' if position_mm <= GOOD_POSITION_MM else '偏大'}]")
    print(f"     最大转角  {rotation_deg:.2f}°      "
          f"[{'好' if rotation_deg <= GOOD_ROTATION_DEG else '偏大'}]")
    if not turned_enough:
        print(f"     姿态转动  跨度 {span:.1f}°，转轴分散 {axis:.1f}°"
              f"  ⚠️ 低于 {MIN_ROTATION_SPREAD_DEG:.0f}°")
        print("        散布小在这里不代表解是对的：姿态没有差异时，任意 X")
        print("        都能让它们一致。这个外参不可信，得回去重采。")
    print(f"     结论      "
          + (f"可信（{label}）" if trustworthy(pairs, position_mm,
                                                rotation_deg)
             else "不可信 —— 先别写进 URDF"))

    # Name the worst pose. "0.8 mm" is a diagnosis; "pose_013" is something to
    # go and look at.
    if pairs:
        offsets = [float(np.linalg.norm(T[:3, 3] - mean_position))
                   for T in transforms]
        worst = int(np.argmax(offsets))
        if offsets[worst] > GOOD_POSITION_MM / 1000.0:
            print(f"     最差      {pairs[worst].name}"
                  f"（偏 {offsets[worst] * 1000:.1f} mm）")
    print()
    return position_mm, rotation_deg


def report_methods(pairs, chosen):
    """Solve with every method and compare. Cheap, and a strong signal."""
    chosen = chosen.upper()
    print(" ④ 五种求解方法对比")
    results = {}
    for name in METHOD_NAMES:
        try:
            results[name] = solve(pairs, name)
        except cv2.error as error:
            print(f"     {name:<11} 失败  {str(error).splitlines()[0][:50]}")
    if chosen not in results:
        print()
        return results

    reference = results[chosen]
    for name, X in results.items():
        gap = float(np.linalg.norm(X[:3, 3] - reference[:3, 3]) * 1000.0)
        angle = rotation_angle_deg(X[:3, :3], reference[:3, :3])
        mark = " ←用的" if name == chosen else ""
        print(f"     {name:<11} 与 {chosen} 差 {gap:7.2f} mm  "
              f"{angle:6.3f}°{mark}")
    print()
    return results


def report_urdf(X_optical, optical_rpy):
    """Both frames, because the difference between them is the classic 90."""
    optical_from_link = matrix_from_rotation(rpy_to_matrix(optical_rpy))
    X_link = X_optical @ optical_from_link

    print(" ⑤ 外参")
    position, rotation = describe(X_optical)
    print("     T_press_tool_link ← 光学系（求解器原始输出）")
    print(f"       平移   ({position[0]:+.4f}, {position[1]:+.4f}, "
          f"{position[2]:+.4f}) m")
    print(f"       四元数 {_quaternion_text(rotation)}")

    position, rotation = describe(X_link)
    rpy = matrix_to_rpy(rotation)
    print()
    print("     T_press_tool_link ← wrist_depth_camera_link（写进 URDF 的）")
    print(f"       平移   ({position[0]:+.4f}, {position[1]:+.4f}, "
          f"{position[2]:+.4f}) m")
    print(f"       rpy    ({rpy[0]:+.6f}, {rpy[1]:+.6f}, {rpy[2]:+.6f}) rad  "
          f"= ({math.degrees(rpy[0]):+.2f}, {math.degrees(rpy[1]):+.2f}, "
          f"{math.degrees(rpy[2]):+.2f})°")
    print()
    print("     wrist_depth_camera.xacro 里 wrist_depth_camera_joint 的 "
          "<origin>：")
    print(f'       <origin xyz="{position[0]:.6f} {position[1]:.6f} '
          f'{position[2]:.6f}"')
    print(f'               rpy="{rpy[0]:.6f} {rpy[1]:.6f} {rpy[2]:.6f}" />')
    print()
    print("     ⚠️ 上面这个 origin 是 T_parent ← wrist_depth_camera_link。")
    print("        如果 URDF 里这个 joint 的 parent 不是 press_tool_link，")
    print("        还要再乘一段已知的固定变换，不能直接粘。")
    print()
    return X_link


def _quaternion_text(rotation):
    x, y, z, w = quaternion_from_matrix(rotation)
    return f"({x:+.6f}, {y:+.6f}, {z:+.6f}, {w:+.6f})"


# -- main -----------------------------------------------------------------

def parse_rpy(text):
    parts = [part for part in text.replace(",", " ").split() if part]
    if len(parts) != 3:
        raise argparse.ArgumentTypeError("需要三个数：roll pitch yaw")
    return tuple(float(part) for part in parts)


def main():
    parser = argparse.ArgumentParser(description="从一组机械臂姿态解出相机在工具上的位置")
    parser.add_argument("--session", help="capture_session.py 的输出目录")
    parser.add_argument("--pairs", help="已经算好的对应关系 JSON")
    parser.add_argument("--camera-info", default="",
                        help="内参文件；--session 模式下必填")
    parser.add_argument("--config", default=DEFAULT_BOARD, help="board.yaml")
    parser.add_argument("--method", default="TSAI",
                        choices=[name.lower() for name in METHOD_NAMES])
    parser.add_argument("--optical-rpy", type=parse_rpy,
                        default=OPTICAL_TO_LINK_RPY,
                        help="光学系到机械系的固定旋转，默认取自 xacro")
    parser.add_argument("--min-markers", type=int, default=6)
    parser.add_argument("--max-rms-px", type=float, default=3.0)
    parser.add_argument("--check-only", action="store_true",
                        help="只体检数据，不解")
    parser.add_argument("--save-pairs", default="",
                        help="把解出来的对应关系存成 JSON")
    parser.add_argument("--json", default="", help="把结果写成 JSON")
    args = parser.parse_args()

    if not args.session and not args.pairs:
        parser.error("要么 --session，要么 --pairs")
    if args.session and not args.camera_info:
        parser.error("--session 模式需要 --camera-info")

    print()
    print("═" * 68)
    print(" 手眼标定")
    print("═" * 68)
    print()

    # -- gather ----------------------------------------------------------
    if args.pairs:
        pairs, stored = load_pairs(resolve_config_path(args.pairs))
        meta = stored
        skipped = []
    else:
        session = os.path.expanduser(args.session)
        meta = load_session_meta(session)
        spec = BoardSpec.from_yaml(resolve_config_path(args.config))
        intrinsics = CameraIntrinsics.from_file_or_directory(
            resolve_config_path(args.camera_info))

        print(f" 板子    {spec}")
        print(f" 内参    {intrinsics}")
        camera = meta.get("camera") or {}
        if camera:
            print(f" 采集    {camera.get('device')}  {camera.get('width')}x"
                  f"{camera.get('height')} @{camera.get('fps')}")
            if (camera.get("width"), camera.get("height")) != \
                    tuple(intrinsics.image_size or ()):
                print("         ⚠️ 采集分辨率与内参记录的不一致 —— 这正是 "
                      "marker_locator\n"
                      "           会换算并告警的那种情况，先确认哪个是对的")
        print(f" 坐标系  {meta.get('base_frame', 'base_link')} ← "
              f"{meta.get('tool_frame', 'press_tool_link')}")
        print()
        pairs, skipped = pairs_from_session(
            session, spec, intrinsics, args.min_markers, args.max_rms_px)

    report_data(pairs, skipped)

    if len(pairs) < 3:
        print(" 姿态少于 3 组，手眼无解。先回去采数据。")
        return 1

    if args.check_only:
        turned_enough, span, axis = rotation_is_usable(pairs)
        if not turned_enough:
            print(f" ⚠️ 姿态转动不够：跨度 {span:.1f}°，转轴分散 {axis:.1f}°"
                  f"（需要都 ≥ {MIN_ROTATION_SPREAD_DEG:.0f}°）。")
            print("    手眼需要机械臂**转过**，而且不是绕着一根轴转。")
            return 1
        print(" 数据形状没问题，可以解。")
        return 0

    # -- solve -----------------------------------------------------------
    print(" ② 求解")
    X_optical = solve(pairs, args.method)
    print(f"     {args.method} 解出，见 ⑤")
    print()

    report_consistency(pairs, X_optical)
    results = report_methods(pairs, args.method)
    X_link = report_urdf(X_optical, args.optical_rpy)

    position_mm, rotation_deg = scatter(board_in_base(pairs, X_optical))[:2]
    if not trustworthy(pairs, position_mm, rotation_deg):
        print(" ⚠️ 一致性检查没通过，这个外参先别写进 URDF。")
        print("    通常按这个顺序查：")
        print("      · 相机内参对不对（跑 check_intrinsics.py）")
        print("      · 姿态够不够散（至少两个方向各转过 30°）")
        print("      · 有没有姿态的板子位姿是错的（看上面点名的那个）")
        print()
        return 1

    if args.save_pairs:
        payload = dict(meta)
        payload.pop("samples", None)
        payload["pairs"] = [p.to_dict() for p in pairs]
        with open(args.save_pairs, "w", encoding="utf-8") as handle:
            json.dump(payload, handle, indent=2, ensure_ascii=False)
        print(f" 对应关系已存 {args.save_pairs}")

    if args.json:
        payload = {
            "method": args.method,
            "pairs": len(pairs),
            "consistency": {"position_mm": position_mm,
                            "rotation_deg": rotation_deg},
            "T_tool_camera_optical": X_optical.tolist(),
            "T_tool_camera_link": X_link.tolist(),
            "methods": {name: X.tolist() for name, X in results.items()},
        }
        with open(args.json, "w", encoding="utf-8") as handle:
            json.dump(payload, handle, indent=2)
        print(f" 结果已存 {args.json}")

    print(" 下一步：把上面那个 <origin> 写进 wrist_depth_camera.xacro，"
          "然后\n"
          "         marker_locator 加 target_frame:=base_link 就能用了。")
    print()
    return 0


if __name__ == "__main__":
    sys.exit(main())
