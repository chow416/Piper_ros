#!/usr/bin/env python3
"""Offline checks for the board-to-button maths. No hardware, no robot.

Two things are being defended here.

The first is the quaternion. `send_press_target.py` is the reference publisher
for `/press/target_pose` and it has already been used to press buttons
successfully, so a target produced by the bridge has to be the *same message*,
not merely an equally valid one. The test imports that file and compares, rather
than re-deriving the algorithm, because a copy of an implementation agrees with
itself and proves nothing.

The second is the sign convention. `position` is the rod tip and `orientation`
+Z is the direction to press, and both are easy to invert in a way that still
produces a well-formed message. A worked example with hand-computed answers
catches that; nothing else does.
"""

import importlib.util
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

from press_targets import (ConfigError, PressTarget,      # noqa: E402
                           PressTargetSet, quaternion_for_direction,
                           quaternion_to_matrix, rotation_with_z_axis,
                           yaml_block)

REFERENCE = os.path.normpath(os.path.join(
    HERE, "..", "..", "piper_elevator_manipulation", "scripts",
    "send_press_target.py"))

FAILURES = []


def check(label, ok, detail=""):
    print(f"  [{'PASS' if ok else 'FAIL'}] {label}"
          f"{('  ' + detail) if detail else ''}")
    if not ok:
        FAILURES.append(label)


def load_reference():
    """Import send_press_target.py without going through a ROS install."""
    spec = importlib.util.spec_from_file_location("send_press_target_ref",
                                                  REFERENCE)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_quaternion_matches_reference():
    print("① 四元数和参考实现逐位一致")
    if not os.path.exists(REFERENCE):
        check(f"找到参考文件 {REFERENCE}", False)
        return
    reference = load_reference()

    directions = [(1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0), (0, 0, 1),
                  (0, 0, -1), (0.577, 0.577, 0.577), (-0.6, 0.5, -0.3),
                  (0.9, 0.1, 0.2), (1, 0.4, -0.2)]
    worst = 0.0
    for direction in directions:
        # The reference is only a rotation for a *unit* input: its y and z
        # columns are built from the raw vector, so a non-unit direction scales
        # them and the result is no longer orthonormal. Its own main()
        # normalises first, so feeding it a unit vector is the fair comparison.
        # Our version normalises internally, which is strictly more forgiving.
        unit = np.asarray(direction, dtype=float)
        unit = unit / np.linalg.norm(unit)
        theirs = reference.quaternion_from_matrix(
            reference.rotation_with_z_axis(list(unit)))
        ours = quaternion_for_direction(direction)
        worst = max(worst, max(abs(a - b) for a, b in zip(theirs, ours)))
    check(f"{len(directions)} 个方向的四元数完全一致（最大差 {worst:.2e}）",
          worst < 1e-12)

    # The value documented in perception_interface.md, so the two are anchored
    # to something written down rather than only to each other.
    q = quaternion_for_direction((1, 0, 0))
    check(f"方向 (1,0,0) -> (0, 0.707107, 0, 0.707107)，得到 "
          f"({q[0]:.6f}, {q[1]:.6f}, {q[2]:.6f}, {q[3]:.6f})",
          all(abs(a - b) < 1e-6 for a, b in
              zip(q, (0.0, 0.7071068, 0.0, 0.7071068))))


def test_worked_example():
    print("② 手工算过的例子")
    # Board taped to a vertical panel at x = 0.30, centre 0.24 m up, facing the
    # arm. Board +Z points at the arm, i.e. along base -X.
    board = np.array([
        [0.0, 0.0, -1.0, 0.30],
        [-1.0, 0.0, 0.0, 0.00],
        [0.0, 1.0, 0.0, 0.24],
        [0.0, 0.0, 0.0, 1.00],
    ])
    # Button 80 mm below the board centre, on the same panel.
    target = PressTarget("red", {"x": 0.0, "y": -0.080, "z": 0.0})

    position, direction = target.resolve(board)
    check(f"杆尖落点 ({position[0]:.4f}, {position[1]:.4f}, "
          f"{position[2]:.4f}) == (0.3000, 0.0000, 0.1600)",
          np.allclose(position, [0.30, 0.0, 0.16], atol=1e-12))
    check(f"按压方向 ({direction[0]:+.3f}, {direction[1]:+.3f}, "
          f"{direction[2]:+.3f}) == (+1, 0, 0)  ← 压进面板",
          np.allclose(direction, [1.0, 0.0, 0.0], atol=1e-12))

    qx, qy, qz, qw = quaternion_for_direction(direction)
    check("四元数 +Z 轴确实是按压方向",
          np.allclose(quaternion_to_matrix(qx, qy, qz, qw)[:, 2], direction,
                      atol=1e-12))

    # A surface *outward* normal is the negation; direction_sign exists for
    # publishers that store one.
    same_position, negative = target.resolve(board, direction_sign=-1.0)
    check("direction_sign=-1 把方向翻过来",
          np.allclose(negative, [-1.0, 0.0, 0.0], atol=1e-12))
    check("direction_sign 不影响落点",
          np.allclose(same_position, position, atol=1e-12))

    # Button above the centre, to catch an axis that is inverted in Y only.
    above = PressTarget("blue", {"x": 0.0, "y": 0.080, "z": 0.0})
    position_above, _ = above.resolve(board)
    check(f"板上方的按键 -> ({position_above[0]:.4f}, "
          f"{position_above[1]:.4f}, {position_above[2]:.4f}) == "
          f"(0.30, 0.0, 0.32)  ← 板系 +Y 对应 base +Z",
          np.allclose(position_above, [0.30, 0.0, 0.32], atol=1e-12))


def test_rotation_is_a_valid_rotation():
    print("③ 构造出的姿态是合法旋转")
    rng = np.random.RandomState(20260929)
    worst_det = 0.0
    worst_z = 0.0
    trials = 0
    for _ in range(500):
        direction = rng.normal(size=3)
        if np.linalg.norm(direction) < 1e-6:
            continue
        trials += 1
        matrix = rotation_with_z_axis(direction)
        worst_det = max(worst_det, abs(np.linalg.det(matrix) - 1.0))
        worst_z = max(worst_z, float(np.linalg.norm(
            matrix @ np.array([0, 0, 1.0])
            - direction / np.linalg.norm(direction))))
    check(f"{trials} 个随机方向行列式都是 +1（最大偏差 {worst_det:.2e}）",
          worst_det < 1e-12)
    check(f"+Z 列就是输入方向（最大偏差 {worst_z:.2e}）", worst_z < 1e-12)


def test_config_and_errors():
    print("④ 配置读取和报错")
    good = PressTargetSet.from_dict({
        "targets": {
            "red": {"taught": True,
                    "tip_in_board": {"x": 0.0, "y": 0.0, "z": 0.0}},
            "blue": {"taught": False,
                     "tip_in_board": [0.0, 0.08, 0.0],
                     "press_axis_in_board": {"x": 0.0, "y": 0.0, "z": -1.0}},
        }})
    check(f"读出 2 个目标: {good.names()}", good.names() == ["blue", "red"])
    check("默认按压轴 = (0,0,-1)",
          np.allclose(good["red"].press_axis_in_board, [0, 0, -1]))
    check("taught 标志被读到", good["red"].taught and not good["blue"].taught)
    check(f"未教示列表 = {good.untaught()}", good.untaught() == ["blue"])
    check("列表形式 [x,y,z] 也能读",
          np.allclose(good["blue"].tip_in_board, [0, 0.08, 0]))

    def fails(label, payload, expect):
        try:
            PressTargetSet.from_dict(payload)
            check(label, False, "没有报错")
        except ConfigError as error:
            check(label, expect in str(error), f"-> {error}")
        except Exception as error:                          # noqa: BLE001
            check(label, False, f"报了 {type(error).__name__}: {error}")

    fails("没有 targets 段 -> ConfigError",
          {"board": {}}, "targets")
    fails("targets 为空 -> ConfigError", {"targets": {}}, "targets")
    fails("方向全零 -> ConfigError",
          {"targets": {"x": {"tip_in_board": [0, 0, 0],
                             "press_axis_in_board": [0, 0, 0]}}}, "zero")
    fails("tip 缺 z -> ConfigError",
          {"targets": {"x": {"tip_in_board": {"x": 0, "y": 0}}}}, "z")

    try:
        good["nope"]
        check("查不存在的目标 -> ConfigError", False, "没有报错")
    except ConfigError as error:
        check("查不存在的目标 -> ConfigError", "nope" in str(error))

    # The axis must come back normalised even when the config is not.
    scaled = PressTargetSet.from_dict({"targets": {"x": {
        "tip_in_board": [0, 0, 0],
        "press_axis_in_board": {"x": 0.0, "y": 0.0, "z": -7.5}}}})
    check("非单位方向会被归一化",
          np.allclose(scaled["x"].press_axis_in_board, [0, 0, -1]))


def test_yaml_block_round_trips():
    print("⑤ 教示输出能贴回配置")
    target = PressTarget("red", (0.000123, -0.080456, 0.000789),
                         (0.0, 0.0, -1.0), taught=True)
    block = yaml_block(target)
    print("    " + block.replace("\n", "\n    "))

    import yaml
    parsed = yaml.safe_load("targets:\n" + block)
    restored = PressTargetSet.from_dict(parsed)["red"]
    check("贴回去后 tip 一致",
          np.allclose(restored.tip_in_board, target.tip_in_board, atol=1e-9))
    check("贴回去后 axis 一致",
          np.allclose(restored.press_axis_in_board,
                      target.press_axis_in_board, atol=1e-9))
    check("taught 变成 true", restored.taught)


def test_real_config():
    print("⑥ 仓库里的 board.yaml 能读")
    path = os.path.join(os.path.dirname(HERE), "config", "board.yaml")
    targets = PressTargetSet.from_yaml(path)
    print(f"    {len(targets)} 个目标: {targets.names()}")
    check("读到 red/blue/green",
          targets.names() == ["blue", "green", "red"])
    check(f"都还没教示（预期，上实机前要跑教示）: {targets.untaught()}",
          len(targets.untaught()) == 3)
    for name in targets.names():
        print(f"    {name:6s} tip={np.round(targets[name].tip_in_board, 4)}")


def main():
    test_quaternion_matches_reference()
    test_worked_example()
    test_rotation_is_a_valid_rotation()
    test_config_and_errors()
    test_yaml_block_round_trips()
    test_real_config()
    print()
    if FAILURES:
        print(f"  FAILURES: {FAILURES}")
        return 1
    print("  全部通过")
    return 0


if __name__ == "__main__":
    sys.exit(main())
