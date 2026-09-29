#!/usr/bin/env python3
"""Prove the hand-eye solver works, without a camera, an arm, or a board.

Why this can be done with nothing in the room
---------------------------------------------
Hand-eye calibration is `AX = XB`: given the arm poses and the board poses,
find X. Testing it needs the same two inputs -- and they can be *manufactured*.
Pick an X, pick a board position, pick some tool poses, and the board pose in
the camera follows from the algebra:

    T_cam_board = inv(X) @ inv(T_base_end) @ T_base_board

Feed those back in and the solver must return the X that was picked. No camera,
no robot, no images. When the real Orbbec arrives nothing here changes except
where the numbers come from.

What is actually being tested
-----------------------------
Not the recovery -- that is arithmetic and it either works or it does not. The
interesting half is the failures, because they are the ones that reach the
robot. In particular this file injects the mistake that was made for real in
September 2026: a camera calibrated badly enough that every distance came out
2.86 times too long. That calibration had a reprojection error of 0.128 px and
looked perfect. Feeding it into a hand-eye solve is how you find out it was not
-- the board cannot be in two places at once, and the disagreement is the only
signal there is.

`observation()` models that the honest way: it projects the corners through one
camera and solves with another. Scaling the answer by 2.86 would be the obvious
thing to write and it would be wrong, because a focal error stretches depth and
leaves lateral offsets alone, and it also tilts the board it recovers.

Run it with no arguments.
"""

import math
import os
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from aruco_board import BoardSpec, resolve_config_path     # noqa: E402
from hand_eye import (METHOD_NAMES, MIN_ROTATION_SPREAD_DEG,   # noqa: E402
                      OPTICAL_TO_LINK_RPY, PosePair, board_in_base, invert,
                      matrix_from_rotation, matrix_to_rpy,
                      rotation_angle_deg, rotation_is_usable,
                      rpy_to_matrix, scatter, solve, trustworthy)

#: A plausible wrist camera: 1280x960, a wide-ish lens, mild barrel distortion.
#: Not either of the real cameras on purpose -- a test that only passes for the
#: camera you happen to own is not a test.
IMAGE_SIZE = (1280, 960)
CAMERA_MATRIX = np.array([[980.0, 0.0, 640.0],
                          [0.0, 980.0, 480.0],
                          [0.0, 0.0, 1.0]])
DIST_COEFFS = np.array([-0.18, 0.06, 0.0008, -0.0006, 0.0])

#: The extrinsic the solver has to find, taken from the design value already in
#: `wrist_depth_camera.xacro`. Using a real number rather than a random one
#: keeps the intermediate figures recognisable, and it is the value the test
#: suite is supposed to disagree with once a real calibration exists.
DESIGN_XYZ = (-0.035, 0.0, 0.07)


def design_extrinsic():
    """`T_press_tool_link <- optical`, from the URDF's design values."""
    tool_from_link = matrix_from_rotation(
        rpy_to_matrix((0.0, -math.pi / 2.0, 0.0)), DESIGN_XYZ)
    link_from_optical = matrix_from_rotation(
        rpy_to_matrix(OPTICAL_TO_LINK_RPY))
    return tool_from_link @ invert(link_from_optical)


# -- synthesising a session ----------------------------------------------

def board_on_table():
    """The board as it would sit: flat, printed side up, off to one side."""
    matrix = np.eye(4)
    matrix[:3, :3] = cv2.Rodrigues(
        np.array([0.0, 0.08, math.radians(25.0)]))[0]
    matrix[:3, 3] = [0.28, 0.02, 0.03]
    return matrix


def nominal_board_in_camera(distance=0.42):
    """The board 42 cm in front of the camera, square to it.

    Board +Z points out of the printed face, camera +Z points away from the
    lens, so "square to the camera" is a 180 degree turn about X. Getting that
    backwards renders the board from behind -- see the same note in
    `selftest_offline.py`, where it cost a few rounds.
    """
    matrix = np.eye(4)
    matrix[:3, :3] = np.diag([1.0, -1.0, -1.0])
    matrix[:3, 3] = [0.0, 0.0, distance]
    return matrix


def is_fully_visible(T_cam_board, spec, camera_matrix, dist_coeffs,
                     image_size=IMAGE_SIZE, margin=0.04):
    """Would the capture tool have kept this pose?

    A pose with the board half out of frame is one nobody would have collected,
    and including it would flatter the solver by giving it an easy geometry.
    """
    if T_cam_board[2, 3] < 0.12:
        return False
    object_points = np.vstack(list(spec.object_points_by_id().values()))
    points, _ = cv2.projectPoints(object_points,
                                  cv2.Rodrigues(T_cam_board[:3, :3])[0],
                                  T_cam_board[:3, 3], camera_matrix,
                                  dist_coeffs)
    points = points.reshape(-1, 2)
    width, height = image_size
    return (points[:, 0].min() > margin * width
            and points[:, 0].max() < (1.0 - margin) * width
            and points[:, 1].min() > margin * height
            and points[:, 1].max() < (1.0 - margin) * height)


def make_pairs(count, X_true, spec, seed=7, translation_only=False,
               camera_matrix=CAMERA_MATRIX):
    """A synthetic session: `count` tool poses that all see one fixed board.

    Draws poses until `count` of them put the whole board in frame, instead of
    accepting whatever came out of the random number generator. Also carries
    `T_cam_board` through to the camera-frame board pose using the same algebra
    the real pipeline produces, so nothing downstream can tell the difference.
    """
    rng = np.random.default_rng(seed)
    T_base_board = board_on_table()
    nominal = nominal_board_in_camera()

    # Put the nominal tool pose exactly where the camera sees the board at 42 cm,
    # then perturb around it. Solving for it here rather than guessing keeps the
    # drawing in a region where the board is visible by construction.
    T_base_end_nominal = (T_base_board @ invert(nominal) @ invert(X_true))

    pairs = []
    attempts = 0
    while len(pairs) < count and attempts < count * 400:
        attempts += 1
        if translation_only:
            # Same orientation every time: the case the solver cannot handle and
            # must therefore recognise.
            rotation = T_base_end_nominal[:3, :3]
        else:
            axis = rng.normal(size=3)
            axis /= np.linalg.norm(axis)
            angle = math.radians(rng.uniform(20.0, 50.0))
            rotation = cv2.Rodrigues(axis * angle)[0] @ T_base_end_nominal[:3, :3]
        translation = (T_base_end_nominal[:3, 3]
                       + rng.uniform(-0.06, 0.06, 3))
        T_base_end = np.eye(4)
        T_base_end[:3, :3] = rotation
        T_base_end[:3, 3] = translation

        T_cam_board = invert(X_true) @ invert(T_base_end) @ T_base_board
        if not is_fully_visible(T_cam_board, spec, camera_matrix, DIST_COEFFS):
            continue
        pairs.append((f"pose_{len(pairs):03d}", T_base_end, T_cam_board))
    return pairs


def observation(T_cam_board, spec, camera_matrix=CAMERA_MATRIX,
                solve_with=None, sigma_px=0.0, rng=None):
    """What solvePnP would report for a board sitting at `T_cam_board`.

    Projects the board's corners through `camera_matrix` -- that projection is
    the photograph -- optionally adds corner noise, then solves again.
    `solve_with` lets the caller hand the solver a camera it was not calibrated
    with, which is the only honest way to model a wrong intrinsic: a focal error
    stretches depth, leaves the lateral offsets alone, and tilts the board it
    recovers, so it is not a uniform scaling of the answer.
    """
    object_points = np.vstack(list(spec.object_points_by_id().values()))
    image_points, _ = cv2.projectPoints(
        object_points, cv2.Rodrigues(T_cam_board[:3, :3])[0],
        T_cam_board[:3, 3], camera_matrix, DIST_COEFFS)
    image_points = image_points.reshape(-1, 2)

    if sigma_px > 0.0 and rng is not None:
        image_points = image_points + rng.normal(0.0, sigma_px,
                                                 image_points.shape)

    matrix = camera_matrix if solve_with is None else solve_with
    ok, rvec, tvec = cv2.solvePnP(object_points, image_points, matrix,
                                  DIST_COEFFS, flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok:
        return None
    try:
        rvec, tvec = cv2.solvePnPRefineLM(object_points, image_points, matrix,
                                          DIST_COEFFS, rvec, tvec)
    except cv2.error:
        pass

    out = np.eye(4)
    out[:3, :3] = cv2.Rodrigues(rvec)[0]
    out[:3, 3] = tvec.ravel()
    return out


def observations(pairs, spec, **kwargs):
    """Push every synthetic pose through `observation`.

    Returns `(pairs, drops)`. `drops` is not hidden: if a camera is wrong enough
    that PnP stops converging on some poses, that is part of the finding.
    """
    out, drops = [], 0
    for name, T_base_end, T_cam_board in pairs:
        seen = observation(T_cam_board, spec, **kwargs)
        if seen is None:
            drops += 1
            continue
        out.append(PosePair(name, T_base_end, seen))
    return out, drops


def pose_error(X, X_true):
    """(translation error in mm, rotation error in degrees)."""
    return (float(np.linalg.norm(X[:3, 3] - X_true[:3, 3]) * 1000.0),
            rotation_angle_deg(X[:3, :3], X_true[:3, :3]))


def consistency_mm(pairs, X):
    return scatter(board_in_base(pairs, X))[:2]


# -- the checks -----------------------------------------------------------

def main():
    spec = BoardSpec.from_yaml(resolve_config_path("config/board.yaml"))
    X_true = design_extrinsic()
    failures = []

    def check(label, ok, detail=""):
        print(f"    [{'PASS' if ok else 'FAIL'}] {label}"
              f"{('  ' + detail) if detail else ''}")
        if not ok:
            failures.append(label)

    print()
    print("═" * 68)
    print(" 手眼标定自测（合成数据，不需要相机/机械臂）")
    print("═" * 68)
    print()
    position, rotation = X_true[:3, 3], X_true[:3, :3]
    roll, pitch, yaw = matrix_to_rpy(rotation)
    print(f" 真值  设计外参 T_tool←optical")
    print(f"       平移 ({position[0]:+.4f}, {position[1]:+.4f}, "
          f"{position[2]:+.4f}) m")
    print(f"       rpy  ({math.degrees(roll):+.2f}, {math.degrees(pitch):+.2f}, "
          f"{math.degrees(yaw):+.2f})°")
    print()

    # -- ① the rpy conversions the URDF output depends on -----------------
    print("  ① rpy 转换（URDF 的 <origin> 用它）")
    round_trips = [(0.0, 0.0, 0.0), (0.3, -0.7, 1.1), (0.2, 1.2, -2.0),
                   (math.pi / 2.0, 0.0, 0.0)]
    ok = all(np.allclose(rpy_to_matrix(matrix_to_rpy(rpy_to_matrix(rpy))),
                         rpy_to_matrix(rpy), atol=1e-12)
             for rpy in round_trips)
    check("rpy → 矩阵 → rpy → 矩阵，往返一致", ok)

    # atol is 1e-9 rather than 1e-12 because the constant is a URDF string that
    # was written rounded -- -1.57079632679, not -pi/2. The recovered angle is
    # the exact quarter turn, so the two differ by about 5e-12 radians. What is
    # being checked is that no axis is lost, not that the literal round-trips.
    locked = OPTICAL_TO_LINK_RPY
    recovered = matrix_to_rpy(rpy_to_matrix(locked))
    check("−90° 俯仰（万向锁）不丢轴",
          np.allclose(rpy_to_matrix(recovered), rpy_to_matrix(locked),
                      atol=1e-9),
          f"角度回读成 ({math.degrees(recovered[0]):.1f}, "
          f"{math.degrees(recovered[1]):.1f}, "
          f"{math.degrees(recovered[2]):.1f})°")
    # At pitch = -90 only the sum roll + yaw is determined, so every split
    # describes the same rotation and the printed angles are a choice. Picking
    # roll = 0 matters because the alternative prints this wrist camera as if it
    # were rolled 55 degrees -- and something that looks wrong gets "fixed".
    gimbal = (0.4, -math.pi / 2.0, 0.7)
    shown = matrix_to_rpy(rpy_to_matrix(gimbal))
    check("万向锁时改印 roll=0 的那个代表（否则会印出 +55° 的假翻滚）",
          abs(shown[0]) < 1e-9 and abs(shown[2] - 1.1) < 1e-9,
          f"(0.4, −90°, 0.7) → ({math.degrees(shown[0]):.1f}, "
          f"{math.degrees(shown[1]):.1f}, {math.degrees(shown[2]):.1f})°")
    check("而这个代表描述的还是同一个旋转",
          np.allclose(rpy_to_matrix(shown), rpy_to_matrix(gimbal), atol=1e-9))
    print()

    # -- ② recovery on clean data -----------------------------------------
    print("  ② 纯恢复：20 个无噪声姿态，应原样解回真值")
    clean = make_pairs(20, X_true, spec)
    check("造出 20 个整块板子都在画面内的姿态", len(clean) == 20,
          f"实际 {len(clean)}")
    clean_pairs, _ = observations(clean, spec)
    X_clean = solve(clean_pairs, "TSAI")
    translation_mm, rotation_deg = pose_error(X_clean, X_true)
    check(f"平移误差 {translation_mm:.2e} mm  < 1e-4", translation_mm < 1e-4)
    check(f"转角误差 {rotation_deg:.2e}°  < 1e-4", rotation_deg < 1e-4)
    position_mm, angle_deg = consistency_mm(clean_pairs, X_clean)
    check(f"板子位置散布 {position_mm:.2e} mm  < 1e-4（无噪声就该是零）",
          position_mm < 1e-4)
    # The gate has to let good data through as well as stop bad data. Getting
    # only the second half right is the easier mistake, and it makes the tool
    # useless in the direction that matters.
    turned_enough, span, axis = rotation_is_usable(clean_pairs)
    check(f"同时通过转动量门槛（跨度 {span:.1f}°，转轴分散 {axis:.1f}°）",
          turned_enough)
    print()

    # -- ③ the five methods -----------------------------------------------
    print("  ③ 五种求解方法应给出同一个答案")
    worst = 0.0
    for name in METHOD_NAMES:
        if name == "TSAI":
            continue
        other = solve(clean_pairs, name)
        worst = max(worst, float(np.linalg.norm(
            other[:3, 3] - X_clean[:3, 3]) * 1000.0))
    check(f"各方法之间最大相差 {worst:.2e} mm  < 1e-4", worst < 1e-4)
    print()

    # -- ④ the optical -> mechanical step ---------------------------------
    print("  ④ 光学系 → 机械系（漏掉会差 90°，而且看着还挺合理）")
    optical_from_link = matrix_from_rotation(rpy_to_matrix(OPTICAL_TO_LINK_RPY))
    X_link = X_clean @ optical_from_link
    mistake = rotation_angle_deg(X_clean[:3, :3], X_link[:3, :3])
    check(f"把求解结果直接当机械系用，会差 {mistake:.1f}°（应约 90°）",
          abs(mistake - 90.0) < 1.0)
    check("乘上 optical_joint 的旋转后，平移不变（只是换个表达）",
          np.allclose(X_link[:3, 3], X_clean[:3, 3], atol=1e-12),
          "两者原点重合，光学系的固定旋转是纯转动")
    print()

    # -- ⑤ a wrong intrinsic, modelled the way it really happens ----------
    print("  ⑤ 内参错了 2.86 倍（同事那份 C270 的真实倍数）")
    wrong = CAMERA_MATRIX.copy()
    wrong[0, 0] *= 2.86
    wrong[1, 1] *= 2.86
    bad_pairs, drops = observations(clean, spec, solve_with=wrong)
    X_bad = solve(bad_pairs, "TSAI")
    translation_mm, rotation_deg = pose_error(X_bad, X_true)
    position_mm, angle_deg = consistency_mm(bad_pairs, X_bad)

    print(f"       解出的平移错 {translation_mm:.1f} mm，"
          f"转角错 {rotation_deg:.2f}°")
    print(f"       板子位置散布 {position_mm:.1f} mm / {angle_deg:.2f}°"
          + (f"（{drops} 个姿态没收敛）" if drops else ""))
    check(f"自检能抓住它：散布 {position_mm:.0f} mm 远超 10 mm 的可用线",
          position_mm > 10.0)
    check("而且解本身也是错的（不是只有自检发现问题）",
          translation_mm > 50.0)

    # The same fit, judged the way a calibration is usually judged. If this
    # passed, the reprojection error would have been a misleading number -- and
    # it did, for real, at 0.128 px.
    rms = _reprojection_rms(clean, spec, wrong)
    print(f"       而重投影误差只有 {rms:.3f} px —— 这个数说不出内参是错的")
    # The threshold is "would this make someone suspicious?", not "is this
    # small". Perfect synthetic data leaves the misfit nowhere to hide, so 1.1 px
    # is what it looks like here; on real images it is smaller and buried in
    # corner noise. The colleague's genuinely broken calibration reported 0.128
    # px, which is *better* than this. The figure does not discriminate.
    check(f"重投影误差 {rms:.3f} px 不会让人起疑，所以它当不了判据",
          rms < 2.0)
    print()

    # -- ⑥ corner noise ----------------------------------------------------
    print("  ⑥ 角点噪声：多少像素的噪声换来多少毫米")
    for sigma in (0.25, 0.5, 1.0):
        rng = np.random.default_rng(1234)
        noisy, _ = observations(clean, spec, sigma_px=sigma, rng=rng)
        X = solve(noisy, "TSAI")
        translation_mm, rotation_deg = pose_error(X, X_true)
        position_mm, _ = consistency_mm(noisy, X)
        print(f"       σ={sigma:.2f} px →  平移错 {translation_mm:6.1f} mm  "
              f"转角错 {rotation_deg:.3f}°  散布 {position_mm:5.2f} mm")
    rng = np.random.default_rng(1234)
    noisy, _ = observations(clean, spec, sigma_px=0.5, rng=rng)
    X_noisy = solve(noisy, "TSAI")
    translation_mm, _ = pose_error(X_noisy, X_true)
    check(f"0.5 px 角点噪声下平移误差 {translation_mm:.1f} mm  < 20 mm",
          translation_mm < 20.0)
    print()

    # -- ⑦ how many poses --------------------------------------------------
    print("  ⑦ 需要多少个姿态（同样 0.5 px 噪声，换姿态数）")
    counts = (3, 5, 10, 20, 30)
    errors = []
    for count in counts:
        rng = np.random.default_rng(99)
        subset = make_pairs(count, X_true, spec, seed=11)
        noisy, _ = observations(subset, spec, sigma_px=0.5, rng=rng)
        if len(noisy) < 3:
            errors.append(float("nan"))
            print(f"       {count:2d} 个姿态 →  解不出来")
            continue
        X = solve(noisy, "TSAI")
        translation_mm, rotation_deg = pose_error(X, X_true)
        errors.append(translation_mm)
        print(f"       {count:2d} 个姿态 →  平移错 {translation_mm:6.2f} mm  "
              f"转角错 {rotation_deg:.3f}°")
    check("姿态越多越好，20 个明显优于 3 个",
          not math.isnan(errors[-1]) and not math.isnan(errors[0])
          and errors[-1] < errors[0])
    check(f"20 个姿态已进入平台期（{errors[3]:.2f} mm，"
          f"30 个是 {errors[4]:.2f} mm）",
          abs(errors[4] - errors[3]) < max(1.0, errors[3]))
    print()

    # -- ⑧ poses that cannot work ------------------------------------------
    print("  ⑧ 只平移、不转动的姿态集")
    flat = make_pairs(20, X_true, spec, translation_only=True)
    if len(flat) < 3:
        print("       造不出足够多的可见姿态，跳过")
    else:
        flat_pairs, _ = observations(flat, spec)
        turned_enough, span, axis = rotation_is_usable(flat_pairs)
        print(f"       姿态跨度 {span:.2f}°，转轴分散 {axis:.2f}°"
              f"（两个都要 ≥ {MIN_ROTATION_SPREAD_DEG:.0f}°）")
        check("数据体检会拦住它", not turned_enough)
        try:
            X_flat = solve(flat_pairs, "TSAI")
            translation_mm, _ = pose_error(X_flat, X_true)
            position_mm, angle_deg = consistency_mm(flat_pairs, X_flat)
            print(f"       求解器照样返回了答案：平移错 {translation_mm:.2e} mm")
            print(f"       而板子位置散布只有 {position_mm:.3f} mm / "
                  f"{angle_deg:.3f}°  ← 看上去完美")
            check(f"解是垃圾：平移错 {translation_mm:.0e} mm，没人会去用它",
                  translation_mm > 1e6)
            check(f"散布却是 {position_mm:.3f} mm —— 光看散布会被骗过去",
                  position_mm < 1.0)
            check("所以散布必须配合姿态转动量一起看（trustworthy 两个都要）",
                  not trustworthy(flat_pairs, position_mm, angle_deg))
            print("       ⚠️ 没有转动时，任意 X 都能让所有姿态精确一致 ——")
            print("          散布为零是**必要不充分**。工具把姿态转动量当硬门槛。")
        except cv2.error as error:
            print(f"       求解器直接拒绝：{str(error).splitlines()[0][:60]}")
            check("求解器拒绝了这个病态数据集", True)
    print()

    # -- verdict -----------------------------------------------------------
    print("═" * 68)
    if failures:
        print(f" 结论：{len(failures)} 项不合格")
        for label in failures:
            print(f"   · {label}")
    else:
        print(" 结论：全部通过")
    print("═" * 68)
    print()
    return 1 if failures else 0


def _reprojection_rms(raw_pairs, spec, camera_matrix):
    """Fit quality the way a calibration tool would report it.

    Only the fit, with no reference to whether the model is right -- which is
    exactly the point being made, so it is computed separately rather than
    borrowed from `observation`.
    """
    object_points = np.vstack(list(spec.object_points_by_id().values()))
    residuals = []
    for _, _, T_cam_board in raw_pairs:
        image_points, _ = cv2.projectPoints(
            object_points, cv2.Rodrigues(T_cam_board[:3, :3])[0],
            T_cam_board[:3, 3], CAMERA_MATRIX, DIST_COEFFS)
        ok, rvec, tvec = cv2.solvePnP(
            object_points, image_points.reshape(-1, 2), camera_matrix,
            DIST_COEFFS, flags=cv2.SOLVEPNP_ITERATIVE)
        if not ok:
            continue
        projected, _ = cv2.projectPoints(
            object_points, rvec, tvec, camera_matrix, DIST_COEFFS)
        delta = projected.reshape(-1, 2) - image_points.reshape(-1, 2)
        residuals.append(np.sum(delta ** 2, axis=1))
    return float(np.sqrt(np.mean(np.concatenate(residuals))))


if __name__ == "__main__":
    sys.exit(main())
