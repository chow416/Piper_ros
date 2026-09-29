#!/usr/bin/env python3
"""Offline end-to-end check: render the board at a known pose, recover it.

No camera, no arm, no calibration. The board image is projected into a synthetic
view with a chosen `T_camera <- board`, distortion included, and then `detect()`
has to find that pose again. Any mistake in the board frame convention shows up
here as a large error rather than as a plausible-looking wrong answer, because
the expected value is known exactly.

This is the step-2 gate from docs/marker_perception_plan.md: get it passing
before touching hardware.
"""

import argparse
import math
import os
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from aruco_board import (BoardSpec, CameraIntrinsics,  # noqa: E402
                         detect, annotate)
from make_board import render_board                   # noqa: E402

#: Matches the DaBai's derived prior at 1920x1080 (fx ~ 1060 px).
IMAGE_SIZE = (1920, 1080)
CAMERA_MATRIX = np.array([[1060.0, 0.0, 960.0],
                          [0.0, 1060.0, 540.0],
                          [0.0, 0.0, 1.0]])
#: A plausible barrel distortion, to prove the pipeline carries distortion
#: through instead of quietly assuming a perfect pinhole.
DIST_COEFFS = np.array([-0.25, 0.08, 0.0012, -0.0008, 0.0])

#: A board printed in the XY plane faces +Z -- towards whoever is looking at the
#: printed side. Camera optical +Z points *away* from the camera, so a board
#: "facing the camera, upright" has its +Z along camera -Z: a 180 degree turn
#: about X.
#:
#: Getting this wrong renders the board from behind. That mirrors every marker
#: pattern, mirrored ArUco patterns are not valid codes, and the detector then
#: finds almost nothing -- a failure that looks like a detector tuning problem
#: and is not one.
BOARD_FACING_CAMERA = np.diag([1.0, -1.0, -1.0])


def facing_camera(rx=0.0, ry=0.0, rz=0.0):
    """Board upright before the camera, then tilted by (rx, ry, rz) degrees.

    The tilt is applied in the camera frame, on top of the facing pose, so the
    angles read the way they would when describing the board's tilt.
    """
    return rotation_from_axis_angle(rx, ry, rz) @ BOARD_FACING_CAMERA


POSITION_TOLERANCE_M = 0.001           # 1 mm
ROTATION_TOLERANCE_DEG = 0.5
RMS_TOLERANCE_PX = 0.5


def look_at(board_position, board_rotation, camera_matrix, dist_coeffs,
            board_image, board_centre_px, pixels_per_metre, image_size,
            supersample=3):
    """Render `board_image` as seen from a camera at the origin.

    Built as an inverse map: for every destination pixel, undo the distortion,
    shoot a ray, intersect the board plane, and read off where that lands in the
    board image. Doing it this way is what lets lens distortion be part of the
    test -- a plain homography cannot express it.

    The map is evaluated at `supersample` times the final resolution and then
    box-filtered down. Without that the sharp black/white cells of a marker get
    point-sampled from a source five times larger and alias badly enough that
    the detector rejects them -- the real lens blurs those edges, and this is
    what stands in for that blur.
    """
    width, height = image_size
    scale = max(1, int(supersample))
    big_matrix = camera_matrix.copy()
    big_matrix[0, 0] *= scale
    big_matrix[1, 1] *= scale
    # The principal point is a pixel index, so it scales about pixel centres.
    big_matrix[0, 2] = (camera_matrix[0, 2] + 0.5) * scale - 0.5
    big_matrix[1, 2] = (camera_matrix[1, 2] + 0.5) * scale - 0.5
    big_size = (width * scale, height * scale)

    us, vs = np.meshgrid(np.arange(big_size[0], dtype=np.float32),
                         np.arange(big_size[1], dtype=np.float32))
    grid = np.stack([us.ravel(), vs.ravel()], axis=1).reshape(-1, 1, 2)

    normalised = cv2.undistortPoints(grid, big_matrix, dist_coeffs)
    rays = np.concatenate([normalised.reshape(-1, 2),
                           np.ones((normalised.shape[0], 1), np.float64)], axis=1)

    normal = board_rotation[:, 2]                       # board +Z, in camera
    denominator = rays @ normal
    numerator = float(normal @ board_position)

    # Pixels whose ray never meets the plane (behind the camera, or parallel).
    valid = np.abs(denominator) > 1e-9
    distance = np.full(rays.shape[0], np.nan)
    distance[valid] = numerator / denominator[valid]
    valid &= distance > 0

    points_camera = rays * distance[:, None]
    points_board = (points_camera - board_position) @ board_rotation
    # Multiplying by R on the right applies R^T, which is what taking a point
    # out of the camera frame and into the board frame requires.

    map_x = board_centre_px[0] + points_board[:, 0] * pixels_per_metre
    map_y = board_centre_px[1] - points_board[:, 1] * pixels_per_metre
    map_x[~valid] = -1
    map_y[~valid] = -1

    warped = cv2.remap(
        board_image,
        map_x.reshape(big_size[1], big_size[0]).astype(np.float32),
        map_y.reshape(big_size[1], big_size[0]).astype(np.float32),
        cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT, borderValue=255)

    if scale > 1:
        warped = cv2.resize(warped, (width, height),
                            interpolation=cv2.INTER_AREA)
    return cv2.cvtColor(warped, cv2.COLOR_GRAY2BGR)


def rotation_from_axis_angle(rx, ry, rz):
    """Degrees about X, Y, Z, applied X then Y then Z."""
    matrix, _ = cv2.Rodrigues(np.array([math.radians(rx), math.radians(ry),
                                        math.radians(rz)], dtype=np.float64))
    return matrix


def pose_error(expected_rotation, expected_position, detection):
    """Return (position error in metres, rotation error in degrees)."""
    position_error = float(np.linalg.norm(
        detection.tvec.ravel() - np.asarray(expected_position).ravel()))

    delta = detection.rotation @ np.asarray(expected_rotation).T
    cosine = (np.trace(delta) - 1.0) / 2.0
    angle = math.degrees(math.acos(max(-1.0, min(1.0, cosine))))
    return position_error, angle


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default=os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        "config", "board.yaml"))
    parser.add_argument("--save-debug", default="",
                        help="write the last synthetic view here")
    args = parser.parse_args()

    spec = BoardSpec.from_yaml(args.config)
    # No ruler in the test render: it would be projected along with the board
    # and land in the frame as a stray line below it.
    board_image, pixels_per_metre = render_board(spec, dpi=300,
                                                 scale_check=False)
    # Board-image pixel of the board-frame origin: centre of the 110 mm square,
    # i.e. offset from the canvas centre by nothing, and down by half the margin.
    margin_px = int(round(0.010 * pixels_per_metre))
    board_h_px = int(round(spec.height * pixels_per_metre))
    board_centre_px = (board_image.shape[1] / 2.0,
                       margin_px + board_h_px / 2.0)

    intrinsics = CameraIntrinsics(CAMERA_MATRIX, DIST_COEFFS, IMAGE_SIZE)
    detector = spec.make_detector()

    failures = []

    def check(label, ok, detail=""):
        print(f"    [{'PASS' if ok else 'FAIL'}] {label}"
              f"{('  ' + detail) if detail else ''}")
        if not ok:
            failures.append(label)

    # -- convention self-checks -------------------------------------------
    print("  ① 板系约定")
    check("marker_id(0,0) == 0（左上角是第一个）",
          spec.marker_id(0, 0) == 0)
    centre = spec.marker_centre(0, 0)
    check(f"左上角 marker 中心在 (−pitch, +pitch) = ({centre[0]:+.3f}, "
          f"{centre[1]:+.3f})",
          np.allclose(centre[:2], [-spec.pitch, spec.pitch]))
    # The board's size is a physical fact that changes when someone prints at a
    # different scale, so assert the relationship that defines it rather than a
    # number. (An earlier version pinned 110 mm and started failing the moment
    # the printer scaled the sheet to 90%.)
    expected_width = (spec.markers_x * spec.marker_length
                      + (spec.markers_x - 1) * spec.marker_separation)
    expected_height = (spec.markers_y * spec.marker_length
                       + (spec.markers_y - 1) * spec.marker_separation)
    check(f"板子尺寸 {spec.width * 1000:.1f} × {spec.height * 1000:.1f} mm "
          f"= {spec.markers_x}L+{spec.markers_x - 1}S，且正方形",
          abs(spec.width - expected_width) < 1e-12
          and abs(spec.height - expected_height) < 1e-12
          and abs(spec.width - spec.height) < 1e-12)
    check("板系原点是几何中心 → 所有 marker 中心关于原点对称",
          np.allclose(sum(spec.marker_centres_by_id().values()),
                      np.zeros(2)))

    # -- pose recovery -----------------------------------------------------
    cases = [
        ("正对，居中 40 cm", facing_camera(), np.array([0.0, 0.0, 0.40])),
        ("倾斜 25°绕X / 15°绕Y，35 cm", facing_camera(25, 15, 0),
         np.array([0.03, -0.02, 0.35])),
        ("偏心 + 三轴都有旋转，50 cm", facing_camera(-12, 20, 9),
         np.array([0.05, 0.04, 0.50])),
    ]

    for index, (name, rotation, position) in enumerate(cases, start=1):
        print(f"  {'②③④'[index - 1]} 位姿恢复：{name}")
        view = look_at(position, rotation, CAMERA_MATRIX, DIST_COEFFS,
                       board_image, board_centre_px, pixels_per_metre,
                       IMAGE_SIZE)

        result = detect(view, CAMERA_MATRIX, DIST_COEFFS, spec,
                        detector=detector)
        if result is None:
            check("检测到板子", False, "detect() 返回 None")
            continue

        position_error, rotation_error = pose_error(rotation, position, result)
        check(f"检出 {len(result.marker_ids)} 个 marker（期望 9）",
              len(result.marker_ids) == 9, f"得到 {len(result.marker_ids)}")
        check(f"位置误差 {position_error * 1000:.3f} mm < "
              f"{POSITION_TOLERANCE_M * 1000:.0f} mm",
              position_error < POSITION_TOLERANCE_M)
        check(f"姿态误差 {rotation_error:.3f}° < "
              f"{ROTATION_TOLERANCE_DEG}°",
              rotation_error < ROTATION_TOLERANCE_DEG)
        check(f"重投影 RMS {result.reproj_rms_px:.3f} px < "
              f"{RMS_TOLERANCE_PX} px",
              result.reproj_rms_px < RMS_TOLERANCE_PX)

        if args.save_debug and index == len(cases):
            cv2.imwrite(args.save_debug,
                        annotate(view, result, CAMERA_MATRIX, DIST_COEFFS))

    # -- rejection behaviour ----------------------------------------------
    print("  ⑤ 该拒绝的时候要拒绝")
    blank = np.full((IMAGE_SIZE[1], IMAGE_SIZE[0], 3), 255, np.uint8)
    check("空白画面 → None", detect(blank, CAMERA_MATRIX, DIST_COEFFS, spec,
                                    detector=detector) is None)

    # Far away the board is small enough that markers start dropping out. That
    # is exactly why the node needs a distance and a marker-count gate, so this
    # checks the mechanism rather than pretending detection simply fails.
    far = look_at(np.array([0.0, 0.0, 3.0]), facing_camera(), CAMERA_MATRIX,
                  DIST_COEFFS, board_image, board_centre_px, pixels_per_metre,
                  IMAGE_SIZE)
    far_result = detect(far, CAMERA_MATRIX, DIST_COEFFS, spec, detector=detector)
    found = 0 if far_result is None else len(far_result.marker_ids)
    print(f"        3 m 处检出 {found}/9 —— 精度已经退化，需要距离门")
    check(f"3 m 处确实退化（{found}/9）", found < 9)

    gated = detect(far, CAMERA_MATRIX, DIST_COEFFS, spec, detector=detector,
                   min_markers=9)
    check("min_markers=9 时同一个画面被拒 → 质量门有效", gated is None)

    # Board well outside the field of view: at 0.5 m the horizontal half-angle
    # covers about 0.45 m, so a board centred 0.8 m off-axis is not in frame at
    # all and every ray misses its plane.
    outside = look_at(np.array([0.8, 0.0, 0.5]), facing_camera(), CAMERA_MATRIX,
                      DIST_COEFFS, board_image, board_centre_px,
                      pixels_per_metre, IMAGE_SIZE)
    check("板子完全在画面外 → None",
          detect(outside, CAMERA_MATRIX, DIST_COEFFS, spec,
                 detector=detector) is None)

    # Board mounted face-down. This is the failure that does not announce
    # itself: eight of the nine markers stop decoding because their mirrored
    # patterns are not valid codes, and the single survivor is enough for
    # solvePnP to return a pose that faces the right way with a low reprojection
    # error -- and is wrong. Nothing about the returned pose reveals it, which
    # is why the marker count is the gate that matters.
    face_down = rotation_from_axis_angle(0, 180, 0) @ BOARD_FACING_CAMERA
    backwards = look_at(np.array([0.0, 0.0, 0.40]), face_down, CAMERA_MATRIX,
                        DIST_COEFFS, board_image, board_centre_px,
                        pixels_per_metre, IMAGE_SIZE)
    back_result = detect(backwards, CAMERA_MATRIX, DIST_COEFFS, spec,
                         detector=detector)
    back_found = 0 if back_result is None else len(back_result.marker_ids)
    back_rms = 0.0 if back_result is None else back_result.reproj_rms_px
    print(f"        贴反时只剩 {back_found}/9 个 marker，"
          f"但解出的位姿看起来完全正常（rms {back_rms:.2f}px）")
    check(f"板子贴反 → marker 数崩塌（{back_found}/9），这是唯一可靠的信号",
          back_found <= 2)
    check("min_markers=4 时贴反的板子被拒 ← 真正起作用的门",
          detect(backwards, CAMERA_MATRIX, DIST_COEFFS, spec,
                 detector=detector, min_markers=4) is None)

    print()
    if failures:
        print(f"  FAILURES: {failures}")
        return 1
    print("  全部通过")
    return 0


if __name__ == "__main__":
    sys.exit(main())
