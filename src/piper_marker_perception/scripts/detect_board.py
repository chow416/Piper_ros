#!/usr/bin/env python3
"""Solve the board pose from a saved image.

The offline half of the perception pipeline: no camera, no ROS, no arm. Point it
at a photograph and it reports `T_camera_optical <- board` plus the fit quality,
and optionally writes a picture of what it found.

This is how detector thresholds get tuned. Take a handful of photographs at the
distances and angles the real system will see, run them all through here, and
look at where the marker count and the reprojection error go bad -- which is
faster and far more repeatable than moving an arm around.

The pose is in the camera optical frame. Turning it into `base_link` needs the
hand-eye calibration and therefore needs the robot; that step lives in the ROS
node, not here.
"""

import argparse
import glob
import json
import os
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from aruco_board import (BoardSpec, CameraIntrinsics,  # noqa: E402
                         annotate, detect, resolve_config_path)

#: A name, not a path -- see `resolve_config_path`.
DEFAULT_CONFIG = "config/board.yaml"

IMAGE_SUFFIXES = (".jpg", ".jpeg", ".png", ".bmp", ".tif", ".tiff")


def gather_images(paths):
    """Expand the arguments into a sorted list of image files."""
    found = []
    for path in paths:
        if os.path.isdir(path):
            for name in sorted(os.listdir(path)):
                if name.lower().endswith(IMAGE_SUFFIXES):
                    found.append(os.path.join(path, name))
        elif os.path.isfile(path):
            found.append(path)
        else:
            found.extend(sorted(glob.glob(path)))
    return found


def main():
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("images", nargs="+",
                        help="image files, directories, or globs")
    parser.add_argument("--config", default=DEFAULT_CONFIG)
    parser.add_argument("--camera-info", required=True,
                        help="intrinsics YAML: `calibrateCamera` output or a ROS "
                             "CameraInfo file")
    parser.add_argument("--out", default="",
                        help="directory for annotated debug images")
    parser.add_argument("--min-markers", type=int, default=4,
                        help="fewer than this and the pose is not reported "
                             "(default 4)")
    parser.add_argument("--ransac", action="store_true",
                        help="use solvePnPRansac; slower, tolerant of a stray "
                             "bad corner")
    parser.add_argument("--json", default="",
                        help="write every result to this JSON file")
    parser.add_argument("--quiet", action="store_true")
    args = parser.parse_args()

    spec = BoardSpec.from_yaml(resolve_config_path(args.config))
    intrinsics = CameraIntrinsics.from_file_or_directory(args.camera_info)

    if not args.quiet:
        print(f"板子     {spec}")
        print(f"内参     {intrinsics}")
        print(f"判定门   marker >= {args.min_markers}")
        print()

    images = gather_images(args.images)
    if not images:
        print("error: 没有匹配到任何图片", file=sys.stderr)
        return 1

    if args.out:
        os.makedirs(args.out, exist_ok=True)

    detector = spec.make_detector()
    results = []
    solved = 0

    for path in images:
        image = cv2.imread(path, cv2.IMREAD_COLOR)
        if image is None:
            print(f"  {os.path.basename(path):32s} 读不了", file=sys.stderr)
            continue

        result = detect(image, intrinsics.camera_matrix, intrinsics.dist_coeffs,
                        spec, detector=detector, use_ransac=args.ransac,
                        min_markers=args.min_markers)

        name = os.path.basename(path)
        if result is None:
            print(f"  {name:32s} 未解出（marker 不足 {args.min_markers} 或没找到）")
            results.append({"image": name, "solved": False})
            continue

        solved += 1
        matrix = result.matrix
        t = result.tvec.ravel()
        print(f"  {name:32s} {len(result.marker_ids):2d} markers  "
              f"rms {result.reproj_rms_px:5.2f} px  "
              f"距离 {result.distance * 100:6.1f} cm  "
              f"t = ({t[0]:+.4f}, {t[1]:+.4f}, {t[2]:+.4f}) m")
        results.append({
            "image": name,
            "solved": True,
            "marker_ids": result.marker_ids,
            "reproj_rms_px": result.reproj_rms_px,
            "distance_m": result.distance,
            # Row-major 4x4, the same layout the calibration JSON uses.
            "T_camera_optical_board": matrix.tolist(),
        })

        if args.out:
            debug = annotate(image, result, intrinsics.camera_matrix,
                             intrinsics.dist_coeffs)
            stem = os.path.splitext(name)[0]
            cv2.imwrite(os.path.join(args.out, stem + "_debug.jpg"), debug)

    if not args.quiet:
        print()
        print(f"  解出 {solved}/{len(results)} 张")
        if solved:
            rms = [r["reproj_rms_px"] for r in results if r["solved"]]
            print(f"  重投影 RMS  最小 {min(rms):.2f}  中位 "
                  f"{sorted(rms)[len(rms) // 2]:.2f}  最大 {max(rms):.2f} px")
        if args.out:
            print(f"  调试图在 {args.out}/")

    if args.json:
        with open(args.json, "w", encoding="utf-8") as handle:
            json.dump({
                "board": {
                    "dictionary": spec.dictionary_name,
                    "markers_x": spec.markers_x,
                    "markers_y": spec.markers_y,
                    "marker_length": spec.marker_length,
                    "marker_separation": spec.marker_separation,
                },
                # The intrinsics are recorded because a pose cannot be read
                # without them: the same tvec means two different physical
                # scenes under fx = 1060 and fx = 1386. A results file that
                # omits this looks complete and cannot be checked later.
                "camera": {
                    "info_file": args.camera_info,
                    "image_size": intrinsics.image_size,
                    "camera_matrix": intrinsics.camera_matrix.tolist(),
                    "dist_coeffs": intrinsics.dist_coeffs.tolist(),
                },
                "results": results,
            }, handle, indent=2, ensure_ascii=False)
        if not args.quiet:
            print(f"  结果在 {args.json}")

    return 0 if solved else 2


if __name__ == "__main__":
    sys.exit(main())
