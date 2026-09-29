#!/usr/bin/env python3
"""Render the ArUco board at exact physical size, ready to print.

The layout is drawn here rather than by `cv2.aruco.drawPlanarBoard` so that it
matches `BoardSpec.marker_centre` by construction -- same row/column order, same
origin. A board drawn one way and interpreted another is a bug that produces a
plausible-looking pose, which is the worst kind.

Printing at 100% scale is not accurate enough on its own
--------------------------------------------------------
Home printers scale by roughly a percent, which on a 110 mm board is a whole
millimetre. `--scale-check` (on by default) draws a line that is exactly
100.0 mm long below the board: measure it with a ruler first. If it does not
measure 100 mm, the print scale is off and `board.yaml` must be corrected from
the printed markers rather than from the design numbers.

After printing, measure one marker's side with calipers and put the measured
value into `config/board.yaml` as `marker_length`.
"""

import argparse
import os
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from aruco_board import BoardSpec                        # noqa: E402

DEFAULT_CONFIG = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "config", "board.yaml")


def render_board(spec, dpi, margin_m=0.010, scale_check=True,
                 ruler_length=0.100):
    """Return (image, pixels_per_metre). Image is greyscale, exact size."""
    pixels_per_metre = dpi / 0.0254

    board_w_px = int(round(spec.width * pixels_per_metre))
    board_h_px = int(round(spec.height * pixels_per_metre))
    margin_px = int(round(margin_m * pixels_per_metre))
    marker_px = int(round(spec.marker_length * pixels_per_metre))

    # The ruler gets a strip of its own so it never overlaps a marker.
    strip_px = int(round(0.014 * pixels_per_metre)) if scale_check else 0
    canvas_w = board_w_px + 2 * margin_px
    canvas_h = board_h_px + 2 * margin_px + strip_px

    canvas = np.full((canvas_h, canvas_w), 255, dtype=np.uint8)

    dictionary = spec.dictionary()
    for row in range(spec.markers_y):
        for column in range(spec.markers_x):
            marker_id = spec.marker_id(column, row)
            if not spec.is_accepted(marker_id):
                continue

            centre = spec.marker_centre(column, row)
            # Board frame -> canvas pixels. +Y is up in the board frame and
            # down in an image, hence the sign flip.
            cx = canvas_w / 2.0 + centre[0] * pixels_per_metre
            cy = margin_px + board_h_px / 2.0 - centre[1] * pixels_per_metre

            marker = cv2.aruco.drawMarker(dictionary, marker_id, marker_px,
                                          borderBits=1)
            x0 = int(round(cx - marker_px / 2.0))
            y0 = int(round(cy - marker_px / 2.0))
            canvas[y0:y0 + marker_px, x0:x0 + marker_px] = marker

    if scale_check:
        canvas = _draw_scale_check(canvas, pixels_per_metre,
                                   board_h_px + 2 * margin_px, ruler_length)
    return canvas, pixels_per_metre


def _draw_scale_check(canvas, pixels_per_metre, top, ruler_length):
    """Draw a line of exactly `ruler_length` metres, plus end ticks."""
    length_px = int(round(ruler_length * pixels_per_metre))
    x0 = (canvas.shape[1] - length_px) // 2
    y = top + (canvas.shape[0] - top) // 2

    ink = 0
    cv2.line(canvas, (x0, y), (x0 + length_px, y), ink, 2, cv2.LINE_AA)
    for fraction in (0.0, 0.5, 1.0):
        x = x0 + int(round(length_px * fraction))
        cv2.line(canvas, (x, y - 8), (x, y + 8), ink, 2, cv2.LINE_AA)

    label = f"{ruler_length * 1000:.1f} mm"
    cv2.putText(canvas, label, (x0 - 70, y + 5), cv2.FONT_HERSHEY_SIMPLEX,
                0.42, ink, 1, cv2.LINE_AA)
    cv2.putText(canvas, "measure this", (x0 + length_px // 2 - 44, y + 26),
                cv2.FONT_HERSHEY_SIMPLEX, 0.36, ink, 1, cv2.LINE_AA)
    return canvas


def save(image, path, dpi):
    """Write the PNG, embedding the DPI so a print dialog defaults to scale."""
    try:
        from PIL import Image
    except ImportError:
        if not cv2.imwrite(path, image):
            raise RuntimeError(f"could not write {path}")
        print(f"  note: Pillow missing, {os.path.basename(path)} has no DPI "
              f"tag -- print it at {dpi} DPI manually.", file=sys.stderr)
        return
    # Board images are black and white, so the BGR/RGB swap is invisible; the
    # conversion keeps Pillow from writing a single-channel RGBA mismatch.
    Image.fromarray(image).convert("RGB").save(path, dpi=(dpi, dpi))


def main():
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--config", default=DEFAULT_CONFIG,
                        help="board definition (default: the package's board.yaml)")
    parser.add_argument("--out", default="aruco_board.png")
    parser.add_argument("--dpi", type=int, default=300,
                        help="print resolution; 300 gives ~0.085 mm per pixel")
    parser.add_argument("--margin-mm", type=float, default=10.0,
                        help="white surround; ArUco needs a quiet zone")
    parser.add_argument("--no-scale-check", action="store_true",
                        help="omit the 100 mm reference line")
    args = parser.parse_args()

    spec = BoardSpec.from_yaml(args.config)
    image, pixels_per_metre = render_board(
        spec, args.dpi, margin_m=args.margin_mm / 1000.0,
        scale_check=not args.no_scale_check)
    save(image, args.out, args.dpi)

    height, width = image.shape[:2]
    print(f"板子      {spec}")
    print(f"标称尺寸  {spec.width * 1000:.1f} x {spec.height * 1000:.1f} mm"
          f"  (marker {spec.marker_length * 1000:.1f} mm, "
          f"间隙 {spec.marker_separation * 1000:.1f} mm)")
    print(f"画布      {width} x {height} px @ {args.dpi} dpi  "
          f"= {width / pixels_per_metre * 1000:.1f} x "
          f"{height / pixels_per_metre * 1000:.1f} mm")
    print(f"输出      {args.out}")
    print()
    print("打印时把缩放设成 **100% / 实际大小**，不要用「适应页面」。")
    print()
    print("印好之后，按顺序做这三件事：")
    print("  1. 量图下方那根刻度线 —— 应该是 100.0 mm。不是就说明打印缩放不对。")
    print("  2. 拿卡尺量一个 marker 的边长，填回 board.yaml 的 marker_length。")
    print("  3. 量相邻两个 marker 之间的空隙，填回 marker_separation。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
