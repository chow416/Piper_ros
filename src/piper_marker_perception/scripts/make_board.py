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


# -- A4 sheet ------------------------------------------------------------

A4_WIDTH_MM = 210.0
A4_HEIGHT_MM = 297.0

#: Droid Sans Fallback ships in the image and covers CJK. The rest are only
#: tried so this still runs on a machine without it.
CJK_FONT_CANDIDATES = (
    "/usr/share/fonts/truetype/droid/DroidSansFallbackFull.ttf",
    "/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc",
    "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
    "/usr/share/fonts/opentype/noto/NotoSerifCJK-Regular.ttc",
)

LATIN_FONT_CANDIDATES = (
    "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
    "/usr/share/fonts/truetype/liberation/LiberationSans-Regular.ttf",
    "/usr/share/fonts/truetype/noto/NotoSansMono-Regular.ttf",
)

#: CJK radicals start around U+2E80, so this separates the two scripts well
#: enough to pick a font per character. Latin punctuation below it is covered by
#: the Latin font, which is the one that has it.
CJK_START = 0x2E80


def _first_existing(paths):
    for path in paths:
        if os.path.exists(path):
            return path
    return None


def find_cjk_font():
    return _first_existing(CJK_FONT_CANDIDATES)


class MixedFont:
    """Latin from one font, CJK from another, chosen character by character.

    The CJK fallback font installed in this image carries no Latin glyphs at
    all. That is what a fallback font is for -- it is only ever reached for
    characters the primary font lacks -- but it means drawing "100.0 mm" with
    it produces a row of boxes, which is exactly what happened the first time
    this sheet was rendered.

    The alternative would be installing a full CJK font (roughly 100 MB) to
    print a handful of labels. Drawing each character with whichever font has
    it costs one extra call per character and works with what is already here.
    """

    def __init__(self, size_px):
        from PIL import ImageFont
        latin_path = _first_existing(LATIN_FONT_CANDIDATES)
        cjk_path = find_cjk_font()
        if latin_path is None and cjk_path is None:
            self.latin = self.cjk = ImageFont.load_default()
            return
        self.latin = ImageFont.truetype(latin_path or cjk_path, size_px)
        self.cjk = (ImageFont.truetype(cjk_path, size_px) if cjk_path
                    else self.latin)

    def font_for(self, character):
        return self.cjk if ord(character) >= CJK_START else self.latin

    def draw(self, canvas, xy, text, fill="black"):
        """Draw `text` at `xy`, returning the x just past the last glyph."""
        x, y = xy
        for character in text:
            font = self.font_for(character)
            canvas.text((x, y), character, font=font, fill=fill)
            x += canvas.textlength(character, font=font)
        return x

    def length(self, canvas, text):
        return sum(canvas.textlength(character, font=self.font_for(character))
                   for character in text)

    def draw_centred(self, canvas, centre_x, y, text, fill="black"):
        self.draw(canvas, (centre_x - self.length(canvas, text) / 2.0, y),
                  text, fill)


def render_a4_page(spec, dpi, margin_m=0.010):
    """The board laid out on an A4 sheet, with the frame annotated on it.

    The annotations are the point of this layout. The board frame's origin and
    axis directions are decided in code and are invisible on a bare print, so a
    board that has been stuck on upside down looks exactly like one that has
    not -- and that mistake is then undetectable from the pose it produces.
    Printing +X, +Y and the origin onto the sheet moves that decision onto the
    object, where it can be checked by looking at it.

    Returns a PIL image, or raises if Pillow is unavailable.
    """
    from PIL import Image, ImageDraw

    pixels_per_mm = dpi / 25.4
    page = Image.new("RGB", (int(round(A4_WIDTH_MM * pixels_per_mm)),
                             int(round(A4_HEIGHT_MM * pixels_per_mm))), "white")
    draw = ImageDraw.Draw(page)

    def font(size_mm):
        return MixedFont(max(8, int(round(size_mm * pixels_per_mm))))

    def mm(value):
        return value * pixels_per_mm

    board_image, _ = render_board(spec, dpi, margin_m=margin_m,
                                  scale_check=False)
    board_px = board_image.shape[0]
    board_left = int(round((page.width - board_px) / 2))
    board_top = int(round(mm(30)))

    # The page is laid out top to bottom in bands, and the cursor below is what
    # keeps them from colliding. The first version placed the legend, the scale
    # line and the notes from three independent guesses at their heights and
    # they all landed on top of each other.
    board_bottom_mm = (board_top + board_px) / pixels_per_mm
    legend_top_mm = board_bottom_mm + 6.0
    scale_line_mm = legend_top_mm + 30.0
    notes_top_mm = scale_line_mm + 12.0

    title_font = font(5.5)
    body_font = font(3.4)
    label_font = font(3.0)

    title = (f"ArUco 标定板  {spec.dictionary_name}  "
             f"{spec.markers_x}×{spec.markers_y}")
    title_font.draw(draw, (int(mm(18)), int(mm(12))), title)
    body_font.draw(draw, (int(mm(18)), int(mm(21))),
                   f"标称 {spec.width * 1000:.1f} × {spec.height * 1000:.1f} mm    "
                   f"marker {spec.marker_length * 1000:.1f} mm    "
                   f"间隙 {spec.marker_separation * 1000:.1f} mm")

    page.paste(Image.fromarray(board_image), (board_left, board_top))

    # Nothing is drawn on the marker grid. The geometric centre of a 3x3 board
    # lands inside the middle marker, so an origin dot and two 28 mm arrows
    # there blind the very thing the sheet is for -- the first version of this
    # did exactly that. The frame is described in the white space below instead.
    legend_top = int(round(mm(legend_top_mm)))
    origin_x = page.width / 2.0 - mm(34)
    origin_y = legend_top + mm(14)
    ink = (200, 0, 0)

    def arrow(start, end):
        import math
        draw.line([start, end], fill=ink, width=3)
        angle = math.atan2(end[1] - start[1], end[0] - start[0])
        head = mm(4.0)
        spread = math.radians(22)
        draw.polygon([
            end,
            (end[0] - head * math.cos(angle - spread),
             end[1] - head * math.sin(angle - spread)),
            (end[0] - head * math.cos(angle + spread),
             end[1] - head * math.sin(angle + spread)),
        ], fill=ink)

    arm = mm(13)
    r = mm(1.5)
    draw.ellipse([origin_x - r, origin_y - r, origin_x + r, origin_y + r],
                 fill=ink)
    arrow((origin_x, origin_y), (origin_x + arm, origin_y))
    arrow((origin_x, origin_y), (origin_x, origin_y - arm))
    label_font.draw(draw, (origin_x + arm + mm(1.5), origin_y - mm(2.6)),
                    "+X 向右", fill=ink)
    label_font.draw(draw, (origin_x + mm(1.5), origin_y - arm - mm(5.5)),
                    "+Y 向上", fill=ink)
    label_font.draw(draw, (origin_x + mm(1.5), origin_y + mm(2.5)),
                    "原点（板子几何中心）", fill=ink)

    # +Z has no direction to point in on a flat sheet, so it gets the usual
    # out-of-page symbol rather than an arrow that would have to lie.
    zx = page.width / 2.0 + mm(46)
    zy = origin_y
    zr = mm(4.0)
    draw.ellipse([zx - zr, zy - zr, zx + zr, zy + zr], outline=ink, width=3)
    draw.ellipse([zx - mm(1.0), zy - mm(1.0), zx + mm(1.0), zy + mm(1.0)],
                 fill=ink)
    label_font.draw(draw, (zx + zr + mm(2), zy - mm(1.6)), "+Z 垂直纸面朝外",
                    fill=ink)

    # Scale check, in its own band below the legend.
    y = int(round(mm(scale_line_mm)))
    length_px = int(round(0.100 * pixels_per_mm))
    x0 = int(round((page.width - length_px) / 2))
    draw.line([(x0, y), (x0 + length_px, y)], fill="black", width=3)
    for fraction in (0.0, 0.5, 1.0):
        x = x0 + int(round(length_px * fraction))
        draw.line([(x, y - mm(2.5)), (x, y + mm(2.5))], fill="black", width=3)
    label_font.draw(draw, (x0 - mm(14), y - mm(2)), "0")
    label_font.draw(draw, (x0 + length_px + mm(3), y - mm(2)), "100 mm")
    body_font.draw_centred(draw, page.width / 2, y + mm(4),
                           "先量这根线：必须是 100.0 mm", fill=(200, 0, 0))

    notes = [
        "打印设置：缩放选「100%」或「实际大小」，绝对不要选「适应页面」。",
        "",
        "印完按顺序做三件事：",
        "  1. 量上面那根线。不是 100.0 mm 就说明打印缩放不对，重新打印。",
        "  2. 用卡尺量一个 marker 的边长，填回 board.yaml 的 marker_length。",
        "  3. 量相邻两个 marker 之间的空隙，填回 marker_separation。",
        "",
        "贴到纸板上时注意：",
        "  · 这一面朝相机，不要贴反。反了的话标记会被镜像，9 个 marker 里",
        "    只会认出 1 个左右 —— 而且解出来的位姿看起来还是「正常」的。",
        "  · 纸面要平整，翘曲会让角点位置失真。",
        "  · 左上角那个 marker 是 ID 0。",
        "",
        "板系（见上方三角标）：原点在板子几何中心，也就是中间那个 marker 的中心；",
        "  +X 向右，+Y 向上，+Z 垂直纸面朝外。贴好后这张纸的正面朝相机。",
    ]
    y_mm = notes_top_mm
    line_step_mm = 5.4
    for line in notes:
        body_font.draw(draw, (int(mm(20)), int(mm(y_mm))), line)
        y_mm += line_step_mm
    if y_mm > A4_HEIGHT_MM:
        raise RuntimeError(
            f"the notes run to {y_mm:.0f} mm, past the {A4_HEIGHT_MM:.0f} mm "
            f"page. Shorten them or reduce line_step_mm.")

    _assert_grid_untouched(page, board_image, board_left, board_top,
                           margin_px=int(round(mm(margin_m * 1000.0))))
    return page


def _assert_grid_untouched(page, board_image, board_left, board_top, margin_px):
    """Fail loudly if anything was drawn on top of the markers.

    This code already made that mistake once: the origin dot and the axis
    arrows were placed at the board centre, which on a 3x3 board is the middle
    of the middle marker. The result looked like a nicely annotated sheet and
    would have shipped a board with one marker blacked out. Comparing the
    pasted region against the source costs one array comparison and makes the
    mistake impossible to repeat silently.

    `margin_px` is the board image's own white border, in pixels. It has to be
    passed in already converted: the caller works in millimetres and this
    function's first version multiplied a metre value by a per-millimetre
    scale, which rounded to zero and made it check the whole board instead of
    the grid.
    """
    top = board_top + margin_px
    bottom = board_top + board_image.shape[0] - margin_px
    left = board_left + margin_px
    right = board_left + board_image.shape[1] - margin_px

    rendered = np.asarray(page)[top:bottom, left:right, 0]
    original = board_image[margin_px:board_image.shape[0] - margin_px,
                           margin_px:board_image.shape[1] - margin_px]
    if rendered.shape != original.shape:
        raise RuntimeError(f"grid region mismatch: {rendered.shape} vs "
                           f"{original.shape}")
    changed = int(np.count_nonzero(rendered != original))
    if changed:
        raise RuntimeError(
            f"{changed} pixels of the marker grid were drawn over by the "
            f"annotations; the board would not detect reliably. Move the "
            f"annotation outside the grid.")


def render_a4(spec, dpi, path, margin_m=0.010):
    """Render the A4 sheet and write it out."""
    page = render_a4_page(spec, dpi, margin_m=margin_m)
    if find_cjk_font() is None:
        print("  note: no CJK font found, the sheet's Chinese will be boxes.",
              file=sys.stderr)
    page.save(path, dpi=(dpi, dpi))


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
    parser.add_argument("--page", choices=("board", "a4"), default="board",
                        help="board: the board alone, tightly cropped (default). "
                             "a4: a whole A4 sheet with the frame axes, the "
                             "origin and printing instructions printed on it")
    args = parser.parse_args()

    spec = BoardSpec.from_yaml(args.config)

    if args.page == "a4":
        render_a4(spec, args.dpi, args.out, margin_m=args.margin_mm / 1000.0)
        print(f"板子      {spec}")
        print(f"A4 页面   {A4_WIDTH_MM:.0f} × {A4_HEIGHT_MM:.0f} mm @ "
              f"{args.dpi} dpi")
        print(f"输出      {args.out}")
        print()
        print("打印时把缩放设成 **100% / 实际大小**，不要用「适应页面」。")
        print("图上已经印了刻度线和注意事项，照着做就行。")
        return 0

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
