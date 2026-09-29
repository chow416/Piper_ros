#!/usr/bin/env python3
"""Screen a camera calibration before trusting it with a pose.

Why this exists
---------------
A calibration can be **wrong and still look perfect**. The number everyone
checks is the reprojection error, and it is the one that cannot catch this: five
distortion coefficients plus a focal length and a principal point have more
freedom than the fit needs, so a solver that has lost its way can drive the
residual down to a tenth of a pixel while the model is unusable outside the
exact pixels it was fitted to.

This script was written for a real one, in `intrinsics.json`:

    fx=2299.2  fy=2664.5   ->   15.9 x 10.3 degrees of view
    k1=-0.607  k2=+61.5  k3=-2092.3
    reprojection 0.128 px, 9 images

Those are three symptoms of one thing. Sixteen degrees of view is a telephoto
lens, and this is a webcam. fx and fy disagree by 16 per cent, which would mean
rectangular pixels. And the three radial terms are enormous, alternate in sign,
and very nearly cancel: at the corner of the frame they sum to -0.0127 while
their magnitudes sum to 0.1020. That cancellation is what a degenerate fit looks
like -- the terms are compensating for each other rather than describing a lens,
which is why the model turns over and goes negative just outside the frame.

Every check below is arithmetic on the numbers alone. The one thing arithmetic
cannot supply is the lens's true field of view, so if you have the datasheet,
pass `--lens` or `--expect-fov-deg` and the script will compare.

Usage
-----
    python3 check_intrinsics.py ../config/camera_info_orbbec_640x480.yaml \
        --lens orbbec
    python3 check_intrinsics.py intrinsics.json --capture-size 1280x960

Exits non-zero when any check fails, so it can be wired into a test.
"""

import argparse
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from aruco_board import CameraIntrinsics, resolve_config_path   # noqa: E402

PASS, WARN, FAIL = "PASS", "WARN", "FAIL"

#: Fields of view from the datasheets. Only used to turn "this focal length is
#: odd" into "distances will read this many times too far", which is the form of
#: the problem that tells you what to do about it.
#:
#: `full_size` is the sensor mode the figure was quoted for, because the figure
#: that survives a change of resolution is not always the one that was quoted.
#: The C270 quotes 60 degrees diagonally and every mode it offers is 4:3, so the
#: diagonal splits the same way at each. The Orbbec quotes 84.3 horizontally on
#: a 16:9 sensor, and 640x480 is that frame with the sides cropped off -- what
#: survives there is the 54 degrees vertically, and the horizontal is down to
#: 67.7. Comparing the cropped axis reports a perfectly good calibration as
#: 1.35x wrong, which is how this line got written twice.
LENSES = {
    "c270": ("Logitech C270",
             {"diagonal_deg": 60.0, "full_size": (1280, 960)}),
    "orbbec": ("Orbbec DaBai DCW2",
               {"vertical_deg": 54.0, "full_size": (1920, 1080)}),
}

#: A calibration is a fit to a set of board poses, and radial distortion is only
#: observable from points far off the optical axis. Under about ten images, or
#: with the board always facing the camera square-on, is used up by the pose
#: degrees of freedom and the distortion terms are left free to wander.
MIN_IMAGES = 10


# -- the radial model ----------------------------------------------------

def radial_terms(dist, r):
    """The three numerator terms of Brown's radial factor, kept separate.

    Separate because it is their *sum* that cancels, and a check on the sum
    alone cannot see that.
    """
    r2 = r * r
    k1, k2 = dist[0], dist[1]
    k3 = dist[4] if len(dist) > 4 else 0.0
    return [k1 * r2, k2 * r2 * r2, k3 * r2 * r2 * r2]


def radial_factor(dist, r):
    """Brown's radial factor at normalised radius `r`, 1.0 when undistorted."""
    if len(dist) == 0:
        return 1.0
    r2 = r * r
    r4 = r2 * r2
    r6 = r4 * r2
    numerator = 1.0 + dist[0] * r2 + dist[1] * r4 + (
        dist[4] * r6 if len(dist) > 4 else 0.0)
    if len(dist) >= 8:                # rational model: k4, k5, k6 divide
        numerator /= 1.0 + dist[5] * r2 + dist[6] * r4 + dist[7] * r6
    return numerator


def frame_radius(intrinsics):
    """The largest normalised radius the frame actually reaches."""
    width, height = intrinsics.image_size
    x = max(intrinsics.cx, width - intrinsics.cx) / intrinsics.fx
    y = max(intrinsics.cy, height - intrinsics.cy) / intrinsics.fy
    return math.hypot(x, y)


def zero_crossing(intrinsics, limit=5.0, steps=200):
    """Where the radial model stops being physical, in frame radii.

    Past a zero crossing the correction folds points through the optical axis,
    which no lens does. A model that crosses just outside the frame was tuned to
    fit the frame and nothing else.
    """
    if len(intrinsics.dist_coeffs) == 0:
        return None
    r_max = frame_radius(intrinsics)
    for step in range(1, steps + 1):
        r = r_max * limit * step / steps
        if radial_factor(intrinsics.dist_coeffs, r) <= 0.0:
            return r / r_max
    return None


# -- reporting -----------------------------------------------------------

class Report:
    def __init__(self):
        self.rows = []
        self.failures = []
        self.warnings = []

    def add(self, level, title, detail=""):
        self.rows.append((level, title, detail))
        if level == FAIL:
            self.failures.append(title)
        elif level == WARN:
            self.warnings.append(title)

    def print(self):
        for level, title, detail in self.rows:
            print(f"    [{level}] {title}")
            for line in detail.splitlines():
                if line:
                    print(f"           {line}")
        print()


def raw_extras(path):
    """The fit statistics, which the model object itself does not carry."""
    import yaml
    try:
        with open(path, encoding="utf-8") as handle:
            data = yaml.safe_load(handle) or {}
    except OSError:
        return {}
    if not isinstance(data, dict):
        return {}
    keys = ("reproj_err_px", "n_images", "image_count", "board_cols",
            "board_rows", "square_len_m")
    return {key: data[key] for key in keys if key in data}


def reference_fov_deg(lens, width, height):
    """The datasheet field of view across the axis this capture still has.

    A camera offering several resolutions usually keeps one field of view and
    crops the other, and which one it keeps follows from the aspect ratios: a
    capture narrower than the sensor has lost its sides and kept its height, a
    capture wider has lost its top and bottom.

    Returns `(degrees, "horizontal"|"vertical", note)`.
    """
    _, spec = LENSES[lens]
    full_w, full_h = spec.get("full_size", (width, height))
    if width / height >= full_w / full_h:
        axis, note = "horizontal", ""
    else:
        axis = "vertical"
        note = (f"传感器是 {full_w}x{full_h}，{width}x{height} 是 4:3 — 只裁掉了左右，\n"
                f"    高度方向的视场留下来了，横向的从 "
                f"{_fov_across(spec, full_w, full_h, 'horizontal'):.1f}° "
                f"降到 {_fov_across(spec, width, height, 'horizontal'):.1f}°。\n"
                "    所以这里比的是纵向。")
    if f"{axis}_deg" in spec:
        return spec[f"{axis}_deg"], axis, note
    return (_fov_across(spec, width, height, axis), axis, note)


def _fov_across(spec, width, height, axis):
    """One axis of the datasheet field of view, for a given frame size.

    Preference order: the figure written down for that axis, then the other axis
    carried across the aspect ratio, then the diagonal split across it. The
    Orbbec needs the second of those -- its 84.3 is horizontal and the surviving
    axis at 640x480 is the vertical one -- and carrying 54.0 across 16:9 gives
    84.3 right back, which is the check that the two figures are one lens.
    """
    if f"{axis}_deg" in spec:
        return spec[f"{axis}_deg"]
    other = "vertical" if axis == "horizontal" else "horizontal"
    if f"{other}_deg" in spec:
        half = math.tan(math.radians(spec[f"{other}_deg"]) / 2.0)
        ratio = (width / height) if axis == "horizontal" else (height / width)
        return math.degrees(2.0 * math.atan(half * ratio))
    half = math.tan(math.radians(spec["diagonal_deg"]) / 2.0)
    full_w, full_h = spec.get("full_size", (width, height))
    span = width if axis == "horizontal" else height
    return math.degrees(2.0 * math.atan(half * span / math.hypot(full_w, full_h)))


# -- the checks ----------------------------------------------------------

def screen(intrinsics, report, args):
    dist = intrinsics.dist_coeffs
    width, height = intrinsics.image_size

    # 1. The numbers have to be numbers.
    values = list(intrinsics.camera_matrix.ravel()) + list(dist)
    if not all(np.isfinite(values)):
        report.add(FAIL, "内参里有 inf/nan", "后面的检查没有意义了")
        return
    report.add(PASS, "所有数字都是有限值")

    if intrinsics.fx <= 0.0 or intrinsics.fy <= 0.0:
        report.add(FAIL, f"焦距不是正数 fx={intrinsics.fx} fy={intrinsics.fy}")
        return
    report.add(PASS, f"焦距为正 fx={intrinsics.fx:.1f} fy={intrinsics.fy:.1f}")

    # 2. fx against fy. A real sensor has square pixels; the tolerance is for
    #    manufacturing, not for a sixteen per cent disagreement.
    anisotropy = abs(intrinsics.fx / intrinsics.fy - 1.0) * 100.0
    if anisotropy > 5.0:
        report.add(
            FAIL, f"fx 与 fy 相差 {anisotropy:.1f}%",
            "真实传感器是方形像素，正常差异在 1% 以内。\n"
            "差这么多说明求解器在用一个方向的伸缩去补偿别的东西 ——\n"
            "通常是棋盘格的格子尺寸或行列数写错了。")
    elif anisotropy > 2.0:
        report.add(WARN, f"fx 与 fy 相差 {anisotropy:.1f}%",
                   "偏大，但还不至于不能用")
    else:
        report.add(PASS, f"fx 与 fy 一致（相差 {anisotropy:.1f}%）")

    # 3. The principal point. Should be near the middle; a long way off usually
    #    means the images were cropped after calibration.
    off_x = (intrinsics.cx - width / 2.0) / width * 100.0
    off_y = (intrinsics.cy - height / 2.0) / height * 100.0
    if not (0 <= intrinsics.cx <= width and 0 <= intrinsics.cy <= height):
        report.add(FAIL, f"主点 ({intrinsics.cx:.1f}, {intrinsics.cy:.1f}) 在画面外")
    elif abs(off_x) > 10.0 or abs(off_y) > 10.0:
        report.add(WARN, f"主点偏离画面中心 {off_x:+.1f}% / {off_y:+.1f}%")
    else:
        report.add(PASS, f"主点接近画面中心（偏离 {off_x:+.1f}% / {off_y:+.1f}%）")

    # 4. Field of view. This is the check that needs outside knowledge, and it
    #    is the one that catches a focal length that is simply the wrong scale.
    hfov = intrinsics.horizontal_fov_deg
    vfov = intrinsics.vertical_fov_deg
    if hfov is None:
        report.add(WARN, "没记录分辨率，跳过视场角检查")
    elif not (20.0 <= hfov <= 175.0):
        report.add(
            FAIL, f"水平视场 {hfov:.1f}°，垂直 {vfov:.1f}° —— 镜头不可能这样",
            "20° 以下是长焦，用来拍按键的相机不会是长焦。\n"
            "焦距比真实值大 n 倍，解出来的距离就大 n 倍，而且位姿看上去自洽。")
    else:
        report.add(PASS, f"视场 {hfov:.1f}° x {vfov:.1f}°，在合理范围")

    # 5. The distortion terms, if there are any.
    #    All-zero is not a degenerate fit -- it is no fit. Worth separating,
    #    because every cancellation check below is undefined when the terms are
    #    zero and would otherwise divide 0 by 0 and report the worst possible
    #    score for the mildest possible model.
    if not np.any(dist):
        report.add(WARN, "畸变全为 0 —— 没标定，或者标定没收敛就放弃了",
                   "解出的位姿会把畸变当成不存在。对一只真实镜头，画面外圈的\n"
                   "角点位置会偏，而偏多少恰恰是不知道的。")
    else:
        # 5a. Cancellation. This is the check the reprojection error cannot
        #     make: huge terms that sum to nearly nothing are not a lens.
        terms = radial_terms(dist, frame_radius(intrinsics))
        total = sum(terms)
        magnitude = sum(abs(term) for term in terms)
        cancellation = magnitude / abs(total) if abs(total) > 1e-12 else float("inf")
        as_text = "  ".join(f"{term:+.5f}" for term in terms)
        if cancellation > 6.0:
            report.add(
                WARN, f"三个径向项互相抵消明显（{cancellation:.1f} 倍）",
                f"画面四角处  k1r² / k2r⁴ / k3r⁶ =  {as_text}\n"
                f"              合计 {total:+.5f}，而量级之和 {magnitude:.5f}\n"
                "⚠️ 单看这一项说明不了问题，所以这里只是提醒。广角镜头的归一化\n"
                "   半径本来就大（r 到 0.85，r⁴ = 0.5、r⁶ = 0.37），系数不大也能\n"
                "   抵消到这个程度：Orbbec 那只 80° 对角就到 23 倍，而模型在整个\n"
                "   画面里偏离无畸变不超过 1%。\n"
                "   真正的判据是下面那条 —— 出了画面它还像不像一只镜头。")
        elif cancellation > 3.0:
            report.add(WARN, f"径向项抵消明显（{cancellation:.1f} 倍）")
        else:
            report.add(PASS, f"径向项没有异常抵消（{cancellation:.1f} 倍）")

        # 5b. The model has to stay physical outside the frame too, or the
        #     distortion is fitting the frame rather than the lens.
        crossing = zero_crossing(intrinsics)
        if crossing is not None and crossing < 2.0:
            report.add(
                FAIL, f"径向因子在画面外 {crossing:.2f} 倍处就翻号了",
                "翻号之后畸变会把点折过光轴，真实镜头不会这样。\n"
                "这才是退化拟合的确切特征：模型在拟合用的那几个像素之外\n"
                "立刻失去物理意义。上面那条抵消倍数只是它的倒影。")
        elif crossing is not None:
            report.add(PASS, f"径向因子在画面外 {crossing:.2f} 倍处才翻号")
        else:
            report.add(PASS, "径向因子在整个探测量程内保持为正")

        # 5c. Tangential distortion is a small correction by nature.
        if len(dist) >= 4:
            worst = max(abs(dist[2]), abs(dist[3]))
            if worst > 0.2:
                report.add(FAIL, f"切向畸变过大 p1={dist[2]:+.4f} p2={dist[3]:+.4f}")
            elif worst > 0.05:
                report.add(WARN, f"切向畸变偏大 p1={dist[2]:+.4f} p2={dist[3]:+.4f}")
            else:
                report.add(PASS, f"切向畸变在正常量级（p1={dist[2]:+.4f} "
                                 f"p2={dist[3]:+.4f}）")

    # 6. The intrinsics have to be for the resolution we actually capture at.
    if args.capture_size:
        want = parse_size(args.capture_size)
        if want != (width, height):
            report.add(
                WARN, f"内参是 {width}x{height} 的，但采集按 "
                      f"{want[0]}x{want[1]}",
                "marker_locator 会自动按比例换算 fx/fy/主点，畸变系数不变。\n"
                "前提是两种分辨率覆盖同一视场 —— UVC 通常如此，但不保证。")
        else:
            report.add(PASS, f"内参分辨率与采集分辨率一致（{width}x{height}）")

    # 7. The fit itself.
    extras = raw_extras(args.path)
    count = extras.get("n_images", extras.get("image_count"))
    if count is not None and count < MIN_IMAGES:
        report.add(
            WARN, f"只用了 {count} 张图标定",
            f"少于 {MIN_IMAGES} 张时，径向畸变基本不可观测，求解器会把它\n"
            "让给焦距去补偿 —— 这正是上面那些症状的来源。")
    # 8. And the one number everyone does check.
    if "reproj_err_px" in extras:
        report.add(
            PASS, f"重投影误差 {extras['reproj_err_px']:.3f} px —— 这个数好看，"
                  "但它不能说明内参是对的")


def parse_size(text):
    width, _, height = text.lower().partition("x")
    return int(width), int(height)


# -- main ----------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="体检一份相机内参")
    parser.add_argument("path", help="YAML/JSON，ROS CameraInfo 或 K/dist 都行")
    parser.add_argument("--lens", choices=sorted(LENSES),
                        help="按规格书核对视场角")
    parser.add_argument("--expect-fov-deg", type=float,
                        help="已知的水平视场角，覆盖 --lens")
    parser.add_argument("--capture-size", metavar="WxH",
                        help="实际采集分辨率，检查是否需要换算")
    args = parser.parse_args()

    # Same path rules as everything else here, so `config/x.yaml` means the
    # same file whichever directory and whichever way of launching.
    args.path = resolve_config_path(args.path)
    if not os.path.isfile(args.path):
        print(f"没有这个文件: {args.path}")
        return 2

    intrinsics = CameraIntrinsics.from_yaml(args.path)
    if intrinsics.image_size is None:
        print(f"{args.path}: 没有记录分辨率，几个检查会被跳过")

    print()
    print("═" * 68)
    print(f" 内参体检   {args.path}")
    print("═" * 68)
    print()

    width, height = intrinsics.image_size or (None, None)
    print(" ① 模型")
    print(f"     fx {intrinsics.fx:10.2f}    fy {intrinsics.fy:10.2f}")
    print(f"     cx {intrinsics.cx:10.2f}    cy {intrinsics.cy:10.2f}")
    if width:
        print(f"     分辨率 {width}x{height}")
    if len(intrinsics.dist_coeffs):
        names = ["k1", "k2", "p1", "p2", "k3", "k4", "k5", "k6"]
        body = "  ".join(f"{names[i]}={value:+.4f}"
                         for i, value in enumerate(intrinsics.dist_coeffs[:8]))
        print(f"     畸变   {body}")
    else:
        print("     畸变   全 0")
    extras = raw_extras(args.path)
    if extras:
        fit = "  ".join(f"{key}={value}" for key, value in extras.items())
        print(f"     拟合   {fit}")
    if intrinsics.horizontal_fov_deg is not None:
        print(f"     视场   {intrinsics.horizontal_fov_deg:.1f}° x "
              f"{intrinsics.vertical_fov_deg:.1f}°")
    print()

    report = Report()
    screen(intrinsics, report, args)

    print(" ② 逐项检查")
    report.print()

    # The datasheet comparison, when we have something to compare against.
    inflation = None
    if args.expect_fov_deg or args.lens:
        note = ""
        if args.expect_fov_deg:
            reference, axis, source = args.expect_fov_deg, "horizontal", "给定值"
        else:
            name, _ = LENSES[args.lens]
            reference, axis, note = reference_fov_deg(
                args.lens, width or 640, height or 480)
            source = f"{name} 规格书"
        focal = intrinsics.fx if axis == "horizontal" else intrinsics.fy
        span = width if axis == "horizontal" else height
        if focal is not None and span:
            expected_focal = (span / 2.0) / math.tan(
                math.radians(reference) / 2.0)
            inflation = focal / expected_focal
            label = "水平" if axis == "horizontal" else "纵向"
            print(f" ③ 与{source}对照（{label}视场 {reference:.1f}°）")
            if note:
                for line in note.splitlines():
                    print(f"     {line}")
            if abs(inflation - 1.0) < 0.10:
                print(f"     [PASS] 焦距与规格书一致（{inflation:.2f} 倍）")
            else:
                print(f"     [FAIL] 焦距是规格书的 {inflation:.2f} 倍")
                print(f"            规格书推出的 f ≈ {expected_focal:.0f}，"
                      f"这份是 {focal:.0f}")
            print()

    print("═" * 68)
    if report.failures:
        print(f" 结论：不可用 —— {len(report.failures)} 项不合格"
              f"，{len(report.warnings)} 项存疑")
    elif report.warnings:
        print(f" 结论：勉强可用 —— {len(report.warnings)} 项存疑，"
              "用之前先看一眼上面")
    else:
        print(" 结论：通过")
    print("═" * 68)

    if inflation is not None and abs(inflation - 1.0) >= 0.10:
        print()
        print(f" 以这份内参解算，距离会被放大 {inflation:.2f} 倍：")
        for truth in (0.30, 0.50, 0.80):
            print(f"     板子真在 {truth:.2f} m  →  读出 {truth * inflation:.2f} m")
        print()
        print(" 这是可以当场验的：把板子放在量好的 0.50 m 处跑一次")
        print(" `marker_locator`，读出 1.6 m 上下就说明焦距确实错了这么多。")

    if report.failures:
        print()
        print(" 怎么修 —— 重标定")
        print(" ────────────────")
        print(" 上面几项不合格通常不是各自独立的问题，而是同一个原因的几种说法：")
        print(" 棋盘姿态不够多样，径向畸变没有被真正观测到，于是求解器让焦距和")
        print(" 畸变互相补偿。这样得到的解在拟合用的那些像素上是自洽的，整体却")
        print(" 错了一个尺度。重标定时：")
        print()
        print("   · 图像 ≥ 15 张，姿态要散开")
        print("   · 棋盘要倾斜，两个方向各 30° 以上")
        print("   · 棋盘要出现在画面四角，不能总在中间")
        print("   · 棋盘要占满画面，不要一直离得远")
        print("   · 标完拿这个脚本再体检一次")

    print()
    return 1 if report.failures else 0


if __name__ == "__main__":
    sys.exit(main())
