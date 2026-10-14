#!/usr/bin/env python3
"""Find the board and publish where it is.

The last piece between the camera and the rest of the system: opens the camera,
detects the board, optionally transforms the result into the robot's frame, and
publishes it. See `aruco_board.py` for the geometry and `press_targets.py` for
what consumes the output.

Two modes, and the difference is whether the robot is involved
--------------------------------------------------------------
With `target_frame` empty (the default) the board pose is published in the
**camera's own optical frame**. That needs no robot, no TF and no hand-eye
calibration, so it works the moment the camera does, and it is the right mode
for checking that detection is working at all: point the camera at the board and
watch the numbers.

With `target_frame:=base_link` the pose is transformed through TF, which means
joint states, `robot_state_publisher`, and a hand-eye calibration written into
the URDF. That is the mode the press task needs, and it cannot work without
those three things.

The stationary gate belongs to the second mode only. This camera is on the
wrist, so `T_base <- camera` changes as the arm moves, and this path has no
hardware capture timestamp -- the frame is stamped when userspace finished
assembling it. Motion therefore turns directly into pose error. In camera-frame
mode none of that applies: the board's pose relative to the camera does not care
what the arm is doing.

Why it opens the camera itself
------------------------------
A separate image publisher would be tidier and would let a bag replace the
camera for offline tuning, but only one process can hold a UVC device, so a
publisher plus this node is the same contention as this node alone. Rather than
add the split now, the annotated debug image doubles as the live view -- it is
what you would be looking at anyway, with the detections already drawn on. The
split is a small refactor when recording becomes worth more than the simpler
topology.

⚠️ The camera is exclusive: stop `tools/cam_view.py` before starting this.
"""

import argparse
import glob
import os
import sys
import time

import cv2
import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from rclpy.time import Time
from sensor_msgs.msg import CompressedImage, JointState
from tf2_ros import Buffer, TransformListener

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from aruco_board import (CameraIntrinsics, annotate,       # noqa: E402
                         board_specs_from_config, detect,
                         resolve_config_path)
from press_targets import quaternion_from_matrix, transform_to_matrix  # noqa: E402

#: A name rather than a path: the same file sits at a different depth in the
#: source tree than it does under `ros2 run`, and `resolve_config_path` knows
#: both. Hardcoding a path here used to work in development and fail on install.
DEFAULT_CONFIG = "config/board.yaml"

DEFAULT_CAMERA_FRAME = "wrist_depth_camera_optical_frame"

#: Used when `refine_board.pose_topic` is absent. The two boards publish to
#: separate topics on purpose -- see `BoardChannel`.
DEFAULT_REFINE_TOPIC = "/perception/refine_board_pose"

CONTRACT_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    durability=DurabilityPolicy.VOLATILE,
)

#: Cheap JPEG encoding of the annotated view; this is for a human, not for
#: further processing.
DEBUG_JPEG_QUALITY = 80


def _video_devices_by_name():
    """{node path: lowercased name} from sysfs, which survives containers."""
    found = {}
    for entry in sorted(glob.glob("/sys/class/video4linux/video*")):
        try:
            with open(os.path.join(entry, "name")) as handle:
                found["/dev/" + os.path.basename(entry)] = handle.read().strip().lower()
        except OSError:
            continue
    return found


def resolve_device(requested, hints=("orbbec", "dabai")):
    """A usable video device path, or None.

    Three strategies, in order, because no single one works everywhere:

      1. `requested`, if it exists. The caller pinning the device always wins.
      2. The udev by-id tree, matched against `hints`. Stable across replugs,
         but udev does not build this tree inside a container -- /dev there is
         a tmpfs that only receives the device nodes themselves.
      3. `/sys/class/video4linux/videoN/name` matched against the same hints.
         sysfs is the host's, mounted read-only, so this survives the container
         where by-id does not.

    `hints` is a parameter rather than a constant because a laptop camera and a
    robot camera do not share a name. Hardcoding "orbbec" here is what made the
    code fail to find a perfectly working Logitech webcam.

    Mirrors `camera_capture.resolve_device` in the capture package, which cannot
    be imported without this package depending on that one for twenty lines.
    """
    if requested and os.path.exists(requested):
        return requested
    lowered = tuple(h.lower() for h in hints if h)
    if not lowered:
        return None

    candidates = sorted(p for p in glob.glob("/dev/v4l/by-id/*")
                        if os.path.exists(p)
                        and any(h in os.path.basename(p).lower()
                                for h in lowered))
    if candidates:
        return candidates[0]

    named = _video_devices_by_name()
    for node, name in sorted(named.items()):
        if any(h in name for h in lowered) and os.path.exists(node):
            return node
    return None


class JointStability:
    """Tracks /joint_states and answers whether the arm has stopped.

    Only used in base-frame mode; see the module docstring.
    """

    def __init__(self, tolerance_rad, samples):
        self.tolerance_rad = tolerance_rad
        self.samples = samples
        self._previous = None
        self._stable = 0
        self._received = False

    def update(self, message):
        by_name = dict(zip(message.name, message.position))
        if not by_name:
            return
        self._received = True
        if self._previous is not None:
            shared = set(by_name) & set(self._previous)
            if shared:
                worst = max(abs(by_name[n] - self._previous[n])
                            for n in shared)
                self._stable = self._stable + 1 if worst <= self.tolerance_rad else 0
        self._previous = by_name

    @property
    def received(self):
        return self._received

    @property
    def is_stable(self):
        return (not self._received) or self._stable >= self.samples


class BoardChannel:
    """One board: its spec, its gate, its publisher, its own reasons to fail.

    A deployment has two of these -- a wide board seen from far away, and a
    single marker seen from the approach pose -- and they differ in geometry,
    marker count, gate and output topic. They are separate objects rather than
    two passes over shared state because the failure they must not have is a
    *routing* failure: a coarse board pose published on the refined topic fuses
    into a wrong correction, and every log line, rms and marker count still
    looks perfectly healthy. Here there is nowhere for the two to mix; each
    channel is the only writer of its own publisher.

    The miss/accept bookkeeping is per channel for the same reason. Shared, the
    two boards failing in turn would each reset the other's "reason unchanged"
    counter, and two warning lines per attempt would be printed forever.
    """

    def __init__(self, name, spec, min_markers, topic, publisher, logger):
        self.name = name
        self.spec = spec
        self.min_markers = min_markers
        self.topic = topic
        self.publisher = publisher
        self.logger = logger
        self.detector = spec.make_detector()
        self.published = 0
        self.misses = 0
        self.last_reason = ""

    def look(self, frame, intrinsics):
        return detect(frame, intrinsics.camera_matrix,
                      intrinsics.dist_coeffs, self.spec,
                      detector=self.detector,
                      min_markers=self.min_markers)

    def gate(self, result, max_reproj, min_distance, max_distance):
        """None when the detection may be published, else (category, detail).

        `min_markers` is already applied inside `detect()`. Reprojection error
        is deliberately the weaker of the two gates: a board seen from behind
        collapses to one marker, and that survivor still solves to a
        confident pose at a fraction of a pixel. A small rms therefore means
        "the arithmetic is self-consistent", not "this is the board".

        The reason is split in two because the caller de-duplicates on it: a
        measured number folded into the category would make every attempt a
        new reason, and the log would fill at the processing rate instead of
        saying why nothing is published.
        """
        if result is None:
            return (f"no board (need {self.min_markers} markers)", "")
        if result.reproj_rms_px > max_reproj:
            return ("reprojection error is above the gate",
                    f"{result.reproj_rms_px:.2f} px, limit {max_reproj} px")
        if not min_distance <= result.distance <= max_distance:
            return ("the board is outside the accepted distance range",
                    f"{result.distance * 100:.1f} cm, range "
                    f"{min_distance * 100:.0f}-{max_distance * 100:.0f} cm")
        return None

    def publish(self, matrix, frame_id, clock):
        qx, qy, qz, qw = quaternion_from_matrix(matrix[:3, :3])
        message = PoseStamped()
        message.header.stamp = clock.now().to_msg()
        message.header.frame_id = frame_id
        message.pose.position.x = float(matrix[0, 3])
        message.pose.position.y = float(matrix[1, 3])
        message.pose.position.z = float(matrix[2, 3])
        message.pose.orientation.x = qx
        message.pose.orientation.y = qy
        message.pose.orientation.z = qz
        message.pose.orientation.w = qw
        self.publisher.publish(message)
        self.published += 1
        if self.published == 1:
            t = matrix[:3, 3]
            self.logger.info(
                f"{self.name}: board at ({t[0]:+.4f}, {t[1]:+.4f}, "
                f"{t[2]:+.4f}) m in '{frame_id}'")

    def reject(self, reason, detail=""):
        """Record why nothing was published, once per reason change.

        Either the detector or the gates fire several times a second, and a
        reason repeated at that rate buries everything else in the log. Which
        only holds if `reason` is a category -- see `gate`.
        """
        if reason == self.last_reason:
            self.misses += 1
            return
        if self.last_reason:
            self.logger.warn(
                f"{self.name}: ...previous reason held for "
                f"{self.misses} attempts")
        self.last_reason = reason
        self.misses = 1
        message = f"{self.name}: not publishing: {reason}"
        if detail:
            message += f" -- {detail}"
        self.logger.warn(message)

    def accept(self):
        if self.last_reason:
            self.logger.info(f"{self.name}: detecting again after "
                             f"{self.misses} refusals")
            self.last_reason = ""
            self.misses = 0


class MarkerLocator(Node):

    def __init__(self, parameter_overrides=None):
        super().__init__("marker_locator",
                         parameter_overrides=parameter_overrides or [])

        self.declare_parameter("config", DEFAULT_CONFIG)
        self.declare_parameter("camera_info", "")
        self.declare_parameter("device", "/dev/video2")
        self.declare_parameter("device_hint", "orbbec,dabai")
        # 640x480 because that is the only size this camera has a calibration
        # for. Defaulting to the sensor's native size would mean every run
        # without a params file rescales the intrinsics and warns about it.
        self.declare_parameter("width", 640)
        self.declare_parameter("height", 480)
        self.declare_parameter("fps", 30)
        self.declare_parameter("warmup_frames", 30)
        self.declare_parameter("process_rate_hz", 5.0)
        self.declare_parameter("debug_rate_hz", 2.0)
        self.declare_parameter("publish_debug_image", True)

        self.declare_parameter("camera_frame", DEFAULT_CAMERA_FRAME)
        self.declare_parameter("target_frame", "")
        self.declare_parameter("require_stationary", True)
        self.declare_parameter("stationary_tolerance_rad", 0.002)
        self.declare_parameter("stationary_samples", 5)

        self.declare_parameter("min_markers", 0)
        self.declare_parameter("max_reproj_px", 3.0)
        self.declare_parameter("min_distance_m", 0.15)
        self.declare_parameter("max_distance_m", 1.5)

        self.declare_parameter("board_topic", "/perception/board_pose")
        self.declare_parameter("debug_topic",
                               "/perception/debug_image/compressed")

        self.camera_frame = self.get_parameter("camera_frame").value
        self.target_frame = self.get_parameter("target_frame").value
        self.max_reproj = float(self.get_parameter("max_reproj_px").value)
        self.min_distance = float(self.get_parameter("min_distance_m").value)
        self.max_distance = float(self.get_parameter("max_distance_m").value)
        self.require_stationary = bool(
            self.get_parameter("require_stationary").value)
        # Named `want_debug` rather than `publish_debug`: a method of that name
        # exists a few hundred lines down, and an attribute of the same name
        # shadows it without a word of complaint. The failure then shows up as
        # "'bool' object is not callable" the first time a board is actually
        # seen -- which is to say, never during development, because there is
        # usually nothing in front of the camera.
        self.want_debug = bool(
            self.get_parameter("publish_debug_image").value)

        self.intrinsics = self._load_intrinsics()
        self.channels = self._load_channels()

        self.stability = JointStability(
            float(self.get_parameter("stationary_tolerance_rad").value),
            int(self.get_parameter("stationary_samples").value))
        self.create_subscription(JointState, "/joint_states",
                                 self.stability.update, 10)

        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)

        self.debug_publisher = self.create_publisher(
            CompressedImage, self.get_parameter("debug_topic").value,
            CONTRACT_QOS)

        self.camera = self._open_camera()
        self.warmup_left = int(self.get_parameter("warmup_frames").value)

        self.process_period = 1.0 / max(0.1, float(
            self.get_parameter("process_rate_hz").value))
        self.debug_period = 1.0 / max(0.1, float(
            self.get_parameter("debug_rate_hz").value))
        self._last_debug = 0.0
        self._last_reason = ""
        self._misses = 0
        self.create_timer(self.process_period, self.tick)

        self._describe_startup()

    # -- setup -------------------------------------------------------------

    def _load_channels(self):
        """One channel per board declared in `config/board.yaml`.

        Two boards is the deployment this was written for -- a wide board far
        away for the coarse target, a single marker at the approach pose for the
        refined one -- but one board stays a first-class configuration: the file
        simply omits `refine_board`.

        An explicit `min_markers` parameter applies to the coarse channel only.
        One number cannot serve both: the same value that is right for the wide
        board is an impossible gate for a lone marker, and the failure it
        produces (`no board (need N markers)`, forever) reads exactly like a
        camera that cannot see anything -- the one diagnosis you would have
        ruled out first, and the wrong one.

        The refined board's topic comes from the same section as its geometry,
        so the node and the bridge that consumes it read it from one place
        instead of two that can drift apart.
        """
        path = resolve_config_path(self.get_parameter("config").value)
        with open(path, encoding="utf-8") as handle:
            data = yaml.safe_load(handle) or {}
        coarse_spec, refine_spec = board_specs_from_config(data)

        plan = [("coarse", coarse_spec,
                 self.get_parameter("board_topic").value,
                 int(self.get_parameter("min_markers").value))]
        if refine_spec is not None:
            refine_node = data.get("refine_board") or {}
            plan.append(("refine", refine_spec,
                         refine_node.get("pose_topic", DEFAULT_REFINE_TOPIC),
                         0))

        channels = []
        for name, spec, topic, override in plan:
            channels.append(BoardChannel(
                name, spec, self._min_markers_for(name, spec, override), topic,
                self.create_publisher(PoseStamped, topic, CONTRACT_QOS),
                self.get_logger()))
        return channels

    def _min_markers_for(self, name, spec, override):
        """How many markers must decode before this board's pose is published.

        A positive override wins; zero (the default) follows the shape already
        described in `config/board.yaml`. Deriving it rather than asking for it
        keeps one source of truth: a leftover constant from the other shape
        fails in a disguise.
        """
        if override > 0:
            return override
        derived = spec.auto_min_markers
        total = spec.markers_x * spec.markers_y
        self.get_logger().info(
            f"{name}: min_markers not set, following board.yaml: "
            f"{spec.markers_x}x{spec.markers_y} board, "
            f"need >= {derived} of {total} markers")
        return derived

    def _load_intrinsics(self):
        path = self.get_parameter("camera_info").value
        if path:
            resolved = resolve_config_path(path)
            intrinsics = CameraIntrinsics.from_file_or_directory(resolved)
            self.get_logger().info(f"intrinsics from {resolved}: {intrinsics}")
            return intrinsics
        # The fallback is a real calibration rather than a provisional one, so
        # it is worth saying which resolution it belongs to and stopping there.
        # A matrix that is right at one size and wrong at another is more
        # dangerous than a placeholder nobody trusts.
        fallback = resolve_config_path(
            "config/camera_info_orbbec_640x480.yaml")
        intrinsics = CameraIntrinsics.from_yaml(fallback)
        self.get_logger().warn(
            f"no camera_info given, using {os.path.basename(fallback)}, "
            f"measured with this camera at 640x480. At any other resolution "
            f"it is a plausible-looking guess; pass camera_info explicitly.")
        return intrinsics

    def _open_camera(self):
        hints = tuple(part.strip() for part in
                      self.get_parameter("device_hint").value.split(","))
        device = resolve_device(self.get_parameter("device").value, hints)
        if device is None:
            requested = self.get_parameter("device").value
            raise RuntimeError(
                f"no camera found for device={requested!r} or hint {hints}.\n"
                "    `ls /sys/class/video4linux/` shows what the kernel sees.\n"
                "    A missing /dev/video* node means the container has no "
                "device passthrough;\n"
                "    a name that does not match means the hint needs changing, "
                "e.g. -p device_hint:=logitech")
        width = int(self.get_parameter("width").value)
        height = int(self.get_parameter("height").value)
        fps = int(self.get_parameter("fps").value)

        capture = cv2.VideoCapture(device, cv2.CAP_V4L2)
        if not capture.isOpened():
            raise RuntimeError(f"cannot open {device}")
        # MJPG explicitly: on USB 2.0 the uncompressed formats are bandwidth
        # capped (1080p YUYV manages about 5 fps), and 1080p is where the
        # marker corners are worth having.
        capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        capture.set(cv2.CAP_PROP_FPS, fps)
        capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self.device = device

        # What was asked for and what was granted differ more often than you
        # would like. A webcam that cannot do the requested size does not
        # refuse; it quietly hands back the nearest mode it has. Since the
        # intrinsics belong to a specific frame size, a surprise here shows up
        # as a wrong distance rather than as a failed start, so read it back.
        self.frame_size = (int(capture.get(cv2.CAP_PROP_FRAME_WIDTH)),
                           int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT)))
        if self.frame_size != (width, height):
            self.get_logger().warn(
                f"asked the camera for {width}x{height}, it gave "
                f"{self.frame_size[0]}x{self.frame_size[1]}")
        self.intrinsics = self._reconciled_intrinsics(self.frame_size)
        return capture

    def _reconciled_intrinsics(self, frame_size):
        """Make the intrinsics agree with the frame size actually arriving.

        A calibration is only valid at the resolution it was measured at. A
        640x480 calibration driving a 1280x960 frame is not a small error: fx,
        fy and both principal point coordinates are all a factor of two out, so
        every distance is too. It is also invisible from the result, which
        still looks entirely self-consistent -- the board is simply reported
        twice as far away as it really is.
        """
        scaled = self.intrinsics.scaled_to(*frame_size)
        if scaled is None:
            if self.intrinsics.image_size is None:
                self.get_logger().warn(
                    "the calibration records no image size, so there is no "
                    f"way to check it against the {frame_size[0]}x"
                    f"{frame_size[1]} frame; assuming they match")
            return self.intrinsics
        self.get_logger().warn(
            f"calibration is for {self.intrinsics.image_size[0]}x"
            f"{self.intrinsics.image_size[1]} but the camera runs at "
            f"{frame_size[0]}x{frame_size[1]}; rescaling fx, fy and the "
            "principal point.\n"
            "    That assumes both sizes cover the same field of view, which is "
            "not always\n"
            "    so: one camera measured here covered 43.5 deg at 640x480 "
            "where 1280x960\n"
            "    covered 49.2, and the rescale then read distances about 15 "
            "per cent long.\n"
            "    Prefer running at the resolution the calibration was measured "
            "at.")
        return scaled

    def _describe_startup(self):
        mode = (f"publishing in '{self.target_frame}'"
                if self.target_frame else
                f"publishing in '{self.camera_frame}' (no robot needed)")
        self.get_logger().info(f"camera {self.device} @ "
                               f"{self.frame_size[0]}x{self.frame_size[1]}, "
                               f"{mode}")
        for channel in self.channels:
            self.get_logger().info(
                f"{channel.name}: {channel.spec}  ->  {channel.topic}, "
                f"markers >= {channel.min_markers}")
        # Printing the field of view rather than only the focal length, because
        # the field of view is the number that can be checked against physics.
        # A 16 degree lens on a webcam bracket is visibly impossible; an
        # fx of 2299 is not.
        if self.intrinsics.horizontal_fov_deg is not None:
            self.get_logger().info(
                f"lens {self.intrinsics.horizontal_fov_deg:.1f} x "
                f"{self.intrinsics.vertical_fov_deg:.1f} deg, "
                f"fx={self.intrinsics.fx:.1f} fy={self.intrinsics.fy:.1f}")
        self.get_logger().info(
            f"gates: rms <= {self.max_reproj} px, "
            f"{self.min_distance}-{self.max_distance} m"
            + (", arm stationary" if (self.require_stationary
                                      and self.target_frame) else ""))

    # -- the loop ----------------------------------------------------------

    def tick(self):
        ok, frame = self.camera.read()
        if not ok or frame is None:
            self.reject("camera returned no frame")
            return
        # The first frames are the AE/AWB loop settling. Their exposure and
        # white balance are still moving, and detection on them is a coin flip.
        if self.warmup_left > 0:
            self.warmup_left -= 1
            if self.warmup_left == 0:
                self.get_logger().info("warm-up done, detecting")
            return

        seen = []
        for channel in self.channels:
            result = channel.look(frame, self.intrinsics)
            verdict = channel.gate(result, self.max_reproj,
                                   self.min_distance, self.max_distance)
            if verdict is not None:
                channel.reject(*verdict)
                continue
            matrix, frame_id = self.to_target_frame(result)
            if matrix is None:
                continue        # to_target_frame has already said why
            channel.publish(matrix, frame_id, self.get_clock())
            channel.accept()
            seen.append((channel, result))

        if seen:
            self.accept()
        self.publish_debug(frame, seen)

    def to_target_frame(self, result):
        """(4x4 in the output frame, its name), or (None, '') with a reason."""
        if not self.target_frame:
            return result.matrix, self.camera_frame

        if self.require_stationary and self.stability.received \
                and not self.stability.is_stable:
            self.reject("arm is moving; T_base<-camera would be stale")
            return None, ""

        try:
            transform = self.buffer.lookup_transform(
                self.target_frame, self.camera_frame, Time(),
                timeout=rclpy.duration.Duration(seconds=0.2))
        except Exception as error:                          # noqa: BLE001
            self.reject(f"TF {self.target_frame} <- {self.camera_frame}: "
                        f"{str(error).splitlines()[0]}")
            return None, ""

        return transform_to_matrix(transform.transform) @ result.matrix, \
            self.target_frame

    # -- output ------------------------------------------------------------

    def publish_debug(self, frame, seen):
        if not self.want_debug or not seen:
            return
        now = time.monotonic()
        if now - self._last_debug < self.debug_period:
            return
        self._last_debug = now
        # Drawn one over the other rather than one instead of the other: seeing
        # both boards in the same picture is the point of this view. It is the
        # only place a mis-solved refined board becomes visible at all, because
        # a mirrored single marker solves to a confident pose at a fraction of
        # a pixel.
        annotated = frame
        for row, (channel, result) in enumerate(seen):
            annotated = annotate(annotated, result,
                                 self.intrinsics.camera_matrix,
                                 self.intrinsics.dist_coeffs,
                                 label=f"{channel.name} ",
                                 text_origin=(10, 28 + 26 * row))
        ok, buffer = cv2.imencode(".jpg", annotated,
                                  [int(cv2.IMWRITE_JPEG_QUALITY),
                                   DEBUG_JPEG_QUALITY])
        if not ok:
            return
        message = CompressedImage()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.camera_frame
        message.format = "jpeg"
        message.data = buffer.tobytes()
        self.debug_publisher.publish(message)

    # -- logging that does not flood ---------------------------------------

    def reject(self, reason):
        """Record why nothing is published, once per reason change.

        Either the detector or the gates fire several times a second, and a
        reason repeated at that rate buries everything else in the log.
        """
        self._misses += 1
        if reason == self._last_reason:
            return
        if self._last_reason:
            self.get_logger().warn(f"...previous reason held for "
                                   f"{self._misses} attempts")
        self._last_reason = reason
        self._misses = 0
        self.get_logger().warn(f"not publishing: {reason}")

    def accept(self):
        if self._last_reason:
            self.get_logger().info(f"detecting again after "
                                   f"{self._misses} refusals")
            self._last_reason = ""
            self._misses = 0


def main(argv=None):
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--device", default=None)
    parser.add_argument("--config", default=None)
    parser.add_argument("--camera-info", default=None)
    parser.add_argument("--target-frame", default=None,
                        help="empty string stays in the camera frame")
    parser.add_argument("--no-debug-image", action="store_true")
    arguments, _ = parser.parse_known_args(argv)

    overrides = []
    if arguments.device:
        overrides.append(Parameter("device", value=arguments.device))
    if arguments.config:
        overrides.append(Parameter("config", value=arguments.config))
    if arguments.camera_info:
        overrides.append(Parameter("camera_info", value=arguments.camera_info))
    if arguments.target_frame is not None:
        overrides.append(Parameter("target_frame",
                                   value=arguments.target_frame))
    if arguments.no_debug_image:
        overrides.append(Parameter("publish_debug_image", value=False))

    rclpy.init()
    try:
        node = MarkerLocator(parameter_overrides=overrides)
    except (RuntimeError, ValueError) as error:
        print(f"error: {error}", file=sys.stderr)
        if rclpy.ok():
            rclpy.shutdown()
        return 1

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        # `timeout`, or a systemd stop, tears the context down underneath
        # spin(), which then raises out of the executor. That is an ordinary way
        # for this process to end and it does not deserve a page of traceback --
        # noise like that is how a real failure goes unnoticed.
        pass
    finally:
        # rclpy installs signal handlers that shut the context down before the
        # finally block runs, so anything touching it afterwards raises. On a
        # SIGTERM that turned a clean exit into a page of traceback, which is
        # exactly the noise that hides a real failure.
        try:
            node.camera.release()
            node.destroy_node()
        except Exception:                                   # noqa: BLE001
            pass
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
