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
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from rclpy.time import Time
from sensor_msgs.msg import CompressedImage, JointState
from tf2_ros import Buffer, TransformListener

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from aruco_board import (BoardSpec, CameraIntrinsics,      # noqa: E402
                         annotate, detect)
from press_targets import quaternion_from_matrix, transform_to_matrix  # noqa: E402

DEFAULT_CONFIG = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "config", "board.yaml")

DEFAULT_CAMERA_FRAME = "wrist_depth_camera_optical_frame"

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


def resolve_config_path(path):
    """Absolute path to a config file, accepting a package-relative name.

    A params file cannot know where the workspace is mounted, so `config/x.yaml`
    has to mean the same thing from any working directory. Absolute paths and
    paths relative to the current directory are honoured first; only then is the
    installed share directory tried.
    """
    if not path:
        return path
    if os.path.exists(path):
        return path
    try:
        from ament_index_python.packages import get_package_share_directory
        candidate = os.path.join(
            get_package_share_directory("piper_marker_perception"), path)
        if os.path.exists(candidate):
            return candidate
    except Exception:                                       # noqa: BLE001
        pass
    return path                 # let the caller report its own failure


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


class MarkerLocator(Node):

    def __init__(self, parameter_overrides=None):
        super().__init__("marker_locator",
                         parameter_overrides=parameter_overrides or [])

        self.declare_parameter("config", DEFAULT_CONFIG)
        self.declare_parameter("camera_info", "")
        self.declare_parameter("device", "/dev/video2")
        self.declare_parameter("device_hint", "orbbec,dabai")
        self.declare_parameter("width", 1920)
        self.declare_parameter("height", 1080)
        self.declare_parameter("fps", 15)
        self.declare_parameter("warmup_frames", 30)
        self.declare_parameter("process_rate_hz", 5.0)
        self.declare_parameter("debug_rate_hz", 2.0)
        self.declare_parameter("publish_debug_image", True)

        self.declare_parameter("camera_frame", DEFAULT_CAMERA_FRAME)
        self.declare_parameter("target_frame", "")
        self.declare_parameter("require_stationary", True)
        self.declare_parameter("stationary_tolerance_rad", 0.002)
        self.declare_parameter("stationary_samples", 5)

        self.declare_parameter("min_markers", 6)
        self.declare_parameter("max_reproj_px", 3.0)
        self.declare_parameter("min_distance_m", 0.15)
        self.declare_parameter("max_distance_m", 1.5)

        self.declare_parameter("board_topic", "/perception/board_pose")
        self.declare_parameter("debug_topic",
                               "/perception/debug_image/compressed")

        self.spec = BoardSpec.from_yaml(
            resolve_config_path(self.get_parameter("config").value))
        self.camera_frame = self.get_parameter("camera_frame").value
        self.target_frame = self.get_parameter("target_frame").value
        self.min_markers = int(self.get_parameter("min_markers").value)
        self.max_reproj = float(self.get_parameter("max_reproj_px").value)
        self.min_distance = float(self.get_parameter("min_distance_m").value)
        self.max_distance = float(self.get_parameter("max_distance_m").value)
        self.require_stationary = bool(
            self.get_parameter("require_stationary").value)
        self.publish_debug = bool(
            self.get_parameter("publish_debug_image").value)

        self.intrinsics = self._load_intrinsics()
        self.detector = self.spec.make_detector()

        self.stability = JointStability(
            float(self.get_parameter("stationary_tolerance_rad").value),
            int(self.get_parameter("stationary_samples").value))
        self.create_subscription(JointState, "/joint_states",
                                 self.stability.update, 10)

        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)

        self.pose_publisher = self.create_publisher(
            PoseStamped, self.get_parameter("board_topic").value, CONTRACT_QOS)
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
        self._published = 0
        self.create_timer(self.process_period, self.tick)

        self._describe_startup()

    # -- setup -------------------------------------------------------------

    def _load_intrinsics(self):
        path = self.get_parameter("camera_info").value
        if path:
            resolved = resolve_config_path(path)
            intrinsics = CameraIntrinsics.from_file_or_directory(resolved)
            self.get_logger().info(f"intrinsics from {resolved}: {intrinsics}")
            return intrinsics
        fallback = os.path.join(os.path.dirname(DEFAULT_CONFIG),
                                "camera_info_nominal.yaml")
        intrinsics = CameraIntrinsics.from_yaml(fallback)
        self.get_logger().warn(
            f"no camera_info given, using the provisional matrix in "
            f"{os.path.basename(fallback)}. Distances are good to a few "
            f"centimetres at best until the real calibration arrives.")
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
        return capture

    def _describe_startup(self):
        mode = (f"publishing in '{self.target_frame}'"
                if self.target_frame else
                f"publishing in '{self.camera_frame}' (no robot needed)")
        self.get_logger().info(f"{self.spec}")
        self.get_logger().info(f"camera {self.device} @ "
                               f"{int(self.get_parameter('width').value)}x"
                               f"{int(self.get_parameter('height').value)}, "
                               f"{mode}")
        self.get_logger().info(
            f"gates: markers >= {self.min_markers}, rms <= {self.max_reproj} px, "
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

        result = detect(frame, self.intrinsics.camera_matrix,
                        self.intrinsics.dist_coeffs, self.spec,
                        detector=self.detector, min_markers=self.min_markers)
        if result is None:
            self.reject(f"no board (need {self.min_markers} markers)")
            return
        if result.reproj_rms_px > self.max_reproj:
            self.reject(f"reprojection rms {result.reproj_rms_px:.2f} px is "
                        f"above {self.max_reproj}")
            return
        if not self.min_distance <= result.distance <= self.max_distance:
            self.reject(f"board at {result.distance * 100:.1f} cm is outside "
                        f"{self.min_distance * 100:.0f}-"
                        f"{self.max_distance * 100:.0f} cm")
            return

        matrix, frame_id = self.to_target_frame(result)
        if matrix is None:
            return

        self.publish_pose(matrix, frame_id)
        self.publish_debug(frame, result)
        self.accept()

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

    def publish_pose(self, matrix, frame_id):
        qx, qy, qz, qw = quaternion_from_matrix(matrix[:3, :3])
        message = PoseStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = frame_id
        message.pose.position.x = float(matrix[0, 3])
        message.pose.position.y = float(matrix[1, 3])
        message.pose.position.z = float(matrix[2, 3])
        message.pose.orientation.x = qx
        message.pose.orientation.y = qy
        message.pose.orientation.z = qz
        message.pose.orientation.w = qw
        self.pose_publisher.publish(message)
        self._published += 1
        if self._published == 1:
            t = matrix[:3, 3]
            self.get_logger().info(
                f"board at ({t[0]:+.4f}, {t[1]:+.4f}, {t[2]:+.4f}) m in "
                f"'{frame_id}'")

    def publish_debug(self, frame, result):
        if not self.publish_debug:
            return
        now = time.monotonic()
        if now - self._last_debug < self.debug_period:
            return
        self._last_debug = now
        annotated = annotate(frame, result, self.intrinsics.camera_matrix,
                             self.intrinsics.dist_coeffs)
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
