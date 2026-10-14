#!/usr/bin/env python3
"""Turn "the board is here" into "press there".

Subscribes to the board pose, applies the constant board-to-button transform from
`board.yaml`, and publishes the point and direction the press task consumes.
See `press_targets.py` for why the transform is stored the way it is, and
`piper_elevator_manipulation/docs/perception_interface.md` for the contract this
has to satisfy.

Why this is a separate node from the one that finds the board
-------------------------------------------------------------
Three reasons, and the first is the one that matters:

  - It can be tested with no hardware at all. Publish a made-up board pose and
    check the numbers that come out. That is how the sign conventions below get
    settled before a rod is anywhere near a button.

  - The two fail for different reasons. "The board is not visible" and "the
    button is outside the workspace" want different fixes, and separate nodes
    let each say what it means.

  - The two change at different rates. Re-taping the board means re-teaching the
    offset and touching nothing else; changing the lens means the reverse.

What it publishes
-----------------
`PoseStamped`, `position` = where the rod tip should land, `orientation` +Z =
the direction to press. The task reads only that axis, and takes the target as
absolute rather than as a displacement, so the point is the landing point and
not an approach point -- the task subtracts its own `approach_distance`.

Reaching the destination at all is not the hard part; arriving with the
conventions intact is. Two of them are easy to get backwards and both are
checked loudly below: the press axis is the board's -Z (a surface *outward*
normal is the negation), and the point is the rod *tip*, not the flange.
"""

import argparse
import json
import os
import sys

import rclpy
import numpy as np
from geometry_msgs.msg import PoseStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from press_targets import (ConfigError, PressTargetSet,   # noqa: E402
                           matrix_from_pose_stamped, quaternion_for_direction,
                           transform_to_matrix)
from aruco_board import resolve_config_path              # noqa: E402

DEFAULT_BOARD_TOPIC = "/perception/board_pose"
DEFAULT_TARGET_TOPIC = "/press/target_pose"
DEFAULT_FRAME = "base_link"

#: The hand-eye result is a property of the rig, not of the perception code, so
#: it stays where the rig keeps it and `tools/press_board_centre.py` reads the
#: same file. Two readers of one file rather than two copies of one matrix.
DEFAULT_HAND_EYE = "/ws/T_flange_cam.json"

#: The link the hand-eye is measured against: the calibration gives
#: `flange <- camera_optical`, so a camera-frame board pose only becomes a
#: base-frame one after this link's pose is known.
DEFAULT_FLANGE_FRAME = "press_tool_link"

#: `world` and `base_link` are the same physical frame -- the URDF ties them with
#: a fixed joint whose origin is all zeros -- so a board pose expressed in either
#: can be republished in either.
SAME_FRAME = ("world", "base_link")

#: Sanity envelope, matching the task node's own default so a target this node
#: accepts is not one the task will silently drop.
DEFAULT_WORKSPACE_MIN = (-0.30, -0.50, -0.02)
DEFAULT_WORKSPACE_MAX = (0.65, 0.50, 0.80)

#: The contract wants RELIABLE, KEEP_LAST 1, VOLATILE -- a latched or lossy
#: publisher here would either replay a stale target or drop a live one.
CONTRACT_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    durability=DurabilityPolicy.VOLATILE,
)


class PressTargetBridge(Node):

    def __init__(self, parameter_overrides=None):
        super().__init__("press_target_bridge",
                         parameter_overrides=parameter_overrides or [])

        self.declare_parameter("target", "")
        self.declare_parameter("config", "")
        self.declare_parameter("board_topic", DEFAULT_BOARD_TOPIC)
        self.declare_parameter("topic", DEFAULT_TARGET_TOPIC)
        self.declare_parameter("frame", DEFAULT_FRAME)
        self.declare_parameter("publish_rate_hz", 5.0)
        self.declare_parameter("max_age_s", 1.0)
        self.declare_parameter("direction_sign", 1.0)
        self.declare_parameter("require_taught", True)
        #: Which of the button's two measurements this instance applies. The
        #: node is otherwise identical either way -- the two channels differ
        #: only in which constant is added and which board pose it arrives on,
        #: so running both is two instances with different topics. They stay
        #: separate nodes on purpose: a coarse pose published on the refined
        #: topic fuses into a wrong correction while every log line still looks
        #: healthy, and separate instances leave nowhere for that to happen.
        self.declare_parameter("offset", "coarse")
        #: Used only for board poses that arrive in the camera frame. Same file
        #: and same composition as `tools/press_board_centre.py` -- deliberately,
        #: because asking TF for `base <- camera` would use the camera's
        #: *nominal* mounting in the URDF, which is not the calibrated one.
        self.declare_parameter("hand_eye", DEFAULT_HAND_EYE)
        self.declare_parameter("flange_frame", DEFAULT_FLANGE_FRAME)
        self.declare_parameter("workspace_min", list(DEFAULT_WORKSPACE_MIN))
        self.declare_parameter("workspace_max", list(DEFAULT_WORKSPACE_MAX))

        name = self.get_parameter("target").value
        if not name:
            raise ConfigError(
                "the 'target' parameter is required and names which button to "
                "aim at.\n"
                "    Parameters only count after --ros-args; without it rclpy "
                "reads\n"
                "    'target:=red' as a remap rule and the parameter never "
                "arrives.\n"
                "        ros2 run piper_marker_perception press_target_bridge.py "
                "--ros-args -p target:=red")

        config_path = resolve_config_path(
            self.get_parameter("config").value or "config/board.yaml")
        targets = PressTargetSet.from_yaml(config_path)
        self.target = targets[name]
        self.target_name = name

        self.frame = self.get_parameter("frame").value
        self.max_age = float(self.get_parameter("max_age_s").value)
        self.direction_sign = float(self.get_parameter("direction_sign").value)
        self.require_taught = bool(self.get_parameter("require_taught").value)
        self.offset = str(self.get_parameter("offset").value)
        if self.offset not in ("coarse", "refine"):
            raise ConfigError(
                f"offset must be 'coarse' or 'refine', got {self.offset!r}")
        self.taught = (self.target.refine_taught if self.offset == "refine"
                       else self.target.taught)

        self.hand_eye_path = str(self.get_parameter("hand_eye").value)
        self.hand_eye = self._load_hand_eye(self.hand_eye_path)
        self.flange_frame = str(self.get_parameter("flange_frame").value)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.last_frame_logged = ""
        self.workspace_min = np.array(
            self.get_parameter("workspace_min").value, dtype=float)
        self.workspace_max = np.array(
            self.get_parameter("workspace_max").value, dtype=float)

        board_topic = self.get_parameter("board_topic").value
        topic = self.get_parameter("topic").value
        rate = float(self.get_parameter("publish_rate_hz").value)

        self.create_subscription(PoseStamped, board_topic, self.on_board,
                                 CONTRACT_QOS)
        self.publisher = self.create_publisher(PoseStamped, topic, CONTRACT_QOS)

        self.board_matrix = None
        self.board_stamp = None
        self.board_frame = ""
        self.last_reason = ""
        self.rejected = 0
        self.published = 0

        self.create_timer(1.0 / max(0.1, rate), self.publish)

        self.get_logger().info(
            f"target '{self.target_name}' [{self.offset}]  "
            f"{self.target.describe()}")
        self.get_logger().info(
            f"board  {board_topic}  ->  target  {topic}  @ {rate:g} Hz  "
            f"frame '{self.frame}'")
        if not self.taught:
            marker = ("refine_taught" if self.offset == "refine" else "taught")
            self.get_logger().error(
                f"target '{self.target_name}' is marked {marker}: false. Its "
                f"numbers are placeholders, so nothing will be published. "
                f"Measure it first, then paste the result into "
                f"{config_path}")

    # -- input ------------------------------------------------------------

    def _load_hand_eye(self, path):
        """`flange <- camera_optical` as a 4x4, or None with the reason logged.

        Absence is not fatal: a deployment whose board poses already arrive in
        the robot frame never needs this file, so a missing one only disables
        the camera-frame input rather than the node.
        """
        try:
            with open(path, encoding="utf-8") as handle:
                matrix = np.array(json.load(handle)["T_flange_cam_4x4"],
                                  dtype=float)
        except (OSError, KeyError, ValueError, TypeError) as error:
            self.get_logger().warn(
                f"cannot read the hand-eye file '{path}' ({error}). Board poses "
                f"already in {SAME_FRAME} still work; poses in the camera frame "
                f"will be refused.")
            return None
        if matrix.shape != (4, 4):
            self.get_logger().warn(f"{path}: T_flange_cam_4x4 is not 4x4")
            return None
        return matrix

    def on_board(self, message):
        frame = message.header.frame_id
        if not frame:
            self.reject("board pose has an empty frame_id, so there is no way "
                        "to know where it is")
            return
        try:
            self.board_matrix = matrix_from_pose_stamped(message)
        except ValueError as error:
            self.reject("board pose has no usable orientation",
                        str(error).splitlines()[0])
            return
        if frame not in SAME_FRAME and frame != self.last_frame_logged:
            # Not refused: a pose in the camera frame is the normal input for the
            # refined channel, and turning one into a base-frame pose is exactly
            # what the hand-eye file is for. Said once, because at 5 Hz a line
            # per message would be noise.
            self.last_frame_logged = frame
            self.get_logger().info(
                f"board poses arrive in '{frame}'; composing '{self.hand_eye_path}' "
                f"on top of TF '{self.frame} <- {self.flange_frame}'")
        self.board_stamp = message.header.stamp
        self.board_frame = frame
        self.accept()

    # -- output -----------------------------------------------------------

    def publish(self):
        if self.board_matrix is None:
            return
        if self.require_taught and not self.taught:
            return

        age = self.age_of_board()
        if age is None:
            self.reject("board pose has no usable stamp (zero or absent)",
                        "the publisher must set header.stamp")
            return
        if age > self.max_age:
            self.reject("board pose is stale",
                        f"{age:.2f} s old, past the {self.max_age:.2f} s limit")
            return

        board_in_base = self.board_in_base()
        if board_in_base is None:
            return                     # board_in_base has already said why

        if self.offset == "refine":
            position, direction = self.target.resolve_refine(
                board_in_base, self.direction_sign)
        else:
            position, direction = self.target.resolve(board_in_base,
                                                      self.direction_sign)

        if not np.all(np.isfinite(position)) or not np.all(np.isfinite(direction)):
            self.reject("resolved target contains a non-finite number")
            return

        outside = ((position < self.workspace_min)
                   | (position > self.workspace_max))
        if np.any(outside):
            self.reject(
                "the resolved point is outside the accepted workspace box",
                f"({position[0]:.3f}, {position[1]:.3f}, {position[2]:.3f}) vs "
                f"{self.workspace_min.tolist()} .. {self.workspace_max.tolist()}")
            return

        qx, qy, qz, qw = quaternion_for_direction(direction)
        message = PoseStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.frame
        message.pose.position.x = float(position[0])
        message.pose.position.y = float(position[1])
        message.pose.position.z = float(position[2])
        message.pose.orientation.x = qx
        message.pose.orientation.y = qy
        message.pose.orientation.z = qz
        message.pose.orientation.w = qw
        self.publisher.publish(message)
        self.published += 1

        if self.published == 1:
            self.get_logger().info(
                f"publishing '{self.target_name}': press rod tip at "
                f"({position[0]:.4f}, {position[1]:.4f}, {position[2]:.4f}), "
                f"direction ({direction[0]:+.3f}, {direction[1]:+.3f}, "
                f"{direction[2]:+.3f})")
        self.accept()

    def board_in_base(self):
        """The board pose as a base-frame 4x4, or None with a reason logged.

        Two shapes arrive here and they need different handling. A pose already
        in the robot frame is used as it stands. A pose in the camera's optical
        frame needs the hand-eye composed on top of the flange pose -- which is
        the same composition `tools/press_board_centre.py` performs from the same
        file. Asking TF for `base <- camera` instead would be simpler and wrong:
        the URDF carries the camera's *nominal* mounting, and the calibrated one
        lives only in that JSON.
        """
        if self.board_frame in SAME_FRAME:
            return self.board_matrix
        if self.hand_eye is None:
            self.reject("board pose is in a frame that needs the hand-eye "
                        "file, and that could not be read", self.board_frame)
            return None
        try:
            flange = self.tf_buffer.lookup_transform(
                self.frame, self.flange_frame, Time())
        except Exception as error:                          # noqa: BLE001
            self.reject(f"TF {self.frame} <- {self.flange_frame} is unavailable",
                        str(error).splitlines()[0])
            return None
        return (transform_to_matrix(flange.transform) @ self.hand_eye
                @ self.board_matrix)

    def age_of_board(self):
        if self.board_stamp is None:
            return None
        if self.board_stamp.sec == 0 and self.board_stamp.nanosec == 0:
            # An unstamped publisher would otherwise look like a board observed
            # in 1970, and the staleness message would be nonsense.
            return None
        return (self.get_clock().now()
                - Time.from_msg(self.board_stamp)).nanoseconds * 1e-9

    # -- logging that does not flood --------------------------------------

    def reject(self, reason, detail=""):
        """Record why nothing is being published, without repeating ourselves.

        A 5 Hz publisher repeating the same complaint would bury everything
        else, so a reason is logged when it appears and counted afterwards.

        That only works if `reason` is a *category*. Anything folded into it
        that changes message to message -- a measured age, a distance, a
        residual -- makes every occurrence a brand new reason, and the
        de-duplication stops working with no symptom other than a log that
        keeps filling. Measured here at five warning lines a second,
        indefinitely. The variable part belongs in `detail`, which is printed
        once, alongside the category.
        """
        if reason == self.last_reason:
            self.rejected += 1
            return
        if self.last_reason:
            self.get_logger().warn(
                f"...previous reason recurred {self.rejected} times")
        self.last_reason = reason
        # Counting starts over with each new reason, so the number printed
        # beside the next one is about that one and not a running total.
        self.rejected = 1
        message = f"not publishing: {reason}"
        if detail:
            message += f" -- {detail}"
        self.get_logger().warn(message)

    def accept(self):
        if self.last_reason:
            self.get_logger().info(f"recovered after {self.rejected} refusals")
            self.last_reason = ""
            self.rejected = 0


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--target", default=None,
                        help="which button to aim at, e.g. red")
    parser.add_argument("--config", default=None,
                        help="board.yaml to read")
    parser.add_argument("--topic", default=None,
                        help="output topic")
    arguments, _ = parser.parse_known_args(argv)

    overrides = []
    for name in ("target", "config", "topic"):
        value = getattr(arguments, name, None)
        if value is not None:
            overrides.append(Parameter(name, value=value))

    # Parameters passed on the command line must sit after --ros-args, because
    # that is the only thing that tells rclpy they are parameters and not remap
    # rules. `--target red` is offered as the way out of remembering that.
    rclpy.init()
    try:
        node = PressTargetBridge(parameter_overrides=overrides)
    except (ConfigError, KeyError) as error:
        print(f"error: {error}", file=sys.stderr)
        rclpy.shutdown()
        return 2

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        # SIGTERM tears the context down underneath spin(), which then raises
        # out of the executor. An ordinary way for this to end.
        pass
    finally:
        # rclpy shuts the context down from its own signal handler, so the
        # cleanup here has to tolerate already having been torn down.
        try:
            node.destroy_node()
        except Exception:                                   # noqa: BLE001
            pass
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
