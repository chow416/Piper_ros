#!/usr/bin/env python3
"""Move the arm to the post-enable home pose and report the result.

Enabling the motors does not move the arm anywhere; it holds whatever pose the
arm is already in, and a parked arm rests with joint2 and joint3 pinned against
their stops. This script drives the arm to a deliberate, well-conditioned pose
so the press task never has to plan from a degenerate configuration.

The pose comes from `config/home_pose.yaml`. Values are validated against the
arm protocol limits before anything is sent, and warnings are printed for values
that fall outside the recommended range.

    ros2 run piper_elevator_manipulation go_home --dry-run
    ros2 run piper_elevator_manipulation go_home
    ros2 run piper_elevator_manipulation go_home --duration 12
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
import yaml

JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5"]

# Protocol limits, in radians, as published by the vendor SDK. A home pose
# outside these cannot be commanded at all.
PROTOCOL_LIMITS = {
    "joint1": (-2.6179, 2.6179),
    "joint2": (0.0, 3.14),
    "joint3": (-2.967, 0.0),
    "joint4": (-1.745, 1.745),
    "joint5": (-1.22, 1.22),
}
# Margin kept from every stop by the recommended home range.
HOME_MARGIN_RAD = 0.2618  # 15 degrees


def radians_to_degrees(value: float) -> float:
    return value * 180.0 / math.pi


def load_home_pose(path: Path) -> dict[str, float]:
    if not path.is_file():
        raise RuntimeError(f"home pose file not found: {path}")
    with path.open("r", encoding="utf-8") as handle:
        document = yaml.safe_load(handle)
    if not isinstance(document, dict) or "home_pose" not in document:
        raise RuntimeError(f"{path} has no 'home_pose' section")
    raw = document["home_pose"]
    if not isinstance(raw, dict):
        raise RuntimeError(f"{path}: 'home_pose' must be a mapping")

    pose: dict[str, float] = {}
    for name in JOINT_NAMES:
        if name not in raw:
            raise RuntimeError(f"{path}: 'home_pose' is missing {name}")
        value = float(raw[name])
        if not math.isfinite(value):
            raise RuntimeError(f"{path}: {name} is not finite")
        low, high = PROTOCOL_LIMITS[name]
        if value < low or value > high:
            raise RuntimeError(
                f"{path}: {name} = {value:.4f} rad "
                f"({radians_to_degrees(value):+.2f} deg) is outside the protocol "
                f"limits [{low:.4f}, {high:.4f}] rad "
                f"([{radians_to_degrees(low):+.1f}, {radians_to_degrees(high):+.1f}] deg)"
            )
        pose[name] = value
    return pose


def describe_pose(pose: dict[str, float]) -> None:
    for name in JOINT_NAMES:
        value = pose[name]
        low, high = PROTOCOL_LIMITS[name]
        margin = min(value - low, high - value)
        note = ""
        if margin < HOME_MARGIN_RAD:
            note = f"  WARNING: only {radians_to_degrees(margin):.1f} deg from a stop"
        print(f"  {name}: {value:+.4f} rad  ({radians_to_degrees(value):+8.2f} deg){note}")


class GoHome(Node):
    def __init__(self, controller: str) -> None:
        super().__init__("piper_go_home")
        self._latest: JointState | None = None
        self._action = f"/{controller}/follow_joint_trajectory"
        self._client = ActionClient(self, FollowJointTrajectory, self._action)
        self.create_subscription(JointState, "/joint_states", self._on_state, 10)

    def _on_state(self, message: JointState) -> None:
        self._latest = message

    def wait_for_state(self, timeout: float) -> dict[str, float]:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self._latest is None:
                continue
            by_name = dict(zip(self._latest.name, self._latest.position))
            if all(name in by_name for name in JOINT_NAMES):
                return {name: by_name[name] for name in JOINT_NAMES}
        raise RuntimeError(
            f"no /joint_states containing {JOINT_NAMES} within {timeout:.1f} s. "
            "Is the hardware bringup running?"
        )

    def wait_for_server(self, timeout: float) -> None:
        self.get_logger().info(f"waiting for action server {self._action} ...")
        if not self._client.wait_for_server(timeout_sec=timeout):
            raise RuntimeError(
                f"{self._action} is not available. Is arm_controller active? "
                "Check `ros2 control list_controllers`."
            )
        self.get_logger().info(f"{self._action} is available")

    def send(self, target: dict[str, float], duration_s: float) -> FollowJointTrajectory.Result:
        current = self.wait_for_state(2.0)

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(JOINT_NAMES)

        # Restate the measured pose first so the controller never has to
        # reconcile a gap between where it thinks the arm is and where it is.
        start_point = JointTrajectoryPoint()
        start_point.positions = [current[name] for name in JOINT_NAMES]
        start_point.time_from_start = Duration(sec=1, nanosec=0)
        goal.trajectory.points.append(start_point)

        end_point = JointTrajectoryPoint()
        end_point.positions = [target[name] for name in JOINT_NAMES]
        end_point.time_from_start = Duration(sec=int(duration_s), nanosec=0)
        goal.trajectory.points.append(end_point)

        largest = max(abs(target[name] - current[name]) for name in JOINT_NAMES)
        self.get_logger().info(
            f"sending home trajectory over {duration_s:.1f} s "
            f"(largest joint motion {radians_to_degrees(largest):.2f} deg)"
        )

        send_future = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=10.0)
        handle = send_future.result()
        if handle is None or not handle.accepted:
            raise RuntimeError("the controller rejected the home trajectory")

        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=duration_s + 20.0)
        wrapped = result_future.result()
        if wrapped is None:
            raise RuntimeError("timed out waiting for the home trajectory result")
        return wrapped.result


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument(
        "--config",
        type=Path,
        default=Path(get_package_share_directory("piper_elevator_manipulation"))
        / "config"
        / "home_pose.yaml",
        help="Home pose YAML file.",
    )
    parser.add_argument("--duration", type=float, default=8.0,
                        help="Travel time in seconds. Longer is gentler.")
    parser.add_argument("--controller", default="arm_controller")
    parser.add_argument("--dry-run", action="store_true",
                        help="Validate and print the pose without moving.")
    return parser


def main() -> int:
    args = build_parser().parse_args()

    rclpy.init()
    node = GoHome(args.controller)
    try:
        home = load_home_pose(args.config)
        print(f"\nhome pose from {args.config}:")
        describe_pose(home)

        node.wait_for_server(15.0)
        current = node.wait_for_state(5.0)
        print("\nmeasured pose before the move:")
        for name in JOINT_NAMES:
            print(f"  {name}: {current[name]:+.4f} rad  "
                  f"({radians_to_degrees(current[name]):+8.2f} deg)")

        if args.dry_run:
            print("\ndry run: nothing was sent.")
            return 0

        result = node.send(home, args.duration)
        print(f"\nresult error_code = {result.error_code}  ({result.error_string or 'ok'})")
        if result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            print("home trajectory did NOT report success", file=sys.stderr)
            return 1

        final = node.wait_for_state(5.0)
        print("\npose after the move:")
        worst = 0.0
        for name in JOINT_NAMES:
            error = final[name] - home[name]
            worst = max(worst, abs(error))
            print(f"  {name}: {final[name]:+.4f} rad  "
                  f"({radians_to_degrees(final[name]):+8.2f} deg)  "
                  f"error {radians_to_degrees(error):+7.3f} deg")
        print(f"\nworst error: {radians_to_degrees(worst):.3f} deg")
        return 0
    except Exception as error:  # noqa: BLE001 - top level reporting
        print(f"\nFAILED: {error}", file=sys.stderr)
        return 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
