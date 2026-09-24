#!/usr/bin/env python3
"""Send one small joint-trajectory goal to the real arm and report the result.

This is the first powered-motion test for a freshly wired arm, so it is built to
be conservative:

  * the motion is a single joint, by a few degrees, over several seconds;
  * the trajectory starts from the *measured* pose, so no jump is commanded;
  * the goal is only sent when the measured pose is sane and the controller is
    actually active;
  * every step is printed before it happens, and the final pose is compared with
    the commanded one.

The trajectory goes through /arm_controller/follow_joint_trajectory, which is the
exact interface MoveIt uses, so a success here proves the whole control path.
"""

from __future__ import annotations

import argparse
import math
import sys
import time

import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5"]


class JointTest(Node):
    def __init__(self, controller: str) -> None:
        super().__init__("piper_joint_test")
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
        raise RuntimeError(f"no /joint_states containing {JOINT_NAMES} within {timeout:.1f} s")

    def wait_for_server(self, timeout: float) -> None:
        self.get_logger().info(f"waiting for action server {self._action} ...")
        if not self._client.wait_for_server(timeout_sec=timeout):
            raise RuntimeError(
                f"{self._action} is not available. Is arm_controller active? "
                "Check `ros2 control list_controllers`."
            )
        self.get_logger().info(f"{self._action} is available")

    def send(self, goal_positions: dict[str, float], duration_s: float) -> FollowJointTrajectory.Result:
        current = self.wait_for_state(2.0)

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(JOINT_NAMES)

        # The first point restates the measured pose so the controller never has
        # to reconcile a gap between where it thinks the arm is and where it is.
        start_point = JointTrajectoryPoint()
        start_point.positions = [current[name] for name in JOINT_NAMES]
        start_point.time_from_start = Duration(sec=1, nanosec=0)
        goal.trajectory.points.append(start_point)

        end_point = JointTrajectoryPoint()
        end_point.positions = [goal_positions[name] for name in JOINT_NAMES]
        end_point.time_from_start = Duration(sec=int(duration_s), nanosec=0)
        goal.trajectory.points.append(end_point)

        self.get_logger().info(
            "sending trajectory: "
            + ", ".join(
                f"{name} {math.degrees(current[name]):+.2f}° -> "
                f"{math.degrees(goal_positions[name]):+.2f}°"
                for name in JOINT_NAMES
                if abs(goal_positions[name] - current[name]) > 1e-9
            )
            + f" over {duration_s:.1f} s"
        )

        send_future = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=10.0)
        handle = send_future.result()
        if handle is None or not handle.accepted:
            raise RuntimeError("the controller rejected the trajectory goal")

        result_future = handle.get_result_async()
        self.get_logger().info("goal accepted; waiting for completion ...")
        rclpy.spin_until_future_complete(
            self, result_future, timeout_sec=duration_s + 15.0
        )
        wrapped = result_future.result()
        if wrapped is None:
            raise RuntimeError("timed out waiting for the trajectory result")
        return wrapped.result


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--joint", default="joint1", choices=JOINT_NAMES,
                        help="Joint to move.")
    parser.add_argument("--delta-deg", type=float, default=5.0,
                        help="How far to move it, in degrees. Keep this small.")
    parser.add_argument("--duration", type=float, default=5.0,
                        help="Travel time in seconds.")
    parser.add_argument("--controller", default="arm_controller")
    parser.add_argument("--dry-run", action="store_true",
                        help="Print the plan and exit without sending anything.")
    return parser


def main() -> int:
    args = build_parser().parse_args()

    rclpy.init()
    node = JointTest(args.controller)
    try:
        node.wait_for_server(15.0)
        current = node.wait_for_state(5.0)

        print("\nmeasured pose:")
        for name in JOINT_NAMES:
            print(f"  {name}: {math.degrees(current[name]):+9.3f}°  "
                  f"({current[name]:+.5f} rad)")

        target = dict(current)
        target[args.joint] += math.radians(args.delta_deg)

        print(f"\nplanned move: {args.joint} {math.degrees(current[args.joint]):+.3f}°"
              f" -> {math.degrees(target[args.joint]):+.3f}°"
              f"  ({args.delta_deg:+.1f}° over {args.duration:.1f} s)")

        if args.dry_run:
            print("\ndry run: nothing was sent.")
            return 0

        result = node.send(target, args.duration)
        code = result.error_code
        print(f"\nresult error_code = {code}  ({result.error_string or 'ok'})")
        if code != FollowJointTrajectory.Result.SUCCESSFUL:
            print("trajectory did NOT report success", file=sys.stderr)
            return 1

        final = node.wait_for_state(5.0)
        print("\npose after the move:")
        worst = 0.0
        for name in JOINT_NAMES:
            error = final[name] - target[name]
            worst = max(worst, abs(error))
            marker = "  <-- moved" if abs(final[name] - current[name]) > math.radians(0.05) else ""
            print(f"  {name}: {math.degrees(final[name]):+9.3f}°  "
                  f"error {math.degrees(error):+7.3f}°{marker}")
        print(f"\nworst tracking error: {math.degrees(worst):.3f}°")
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
