#!/usr/bin/env python3
"""Measure where a button is with respect to the board, using the arm itself.

The board-to-button transform is a physical fact about the panel. It can be
measured with a ruler, but a ruler has to guess where the board frame's origin
and axes are on the printed sheet, and it cannot measure the press axis at all.

The arm can measure both, and better. Jog the rod tip onto the button, and at
that instant:

    T_base <- board    from the perception node
    T_base <- press_tcp from TF, i.e. where the rod tip actually is

The offset is then the first expressed in the frame of the second:

    tip_in_board       = R_base_board^-1 . (p_tcp - p_base_board)
    press_axis_in_board = R_base_board^-1 . (R_base_tcp . Z)

The second line is the part a ruler cannot do. It says the press axis is
whichever way the tool was pointing when it was touching the button, which is
exactly the direction a person aims by hand -- and it comes out of the same
motion, for free.

Teaching at several arm poses is worth the extra minute
-------------------------------------------------------
The offset is a constant, so teaching it from three different arm poses has to
give three answers that agree. When they do not, the disagreement is not in the
offset: it is in the hand-eye calibration, because that is the only thing that
differs between the three measurements. This makes the check a direct read on
the calibration's quality, on the real panel, before anything presses anything.

Usage
-----
    ros2 run piper_marker_perception teach_press_offset.py --name red
    ros2 run piper_marker_perception teach_press_offset.py --name red --poses 3

Output is a YAML block to paste into `board.yaml`. It is printed rather than
written, because rewriting the file would discard the comments that explain what
the numbers mean.
"""

import argparse
import os
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from press_targets import (PressTarget, matrix_from_pose_stamped,  # noqa: E402
                           transform_to_matrix, yaml_block)

DEFAULT_BOARD_TOPIC = "/perception/board_pose"
DEFAULT_BASE_FRAME = "base_link"
DEFAULT_TOOL_FRAME = "press_tcp"

CONTRACT_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    durability=DurabilityPolicy.VOLATILE,
)

#: The tool's +Z is the pressing direction; see piper_press_tool.xacro.
TOOL_AXIS = np.array([0.0, 0.0, 1.0])

#: Across poses, offsets should agree to well under a millimetre if the
#: hand-eye calibration is good. Past this, the calibration is the suspect.
AGREEMENT_WARN_M = 0.002


class OffsetTeacher(Node):

    def __init__(self, board_topic, base_frame, tool_frame):
        super().__init__("teach_press_offset")
        self.base_frame = base_frame
        self.tool_frame = tool_frame
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)

        self.board_matrix = None
        self.board_stamp = None

        self.create_subscription(PoseStamped, board_topic, self.on_board,
                                 CONTRACT_QOS)
        self.get_logger().info(f"board pose from {board_topic}")

    def on_board(self, message):
        frame = message.header.frame_id
        if frame not in ("world", "base_link"):
            return
        try:
            self.board_matrix = matrix_from_pose_stamped(message)
        except ValueError:
            return
        self.board_stamp = message.header.stamp

    def spin_for(self, seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)

    def fresh_board(self, max_age=1.0):
        """The board pose, if it is recent enough to belong to this moment."""
        self.spin_for(0.3)
        if self.board_matrix is None or self.board_stamp is None:
            return None, "no board pose has arrived yet"
        age = (self.get_clock().now()
               - Time.from_msg(self.board_stamp)).nanoseconds * 1e-9
        if age > max_age:
            return None, f"board pose is {age:.2f} s old"
        return self.board_matrix, ""

    def tool_pose(self, timeout=1.0):
        """4x4 of the rod tip, or (None, reason)."""
        try:
            transform = self.buffer.lookup_transform(
                self.base_frame, self.tool_frame, Time(),
                timeout=rclpy.duration.Duration(seconds=timeout))
        except Exception as error:                          # noqa: BLE001
            return None, f"TF {self.base_frame} <- {self.tool_frame}: {error}"
        return transform_to_matrix(transform.transform), ""

    def measure(self):
        """One measurement, or (None, reason)."""
        board, reason = self.fresh_board()
        if board is None:
            return None, reason
        tool, reason = self.tool_pose()
        if tool is None:
            return None, reason

        rotation_board = board[:3, :3]
        tip = rotation_board.T @ (tool[:3, 3] - board[:3, 3])
        # The tool's +Z while it touches the button is the press axis, by
        # definition of how the operator posed it.
        axis = rotation_board.T @ (tool[:3, :3] @ TOOL_AXIS)
        return (tip, axis, board, tool), ""


def describe(name, tip, axis, board, tool):
    t = tip * 1000.0
    a = axis
    print(f"  板心     ({board[0, 3]:+.4f}, {board[1, 3]:+.4f}, "
          f"{board[2, 3]:+.4f}) m")
    print(f"  杆尖     ({tool[0, 3]:+.4f}, {tool[1, 3]:+.4f}, "
          f"{tool[2, 3]:+.4f}) m")
    print(f"  板系下   tip  = ({tip[0]:+.4f}, {tip[1]:+.4f}, {tip[2]:+.4f}) m"
          f"   = ({t[0]:+.1f}, {t[1]:+.1f}, {t[2]:+.1f}) mm")
    print(f"           axis = ({a[0]:+.4f}, {a[1]:+.4f}, {a[2]:+.4f})")
    tilt = np.degrees(np.arccos(np.clip(-a[2], -1.0, 1.0)))
    if tilt > 15.0:
        print(f"           ⚠️ 按压方向和板面法线差了 {tilt:.1f}° —— "
              f"是板子和按键不共面，还是杆尖没顶正？")


def main(argv=None):
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--name", required=True,
                        help="target name, e.g. red")
    parser.add_argument("--poses", type=int, default=1,
                        help="how many arm poses to teach from; >1 also checks "
                             "the hand-eye calibration's consistency")
    parser.add_argument("--board-topic", default=DEFAULT_BOARD_TOPIC)
    parser.add_argument("--base-frame", default=DEFAULT_BASE_FRAME)
    parser.add_argument("--tool-frame", default=DEFAULT_TOOL_FRAME)
    arguments = parser.parse_args(argv)

    rclpy.init()
    node = OffsetTeacher(arguments.board_topic, arguments.base_frame,
                         arguments.tool_frame)

    print("=" * 66)
    print(f"  教示按压偏移：{arguments.name}")
    print("=" * 66)
    print("  目标：把压杆尖端**顶到按键中心**，顶正、顶实。")
    print("  脚本记录那一刻杆尖相对板心的位置和朝向。")
    print()
    print("  ⚠️ 需要感知节点在跑，而且此刻能看到板子：")
    print("       ros2 run piper_marker_perception marker_locator.py ...")
    print(f"     （本脚本订阅 {arguments.board_topic}）")
    print()

    measurements = []
    try:
        for index in range(1, max(1, arguments.poses) + 1):
            if arguments.poses > 1:
                label = f"[{index}/{arguments.poses}] "
                input(f"  {label}把杆尖顶到按键上，摆好后回车...")
            else:
                input("  把杆尖顶到按键上，摆好后回车...")

            record, reason = node.measure()
            if record is None:
                print(f"  ✗ 测不了：{reason}")
                print("    确认板子在画面里、感知节点在发 /perception/board_pose")
                if arguments.poses == 1:
                    return 1
                continue
            tip, axis, board, tool = record
            describe(arguments.name, tip, axis, board, tool)
            measurements.append((tip, axis))
            print()

        if not measurements:
            print("  没有测到任何数据")
            return 1

        tips = np.array([m[0] for m in measurements])
        axes = np.array([m[1] for m in measurements])
        mean_tip = tips.mean(axis=0)
        mean_axis = axes.mean(axis=0)
        mean_axis /= np.linalg.norm(mean_axis)

        if len(measurements) > 1:
            spread = float(np.max(np.linalg.norm(tips - mean_tip, axis=1)))
            axis_spread = np.degrees(max(
                np.arccos(np.clip(float(a @ mean_axis), -1.0, 1.0))
                for a in axes))
            print(f"  位姿间一致性：位置最大偏差 {spread * 1000:.2f} mm，"
                  f"方向最大偏差 {axis_spread:.2f}°")
            if spread > AGREEMENT_WARN_M:
                print()
                print(f"  ⚠️ 偏差 {spread * 1000:.1f} mm 超过 "
                      f"{AGREEMENT_WARN_M * 1000:.0f} mm。")
                print("     这个偏移是个常量，不同位姿测出来应该一样。不一致说明")
                print("     **手眼标定**有问题（那是三次测量之间唯一变化的东西），")
                print("     不是这次教示不准。先回去查标定。")
            print()

        target = PressTarget(arguments.name, mean_tip, mean_axis, taught=True)
        print("  把下面这段贴进 config/board.yaml 的 targets: 下面：")
        print()
        print(yaml_block(target))
        print()
        if len(measurements) > 1:
            print("  （上面是多次测量的平均值）")
        print("  贴完重启桥节点：ros2 run piper_marker_perception "
              "press_target_bridge.py -p target:=" + arguments.name)
        return 0
    except KeyboardInterrupt:
        print("\n  已中断")
        return 130
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
