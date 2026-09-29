"""Integration check for press_target_bridge: fake board pose in, target out.

The offline self-test proves the maths. This proves the glue: parameters,
QoS, the taught gate, freshness, the workspace box, and the numbers that
actually reach /press/target_pose.
"""
import os
import subprocess
import sys
import tempfile
import time

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)

HERE = "/ws/src/Piper_ros/src/piper_marker_perception/scripts"
sys.path.insert(0, HERE)
from press_targets import (quaternion_from_matrix,    # noqa: E402
                           quaternion_to_matrix)

CONTRACT_QOS = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                          history=HistoryPolicy.KEEP_LAST, depth=1,
                          durability=DurabilityPolicy.VOLATILE)

FAILURES = []


def check(label, ok, detail=""):
    print(f"  [{'PASS' if ok else 'FAIL'}] {label}"
          f"{('  ' + detail) if detail else ''}")
    if not ok:
        FAILURES.append(label)


def write_config(path, taught, tip, axis=None):
    entry = {"taught": taught, "tip_in_board": tip}
    if axis:
        entry["press_axis_in_board"] = axis
    with open(path, "w") as handle:
        yaml.safe_dump({"targets": {"red": entry}}, handle)


def stub_stamp():
    """Now, as a ROS stamp.

    A real publisher has to stamp its observations, and the bridge refuses a
    zero stamp rather than measuring freshness against 1970, so the test has to
    stamp them too.
    """
    import rclpy.clock
    return rclpy.clock.Clock().now().to_msg()


def board_pose_message(frame="base_link"):
    """Board on a vertical panel at x=0.30, centre 0.24 m up, facing the arm."""
    board = np.array([
        [0.0, 0.0, -1.0, 0.30],
        [-1.0, 0.0, 0.0, 0.00],
        [0.0, 1.0, 0.0, 0.24],
        [0.0, 0.0, 0.0, 1.00],
    ])
    qx, qy, qz, qw = quaternion_from_matrix(board[:3, :3])
    message = PoseStamped()
    message.header.frame_id = frame
    message.header.stamp = stub_stamp()
    message.pose.position.x = float(board[0, 3])
    message.pose.position.y = float(board[1, 3])
    message.pose.position.z = float(board[2, 3])
    message.pose.orientation.x = qx
    message.pose.orientation.y = qy
    message.pose.orientation.z = qz
    message.pose.orientation.w = qw
    return message, board


class Harness(Node):
    def __init__(self):
        super().__init__("bridge_test_harness")
        self.publisher = self.create_publisher(PoseStamped, "/perception/board_pose",
                                               CONTRACT_QOS)
        self.received = []
        self.create_subscription(PoseStamped, "/press/target_pose",
                                 lambda m: self.received.append(m), CONTRACT_QOS)

    def spin_for(self, seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)

    def publish_board(self, message, count=6):
        for _ in range(count):
            self.publisher.publish(message)
            self.spin_for(0.05)


def run_bridge(config_path, extra=None):
    command = ["python3", os.path.join(HERE, "press_target_bridge.py"),
               "--ros-args",
               "-p", f"config:={config_path}", "-p", "target:=red",
               "-p", "publish_rate_hz:=20.0"]
    if extra:
        command += extra
    return subprocess.Popen(command, stdout=subprocess.PIPE,
                            stderr=subprocess.STDOUT, text=True)


def main():
    work = tempfile.mkdtemp(prefix="bridge_")
    rclpy.init()
    harness = Harness()

    # ---- 1. a taught target produces the hand-computed numbers -------------
    print("① 已教示的目标 -> 正确的 /press/target_pose")
    config = os.path.join(work, "board.yaml")
    write_config(config, taught=True, tip={"x": 0.0, "y": -0.080, "z": 0.0})
    bridge = run_bridge(config)
    try:
        harness.spin_for(2.0)                       # let the bridge subscribe
        message, board = board_pose_message()
        harness.publish_board(message)
        harness.spin_for(1.5)
        check("收到了目标", len(harness.received) > 0,
              f"({len(harness.received)} 条)")
        if harness.received:
            target = harness.received[-1]
            position = np.array([target.pose.position.x, target.pose.position.y,
                                 target.pose.position.z])
            q = target.pose.orientation
            direction = quaternion_to_matrix(q.x, q.y, q.z, q.w)[:, 2]
            print(f"        位置 {np.round(position, 4)}  期望 [0.3 0. 0.16]")
            print(f"        方向 {np.round(direction, 4)}  期望 [1. 0. 0.]")
            check("frame_id = base_link", target.header.frame_id == "base_link")
            check("位置 == (0.3000, 0.0000, 0.1600)",
                  np.allclose(position, [0.30, 0.0, 0.16], atol=1e-6))
            check("方向 == (+1, 0, 0)",
                  np.allclose(direction, [1.0, 0.0, 0.0], atol=1e-6))
            check(f"四元数 == (0, 0.707107, 0, 0.707107) "
                  f"得到 ({q.x:.6f}, {q.y:.6f}, {q.z:.6f}, {q.w:.6f})",
                  abs(q.y - 0.7071068) < 1e-6 and abs(q.w - 0.7071068) < 1e-6
                  and abs(q.x) < 1e-9 and abs(q.z) < 1e-9)
    finally:
        bridge.terminate()
        bridge.wait(timeout=5)

    # ---- 2. the taught gate ----------------------------------------------
    print("② taught: false 时拒绝发布")
    config2 = os.path.join(work, "untaught.yaml")
    write_config(config2, taught=False, tip={"x": 0.0, "y": -0.080, "z": 0.0})
    bridge = run_bridge(config2)
    try:
        harness.received.clear()
        harness.spin_for(2.0)
        message, _ = board_pose_message()
        harness.publish_board(message)
        harness.spin_for(1.5)
        check("未教示 -> 一条都没发", len(harness.received) == 0,
              f"({len(harness.received)} 条)")
    finally:
        bridge.terminate()
        output = bridge.communicate(timeout=5)[0]
        check("日志里说明了原因和怎么修",
              "taught: false" in output and "teach_press_offset" in output)
        bridge.wait(timeout=5)

    # ---- 3. the workspace box --------------------------------------------
    print("③ 目标落到工作空间盒外 -> 拒绝")
    config3 = os.path.join(work, "outside.yaml")
    write_config(config3, taught=True, tip={"x": 0.0, "y": 0.0, "z": 0.80})
    bridge = run_bridge(config3)
    try:
        harness.received.clear()
        harness.spin_for(2.0)
        message, _ = board_pose_message()
        harness.publish_board(message)
        harness.spin_for(1.5)
        check("1.04 m 高处的点被拒", len(harness.received) == 0,
              f"({len(harness.received)} 条)")
    finally:
        bridge.terminate()
        output = bridge.communicate(timeout=5)[0]
        check("日志里有工作空间盒的范围", "outside the accepted workspace box"
              in output)
        bridge.wait(timeout=5)

    # ---- 4. staleness ------------------------------------------------------
    print("④ 板子位姿过期 -> 停发")
    config4 = os.path.join(work, "fresh.yaml")
    write_config(config4, taught=True, tip={"x": 0.0, "y": -0.080, "z": 0.0})
    bridge = run_bridge(config4, extra=["-p", "max_age_s:=0.5"])
    try:
        harness.received.clear()
        harness.spin_for(2.0)
        message, _ = board_pose_message()
        harness.publish_board(message, count=4)
        harness.spin_for(0.4)
        early = len(harness.received)
        harness.spin_for(1.5)                      # stop publishing, wait it out
        late = len(harness.received)
        check(f"新鲜时在发（{early} 条）", early > 0)
        check(f"过期后不再发（{early} -> {late}）", late == early)
    finally:
        bridge.terminate()
        bridge.wait(timeout=5)

    # ---- 5. a camera-frame board pose is refused --------------------------
    print("⑤ 板子位姿在相机系里 -> 拒绝")
    config5 = os.path.join(work, "frame.yaml")
    write_config(config5, taught=True, tip={"x": 0.0, "y": -0.080, "z": 0.0})
    bridge = run_bridge(config5)
    try:
        harness.received.clear()
        harness.spin_for(2.0)
        message, _ = board_pose_message(frame="wrist_depth_camera_optical_frame")
        harness.publish_board(message)
        harness.spin_for(1.5)
        check("相机系 -> 一条都没发", len(harness.received) == 0,
              f"({len(harness.received)} 条)")
    finally:
        bridge.terminate()
        output = bridge.communicate(timeout=5)[0]
        check("日志里报了 frame 名", "wrist_depth_camera_optical_frame" in output)
        bridge.wait(timeout=5)

    harness.destroy_node()
    rclpy.shutdown()
    print()
    if FAILURES:
        print(f"  FAILURES: {FAILURES}")
        return 1
    print("  全部通过")
    return 0


if __name__ == "__main__":
    sys.exit(main())
