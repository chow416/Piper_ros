"""Select a requested colored button and publish its stable RGB-D 3D pose."""
from collections import deque
import time

import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
import numpy as np
from PIL import Image as PilImage
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
import tf2_ros
import torch

from .detector import CLASSES, letterbox, load_detector, restore_boxes
from .localization import (
    physical_size_ratio, project_pixel, robust_depth, stable_candidate, transform_point,
)


NAME_TO_ID = {name.removeprefix("button_"): category for category, name in CLASSES.items()}


class ButtonTargetNode(Node):
    def __init__(self):
        super().__init__("button_target_node")
        defaults = {
            "checkpoint": "/ws/models/button_ssdlite_v2/best.pt",
            "rgb_topic": "/wrist_camera/depth/image_raw",
            "depth_topic": "/wrist_camera/depth/depth/image_raw",
            "camera_info_topic": "/wrist_camera/depth/camera_info",
            "target_color_topic": "/task/target_button_color",
            "output_pose_topic": "/perception/selected_button_pose",
            "debug_image_topic": "/perception/button_debug_image",
            "base_frame": "base_link",
            "default_target_color": "red",
            "score_threshold": 0.12,
            "depth_min": 0.10,
            "depth_max": 1.20,
            "depth_roi_radius": 4,
            "button_diameter": 0.050,
            "size_ratio_min": 0.45,
            "size_ratio_max": 1.80,
            "sync_tolerance": 0.12,
            "stability_window": 5,
            "stability_required": 3,
            "stability_tolerance": 0.015,
            "workspace_x_min": 0.20,
            "workspace_x_max": 0.45,
            "workspace_y_abs_max": 0.20,
            "workspace_z_min": 0.08,
            "workspace_z_max": 0.40,
            "publish_period": 0.20,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.bridge = CvBridge()
        self.model, checkpoint = load_detector(self.get_parameter("checkpoint").value)
        torch.set_num_threads(2)
        self.get_logger().info(
            f"Loaded button detector epoch {checkpoint['epoch']} from "
            f"{self.get_parameter('checkpoint').value}")

        self.target_color = self._normalize_color(self.get_parameter("default_target_color").value)
        self.depth_message = None
        self.camera_info = None
        self.samples = deque(maxlen=int(self.get_parameter("stability_window").value))
        self.last_publish_wall_time = 0.0
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        output_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pose_publisher = self.create_publisher(
            PoseStamped, self.get_parameter("output_pose_topic").value, output_qos)
        self.debug_publisher = self.create_publisher(
            Image, self.get_parameter("debug_image_topic").value, qos_profile_sensor_data)
        self.status_publisher = self.create_publisher(
            String, "/perception/button_status", QoSProfile(depth=10))
        self.create_subscription(
            String, self.get_parameter("target_color_topic").value,
            self._target_callback, QoSProfile(depth=10))
        self.create_subscription(
            CameraInfo, self.get_parameter("camera_info_topic").value,
            self._camera_info_callback, qos_profile_sensor_data)
        self.create_subscription(
            Image, self.get_parameter("depth_topic").value,
            self._depth_callback, qos_profile_sensor_data)
        self.create_subscription(
            Image, self.get_parameter("rgb_topic").value,
            self._rgb_callback, qos_profile_sensor_data)
        self._publish_status(f"waiting for {self.target_color} button")

    @staticmethod
    def _stamp_seconds(message):
        return message.header.stamp.sec + message.header.stamp.nanosec * 1e-9

    @staticmethod
    def _normalize_color(value):
        color = str(value).strip().lower().removeprefix("button_")
        if color not in NAME_TO_ID:
            raise ValueError(f"Unsupported button color '{value}'; choose {sorted(NAME_TO_ID)}")
        return color

    def _publish_status(self, value):
        message = String()
        message.data = value
        self.status_publisher.publish(message)

    def _target_callback(self, message):
        try:
            color = self._normalize_color(message.data)
        except ValueError as error:
            self.get_logger().warning(str(error))
            return
        if color != self.target_color:
            self.target_color = color
            self.samples.clear()
            self._publish_status(f"target changed to {color}; collecting stable frames")

    def _camera_info_callback(self, message):
        self.camera_info = message

    def _depth_callback(self, message):
        self.depth_message = message

    @torch.inference_mode()
    def _detect(self, bgr_image):
        pil = PilImage.fromarray(cv2.cvtColor(bgr_image, cv2.COLOR_BGR2RGB))
        tensor, geometry = letterbox(pil)
        output = self.model([tensor])[0]
        boxes = restore_boxes(output["boxes"], geometry, (bgr_image.shape[1], bgr_image.shape[0]))
        target_id = NAME_TO_ID[self.target_color]
        candidates = []
        threshold = float(self.get_parameter("score_threshold").value)
        for box, label, score in zip(boxes, output["labels"], output["scores"]):
            if int(label) != target_id or float(score) < threshold:
                continue
            x1, y1, x2, y2 = (float(value) for value in box)
            width, height = x2 - x1, y2 - y1
            if width > 0 and height > 0 and 0.45 <= width / height <= 2.2:
                candidates.append(((x1, y1, x2, y2), float(score)))
        return max(candidates, key=lambda item: item[1]) if candidates else None

    def _rgb_callback(self, message):
        if self.depth_message is None or self.camera_info is None:
            return
        if abs(self._stamp_seconds(message) - self._stamp_seconds(self.depth_message)) > float(
                self.get_parameter("sync_tolerance").value):
            self.samples.append(None)
            return
        bgr = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
        detection = self._detect(bgr)
        if detection is None:
            self.samples.append(None)
            self._publish_debug(message, bgr, None, "no target")
            return
        box, confidence = detection
        x1, y1, x2, y2 = box
        u, v = int(round((x1+x2)/2)), int(round((y1+y2)/2))
        depth_array = self.bridge.imgmsg_to_cv2(self.depth_message, desired_encoding="passthrough")
        scale = 0.001 if self.depth_message.encoding in ("16UC1", "mono16") else 1.0
        depth = robust_depth(
            depth_array, u, v, int(self.get_parameter("depth_roi_radius").value),
            float(self.get_parameter("depth_min").value),
            float(self.get_parameter("depth_max").value), scale)
        if depth is None:
            self.samples.append(None)
            self._publish_debug(message, bgr, detection, "invalid depth")
            return
        focal = (self.camera_info.k[0] + self.camera_info.k[4]) / 2.0
        ratio = physical_size_ratio(
            box, focal, depth, float(self.get_parameter("button_diameter").value))
        if not (float(self.get_parameter("size_ratio_min").value) <= ratio <=
                float(self.get_parameter("size_ratio_max").value)):
            self.samples.append(None)
            self._publish_debug(message, bgr, detection, f"size rejected {ratio:.2f}")
            return
        point_camera = project_pixel(u, v, depth, self.camera_info.k)
        try:
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.get_parameter("base_frame").value, message.header.frame_id,
                    Time.from_msg(message.header.stamp), timeout=Duration(seconds=0.05))
            except Exception:
                # Gazebo may publish the camera image a few milliseconds before
                # robot_state_publisher publishes TF for that simulation tick.
                # Detection happens while the arm is stationary, so the newest
                # transform is the appropriate bounded fallback.
                transform = self.tf_buffer.lookup_transform(
                    self.get_parameter("base_frame").value, message.header.frame_id,
                    Time(), timeout=Duration(seconds=0.10))
        except Exception as error:
            self.get_logger().warning(f"Transform unavailable: {error}", throttle_duration_sec=2.0)
            self.samples.append(None)
            self._publish_debug(message, bgr, detection, "transform unavailable")
            return
        t, q = transform.transform.translation, transform.transform.rotation
        point_base = transform_point(point_camera, (t.x,t.y,t.z), (q.x,q.y,q.z,q.w))
        if not self._inside_workspace(point_base):
            self.samples.append(None)
            self._publish_debug(message, bgr, detection, "workspace rejected")
            return
        self.samples.append((self.target_color, point_base, confidence, (u,v), depth))
        stable = stable_candidate(
            self.samples, int(self.get_parameter("stability_required").value),
            float(self.get_parameter("stability_tolerance").value))
        if stable is None:
            self._publish_debug(message, bgr, detection, "collecting stable frames")
            return
        now = time.monotonic()
        if now - self.last_publish_wall_time >= float(self.get_parameter("publish_period").value):
            color, point, score, pixel, selected_depth = stable
            pose = PoseStamped()
            pose.header = message.header
            pose.header.frame_id = self.get_parameter("base_frame").value
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = point
            # Controlled scene contract: panel normal is +X and link6 faces it after pi about Y.
            pose.pose.orientation.y = 1.0
            pose.pose.orientation.w = 0.0
            self.pose_publisher.publish(pose)
            self._publish_status(
                f"stable {color}: score={score:.3f} pixel=({pixel[0]:.1f},{pixel[1]:.1f}) "
                f"depth={selected_depth:.3f} base=({point[0]:.3f},{point[1]:.3f},{point[2]:.3f})")
            self.last_publish_wall_time = now
        self._publish_debug(message, bgr, detection, "stable")

    def _inside_workspace(self, point):
        return (float(self.get_parameter("workspace_x_min").value) <= point[0] <=
                float(self.get_parameter("workspace_x_max").value)
                and abs(point[1]) <= float(self.get_parameter("workspace_y_abs_max").value)
                and float(self.get_parameter("workspace_z_min").value) <= point[2] <=
                float(self.get_parameter("workspace_z_max").value))

    def _publish_debug(self, source, image, detection, status):
        debug = image.copy()
        if detection:
            (x1,y1,x2,y2),score = detection
            cv2.rectangle(debug,(round(x1),round(y1)),(round(x2),round(y2)),(0,255,255),2)
            cv2.putText(debug,f"{self.target_color} {score:.2f}",(round(x1),max(15,round(y1)-5)),
                        cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,255,255),1,cv2.LINE_AA)
        cv2.putText(debug,status,(5,20),cv2.FONT_HERSHEY_SIMPLEX,0.5,(255,255,255),1,cv2.LINE_AA)
        output = self.bridge.cv2_to_imgmsg(debug, encoding="bgr8")
        output.header = source.header
        self.debug_publisher.publish(output)


def main():
    rclpy.init()
    try:
        node = ButtonTargetNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ValueError, FileNotFoundError) as error:
        if not isinstance(error, KeyboardInterrupt):
            print(f"button_target_node: {error}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
