#!/usr/bin/env python3
import json
import math
from pathlib import Path
import shutil
import sys
import threading
import time

import cv2
from cv_bridge import CvBridge
from gazebo_msgs.msg import EntityState
from gazebo_msgs.srv import SetEntityState
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformException, TransformListener


BUTTONS = (
    {"model": "elevator_button_blue", "category_id": 1, "name": "button_blue", "z": 0.320, "radius": 0.023},
    {"model": "elevator_button_red", "category_id": 2, "name": "button_red", "z": 0.240, "radius": 0.025},
    {"model": "elevator_button_green", "category_id": 3, "name": "button_green", "z": 0.160, "radius": 0.023},
)
PANEL_MODEL = "elevator_button_panel"
BUTTON_X = 0.306
BUTTON_HALF_DEPTH = 0.006


def quaternion_from_rpy(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    return np.array([
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    ], dtype=np.float64)


def quaternion_multiply(left, right):
    x1, y1, z1, w1 = left
    x2, y2, z2, w2 = right
    return np.array([
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    ], dtype=np.float64)


def rotation_from_quaternion(quaternion):
    x, y, z, w = quaternion / np.linalg.norm(quaternion)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ], dtype=np.float64)


def pose_message(position, quaternion):
    state = EntityState()
    state.pose.position.x = float(position[0])
    state.pose.position.y = float(position[1])
    state.pose.position.z = float(position[2])
    state.pose.orientation.x = float(quaternion[0])
    state.pose.orientation.y = float(quaternion[1])
    state.pose.orientation.z = float(quaternion[2])
    state.pose.orientation.w = float(quaternion[3])
    state.reference_frame = "world"
    return state


class ButtonDatasetGenerator(Node):
    def __init__(self):
        super().__init__("button_dataset_generator")
        self._declare_parameters()

        self.output_dir = Path(self.get_parameter("output_dir").value)
        self.num_samples = int(self.get_parameter("num_samples").value)
        self.random_seed = int(self.get_parameter("random_seed").value)
        self.settle_seconds = float(self.get_parameter("settle_seconds").value)
        self.jpeg_quality = int(self.get_parameter("jpeg_quality").value)
        self.overwrite = bool(self.get_parameter("overwrite").value)
        self.world_frame = str(self.get_parameter("world_frame").value)
        self.rng = np.random.default_rng(self.random_seed)

        if self.num_samples <= 0:
            raise ValueError("num_samples must be positive")
        self._prepare_output()

        self.bridge = CvBridge()
        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.state_client = self.create_client(SetEntityState, "/set_entity_state")

        image_topic = str(self.get_parameter("image_topic").value)
        info_topic = str(self.get_parameter("camera_info_topic").value)
        self.create_subscription(Image, image_topic, self._image_callback, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, info_topic, self._info_callback, qos_profile_sensor_data)

        self.latest_image = None
        self.camera_info = None
        self.phase = "waiting"
        self.state_futures = []
        self.ready_after = None
        self.current_button_poses = {}
        self.sample_index = 0
        self.annotation_id = 1
        self.negative_count = 0
        self.images = []
        self.annotations = []
        self.completed = False
        self.timer = self.create_timer(0.05, self._tick)
        self.get_logger().info(
            f"Preparing {self.num_samples} synthetic samples in {self.output_dir}"
        )

    def _declare_parameters(self):
        defaults = {
            "output_dir": "/ws/data/button_dataset_v1",
            "image_topic": "/wrist_camera/depth/image_raw",
            "camera_info_topic": "/wrist_camera/depth/camera_info",
            "world_frame": "world",
            "num_samples": 500,
            "random_seed": 17,
            "settle_seconds": 0.18,
            "jpeg_quality": 95,
            "overwrite": False,
            "translation_x_min": -0.015,
            "translation_x_max": 0.025,
            "translation_y_abs": 0.025,
            "translation_z_abs": 0.025,
            "roll_abs_deg": 4.0,
            "pitch_abs_deg": 5.0,
            "yaw_abs_deg": 6.0,
            "augment_images": True,
            "negative_fraction": 0.10,
            "brightness_abs": 24.0,
            "contrast_min": 0.80,
            "contrast_max": 1.20,
            "saturation_min": 0.75,
            "saturation_max": 1.25,
            "noise_sigma_max": 4.0,
            "blur_probability": 0.20,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    def _prepare_output(self):
        if self.output_dir.exists():
            if not self.overwrite:
                raise FileExistsError(
                    f"{self.output_dir} already exists; set overwrite:=true or choose another path"
                )
            shutil.rmtree(self.output_dir)
        (self.output_dir / "images").mkdir(parents=True)
        (self.output_dir / "annotations").mkdir(parents=True)

    def _image_callback(self, message):
        self.latest_image = message

    def _info_callback(self, message):
        self.camera_info = message

    def _tick(self):
        if self.completed:
            return
        if self.phase == "waiting":
            if self.latest_image is None or self.camera_info is None:
                return
            if not self.state_client.service_is_ready():
                return
            self._begin_randomization()
            return

        if self.phase == "moving":
            if not all(future.done() for future in self.state_futures):
                return
            for future in self.state_futures:
                response = future.result()
                if response is None or not response.success:
                    self._fail("Gazebo rejected an entity pose update")
                    return
            self.ready_after = self.get_clock().now() + Duration(seconds=self.settle_seconds)
            self.phase = "settling"
            return

        if self.phase == "settling":
            if self.get_clock().now() < self.ready_after:
                return
            if Time.from_msg(self.latest_image.header.stamp) < self.ready_after:
                return
            if self._capture_sample():
                self.sample_index += 1
                if self.sample_index >= self.num_samples:
                    self._finish()
                else:
                    self._begin_randomization()
            else:
                self._begin_randomization()

    def _begin_randomization(self):
        tx = self.rng.uniform(
            float(self.get_parameter("translation_x_min").value),
            float(self.get_parameter("translation_x_max").value),
        )
        ty_abs = float(self.get_parameter("translation_y_abs").value)
        tz_abs = float(self.get_parameter("translation_z_abs").value)
        translation = np.array([
            tx,
            self.rng.uniform(-ty_abs, ty_abs),
            self.rng.uniform(-tz_abs, tz_abs),
        ])
        rpy = []
        for parameter in ("roll_abs_deg", "pitch_abs_deg", "yaw_abs_deg"):
            bound = math.radians(float(self.get_parameter(parameter).value))
            rpy.append(self.rng.uniform(-bound, bound))
        group_quaternion = quaternion_from_rpy(*rpy)
        group_rotation = rotation_from_quaternion(group_quaternion)

        requests = []
        panel_state = pose_message(translation, group_quaternion)
        panel_state.name = PANEL_MODEL
        requests.append(panel_state)

        base_button_quaternion = quaternion_from_rpy(0.0, math.pi / 2.0, 0.0)
        negative_fraction = float(self.get_parameter("negative_fraction").value)
        self.current_is_negative = self.rng.random() < negative_fraction
        self.current_button_poses = {}
        for index, button in enumerate(BUTTONS):
            base_position = np.array([BUTTON_X, 0.0, button["z"]])
            if self.current_is_negative:
                position = np.array([5.0, 5.0, 5.0 + 0.2 * index])
                quaternion = base_button_quaternion
            else:
                position = translation + group_rotation @ base_position
                quaternion = quaternion_multiply(group_quaternion, base_button_quaternion)
            state = pose_message(position, quaternion)
            state.name = button["model"]
            requests.append(state)
            if not self.current_is_negative:
                self.current_button_poses[button["model"]] = (position, quaternion)

        self.state_futures = []
        for state in requests:
            request = SetEntityState.Request()
            request.state = state
            self.state_futures.append(self.state_client.call_async(request))
        self.phase = "moving"

    def _capture_sample(self):
        message = self.latest_image
        try:
            transform = self.tf_buffer.lookup_transform(
                message.header.frame_id,
                self.world_frame,
                Time.from_msg(message.header.stamp),
                timeout=Duration(seconds=0.25),
            )
        except TransformException as error:
            self.get_logger().warning(f"Waiting for camera transform: {error}")
            return False

        image = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
        if bool(self.get_parameter("augment_images").value):
            image = self._augment(image)

        image_id = self.sample_index + 1
        file_name = f"sim_{image_id:06d}.jpg"
        image_path = self.output_dir / "images" / file_name
        written = cv2.imwrite(
            str(image_path), image, [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality]
        )
        if not written:
            self._fail(f"Failed to write {image_path}")
            return False

        height, width = image.shape[:2]
        sample_annotations = self._project_annotations(transform, width, height)
        expected_annotations = 0 if self.current_is_negative else len(BUTTONS)
        if len(sample_annotations) != expected_annotations:
            image_path.unlink(missing_ok=True)
            self.get_logger().warning(
                "Random pose moved one or more buttons outside the camera; resampling"
            )
            return False

        if self.current_is_negative:
            self.negative_count += 1

        self.images.append({
            "id": image_id,
            "file_name": file_name,
            "width": width,
            "height": height,
        })
        for annotation in sample_annotations:
            annotation["id"] = self.annotation_id
            annotation["image_id"] = image_id
            self.annotation_id += 1
            self.annotations.append(annotation)

        if image_id == 1 or image_id % 10 == 0 or image_id == self.num_samples:
            self.get_logger().info(f"Captured {image_id}/{self.num_samples}")
        return True

    def _project_annotations(self, transform, width, height):
        if self.current_is_negative:
            return []
        tf_translation = np.array([
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z,
        ])
        tf_quaternion = np.array([
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z,
            transform.transform.rotation.w,
        ])
        tf_rotation = rotation_from_quaternion(tf_quaternion)
        fx, fy = self.camera_info.k[0], self.camera_info.k[4]
        cx, cy = self.camera_info.k[2], self.camera_info.k[5]

        annotations = []
        angles = np.linspace(0.0, 2.0 * math.pi, 48, endpoint=False)
        for button in BUTTONS:
            position, quaternion = self.current_button_poses[button["model"]]
            local_points = np.stack([
                button["radius"] * np.cos(angles),
                button["radius"] * np.sin(angles),
                np.full_like(angles, -BUTTON_HALF_DEPTH),
            ], axis=1)
            world_points = (
                rotation_from_quaternion(quaternion) @ local_points.T
            ).T + position
            camera_points = (tf_rotation @ world_points.T).T + tf_translation
            valid = camera_points[:, 2] > 0.05
            if np.count_nonzero(valid) < 4:
                continue
            camera_points = camera_points[valid]
            u = fx * camera_points[:, 0] / camera_points[:, 2] + cx
            v = fy * camera_points[:, 1] / camera_points[:, 2] + cy
            x0 = float(np.clip(np.min(u), 0.0, width - 1.0))
            y0 = float(np.clip(np.min(v), 0.0, height - 1.0))
            x1 = float(np.clip(np.max(u), 0.0, width - 1.0))
            y1 = float(np.clip(np.max(v), 0.0, height - 1.0))
            box_width, box_height = x1 - x0, y1 - y0
            if box_width < 4.0 or box_height < 4.0:
                continue
            annotations.append({
                "category_id": button["category_id"],
                "bbox": [x0, y0, box_width, box_height],
                "area": box_width * box_height,
                "iscrowd": 0,
            })
        return annotations

    def _augment(self, image):
        contrast = self.rng.uniform(
            float(self.get_parameter("contrast_min").value),
            float(self.get_parameter("contrast_max").value),
        )
        brightness = self.rng.uniform(
            -float(self.get_parameter("brightness_abs").value),
            float(self.get_parameter("brightness_abs").value),
        )
        result = cv2.convertScaleAbs(image, alpha=float(contrast), beta=float(brightness))

        hsv = cv2.cvtColor(result, cv2.COLOR_BGR2HSV).astype(np.float32)
        saturation = self.rng.uniform(
            float(self.get_parameter("saturation_min").value),
            float(self.get_parameter("saturation_max").value),
        )
        hsv[:, :, 1] = np.clip(hsv[:, :, 1] * saturation, 0, 255)
        result = cv2.cvtColor(hsv.astype(np.uint8), cv2.COLOR_HSV2BGR)

        sigma = self.rng.uniform(0.0, float(self.get_parameter("noise_sigma_max").value))
        if sigma > 0.1:
            noise = self.rng.normal(0.0, sigma, result.shape).astype(np.float32)
            result = np.clip(result.astype(np.float32) + noise, 0, 255).astype(np.uint8)
        if self.rng.random() < float(self.get_parameter("blur_probability").value):
            result = cv2.GaussianBlur(result, (3, 3), 0.0)
        return result

    def _finish(self):
        categories = [
            {"id": button["category_id"], "name": button["name"]}
            for button in BUTTONS
        ]
        full = {
            "info": {
                "description": "Piper Gazebo colored elevator button dataset",
                "random_seed": self.random_seed,
            },
            "images": self.images,
            "annotations": self.annotations,
            "categories": categories,
        }
        self._write_json(self.output_dir / "annotations" / "instances_all.json", full)

        ids = np.array([image["id"] for image in self.images], dtype=np.int64)
        self.rng.shuffle(ids)
        train_end = int(round(0.70 * len(ids)))
        val_end = train_end + int(round(0.15 * len(ids)))
        splits = {
            "train": set(ids[:train_end].tolist()),
            "val": set(ids[train_end:val_end].tolist()),
            "test": set(ids[val_end:].tolist()),
        }
        for name, selected_ids in splits.items():
            subset = {
                "info": dict(full["info"], split=name),
                "images": [image for image in self.images if image["id"] in selected_ids],
                "annotations": [
                    annotation
                    for annotation in self.annotations
                    if annotation["image_id"] in selected_ids
                ],
                "categories": categories,
            }
            self._write_json(
                self.output_dir / "annotations" / f"instances_{name}.json", subset
            )

        manifest = {
            "samples": len(self.images),
            "positive_samples": len(self.images) - self.negative_count,
            "negative_samples": self.negative_count,
            "annotations": len(self.annotations),
            "splits": {name: len(ids) for name, ids in splits.items()},
            "image_topic": str(self.get_parameter("image_topic").value),
            "camera_frame": self.latest_image.header.frame_id,
            "random_seed": self.random_seed,
        }
        self._write_json(self.output_dir / "manifest.json", manifest)
        self.get_logger().info(
            f"Dataset complete: {len(self.images)} images, {len(self.annotations)} boxes"
        )
        self.completed = True
        self.timer.cancel()
        self._schedule_shutdown()

    @staticmethod
    def _write_json(path, value):
        temporary = path.with_suffix(path.suffix + ".tmp")
        temporary.write_text(json.dumps(value, indent=2), encoding="utf-8")
        temporary.replace(path)

    def _fail(self, message):
        self.get_logger().error(message)
        self.completed = False
        self.timer.cancel()
        self._schedule_shutdown()

    @staticmethod
    def _schedule_shutdown():
        # Shutting the global context down from the executor callback can block
        # while TF subscriptions are being serviced.  Request it from a short-
        # lived helper thread so main() can leave rclpy.spin() cleanly.
        def stop_context():
            time.sleep(0.10)
            if rclpy.ok():
                rclpy.shutdown()

        threading.Thread(target=stop_context, daemon=True).start()


def main(args=None):
    rclpy.init(args=args)
    node = None
    exit_code = 1
    try:
        node = ButtonDatasetGenerator()
        rclpy.spin(node)
        exit_code = 0 if node.completed else 1
    except Exception as error:  # Keep launch output concise and actionable.
        if node is not None:
            node.get_logger().error(str(error))
        else:
            print(f"dataset generator failed: {error}", file=sys.stderr)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(exit_code)


if __name__ == "__main__":
    main()
