"""Small, ROS-independent helpers for RGB-D button localization."""
from collections import Counter

import numpy as np


def robust_depth(depth_image, u, v, radius, min_depth, max_depth, scale=1.0):
    """Return the median valid depth around a pixel, in metres."""
    height, width = depth_image.shape[:2]
    x0, x1 = max(0, u - radius), min(width, u + radius + 1)
    y0, y1 = max(0, v - radius), min(height, v + radius + 1)
    values = np.asarray(depth_image[y0:y1, x0:x1], dtype=np.float32) * scale
    valid = values[np.isfinite(values) & (values >= min_depth) & (values <= max_depth)]
    return None if valid.size < 5 else float(np.median(valid))


def project_pixel(u, v, depth, camera_matrix):
    """Project an RGB/depth pixel into the REP-103 optical camera frame."""
    fx, fy, cx, cy = camera_matrix[0], camera_matrix[4], camera_matrix[2], camera_matrix[5]
    if fx <= 0 or fy <= 0 or depth <= 0:
        raise ValueError("Invalid camera intrinsics or depth")
    return np.array([(u - cx) * depth / fx, (v - cy) * depth / fy, depth], dtype=float)


def transform_point(point, translation, quaternion):
    """Apply a geometry_msgs-style quaternion transform to a 3-vector."""
    x, y, z, w = quaternion
    norm = np.linalg.norm(quaternion)
    if norm < 1e-9:
        raise ValueError("Zero-length transform quaternion")
    x, y, z, w = np.asarray(quaternion, dtype=float) / norm
    rotation = np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
        [2*(x*y + z*w), 1 - 2*(x*x + z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x*x + y*y)],
    ])
    return rotation @ np.asarray(point, dtype=float) + np.asarray(translation, dtype=float)


def physical_size_ratio(box, focal_length, depth, diameter):
    """Compare a roughly circular image box with its expected pinhole size."""
    x1, y1, x2, y2 = box
    observed = (max(0.0, x2 - x1) + max(0.0, y2 - y1)) / 2.0
    expected = focal_length * diameter / depth
    return observed / expected if expected > 0 else 0.0


def stable_candidate(samples, required, tolerance):
    """Return median (point, confidence, pixel, depth) for a stable same-color window."""
    valid = [sample for sample in samples if sample is not None]
    if len(valid) < required:
        return None
    color = Counter(sample[0] for sample in valid).most_common(1)[0][0]
    same = [sample for sample in valid if sample[0] == color]
    if len(same) < required:
        return None
    points = np.asarray([sample[1] for sample in same[-required:]], dtype=float)
    median = np.median(points, axis=0)
    if np.max(np.linalg.norm(points - median, axis=1)) > tolerance:
        return None
    confidence = float(np.median([sample[2] for sample in same[-required:]]))
    pixel = np.median([sample[3] for sample in same[-required:]], axis=0)
    depth = float(np.median([sample[4] for sample in same[-required:]]))
    return color, median, confidence, pixel, depth
