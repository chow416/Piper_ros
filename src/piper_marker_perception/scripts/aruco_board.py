#!/usr/bin/env python3
"""ArUco grid board: geometry, detection, and pose estimation.

Two conventions live here. Everything else in the package inherits them, so
they are stated once, in this docstring, rather than re-derived at each call.

The board frame
---------------
    origin   the board's geometric centre
    +X       right, looking at the front of the board
    +Y       up
    +Z       out of the plane, towards the viewer

+Z is the direction the board faces, so *into* the board is -Z. `detect()`
returns the pose of exactly this frame in the camera optical frame.

The 180 degree camera roll -- read before changing anything
-----------------------------------------------------------
The sensor in this camera is mounted upside down, so raw frames are rotated by
180 degrees. Nothing here undoes that, on purpose: the rotation is *absorbed*
into the hand-eye calibration. Both the intrinsics and the hand-eye solve are
done on the same raw frames, so the two errors cancel in exactly the way that
leaves the final `base_link <- optical` transform correct.

That gives a consistency rule spanning three places:

    capture_session.py   stores raw frames
    the calibrator       solves intrinsics and hand-eye on raw frames
    this module          detects on raw frames

Rotate in any one of them and not the others and the answer is wrong by a
rotation that still looks plausible -- the 180 degrees is a fixed rotation about
the optical axis, so it silently folds into the board pose. If you ever want the
rotated convention, change it in all three at once.

Why the object points are built here instead of by cv2.aruco.GridBoard
---------------------------------------------------------------------
`GridBoard` has its own opinion about where the board origin and axes sit, and
that opinion has moved between OpenCV versions. `solvePnP` can only be as
correct as the points it is handed, so they are computed explicitly below. The
cost is arithmetic on the marker pitch; the benefit is that a pose means the
same thing whatever OpenCV produced it.
"""

import os

import cv2
import numpy as np

#: Corner positions within one marker, in units of marker side length, expressed
#: in the marker's own frame (origin at its centre, +Z out of its plane). This
#: is the order `cv2.aruco.detectMarkers` reports corners in: clockwise,
#: starting from the top-left seen from the front.
CORNER_OFFSETS = np.array([
    [-0.5, +0.5, 0.0],
    [+0.5, +0.5, 0.0],
    [+0.5, -0.5, 0.0],
    [-0.5, -0.5, 0.0],
])

#: Detector settings for a board seen from roughly 0.2-0.8 m at 1080p.
#:
#: Subpixel corner refinement is the single most valuable entry here: without it
#: corners land on whole pixels and pose noise is about an order of magnitude
#: worse, which is the difference between hitting a button and missing it.
DETECTOR_DEFAULTS = {
    "adaptiveThreshWinSizeMin": 5,
    "adaptiveThreshWinSizeMax": 53,
    "adaptiveThreshWinSizeStep": 4,
    "adaptiveThreshConstant": 7.0,
    "cornerRefinementMethod": "CORNER_REFINE_SUBPIX",
    "cornerRefinementWinSize": 5,
    "cornerRefinementMaxIterations": 50,
    "cornerRefinementMinAccuracy": 0.01,
    # A marker that fills more of the frame is fine; the default upper bound is
    # tuned for close-up work and rejects legitimate detections here.
    "maxMarkerPerimeterRate": 4.0,
    "minMarkerPerimeterRate": 0.02,
    "polygonalApproxAccuracyRate": 0.03,
}


class BoardSpec:
    """The physical board: layout, sizes, and the frame they imply."""

    def __init__(self, dictionary="DICT_4X4_50", markers_x=3, markers_y=3,
                 marker_length=0.030, marker_separation=0.010,
                 first_marker_id=0, accepted_ids=None):
        if markers_x < 1 or markers_y < 1:
            raise ValueError("markers_x and markers_y must be at least 1")
        if marker_length <= 0:
            raise ValueError("marker_length must be positive")
        if marker_separation < 0:
            raise ValueError("marker_separation cannot be negative")
        self.dictionary_name = str(dictionary)
        self.markers_x = int(markers_x)
        self.markers_y = int(markers_y)
        self.marker_length = float(marker_length)
        self.marker_separation = float(marker_separation)
        self.first_marker_id = int(first_marker_id)
        self.accepted_ids = (None if accepted_ids is None
                             else sorted({int(i) for i in accepted_ids}))

    # -- geometry ---------------------------------------------------------

    @property
    def pitch(self):
        """Centre-to-centre distance between neighbouring markers."""
        return self.marker_length + self.marker_separation

    @property
    def width(self):
        return (self.markers_x * self.marker_length
                + (self.markers_x - 1) * self.marker_separation)

    @property
    def height(self):
        return (self.markers_y * self.marker_length
                + (self.markers_y - 1) * self.marker_separation)

    def marker_id(self, column, row):
        """ID of the marker at (column, row). Row 0 is the *top* row."""
        return self.first_marker_id + row * self.markers_x + column

    def marker_centre(self, column, row):
        """Marker centre in the board frame, metres."""
        return np.array([
            (column - (self.markers_x - 1) / 2.0) * self.pitch,
            ((self.markers_y - 1) / 2.0 - row) * self.pitch,
            0.0,
        ])

    def is_accepted(self, marker_id):
        return self.accepted_ids is None or marker_id in self.accepted_ids

    def object_points_by_id(self):
        """{marker id: 4x3 array} of corners in the board frame, metres."""
        points = {}
        for row in range(self.markers_y):
            for column in range(self.markers_x):
                marker_id = self.marker_id(column, row)
                if not self.is_accepted(marker_id):
                    continue
                points[marker_id] = (self.marker_centre(column, row)
                                     + CORNER_OFFSETS * self.marker_length)
        return points

    def marker_centres_by_id(self):
        """{marker id: (x, y)} centres, for drawing and for sanity checks."""
        return {self.marker_id(c, r): self.marker_centre(c, r)[:2]
                for r in range(self.markers_y)
                for c in range(self.markers_x) if self.is_accepted(self.marker_id(c, r))}

    # -- OpenCV objects ---------------------------------------------------

    def dictionary(self):
        """The `cv2.aruco` dictionary this spec names."""
        if not hasattr(cv2.aruco, self.dictionary_name):
            known = sorted(n for n in dir(cv2.aruco) if n.startswith("DICT_"))
            raise ValueError(f"unknown ArUco dictionary {self.dictionary_name!r}. "
                             f"Available: {', '.join(known)}")
        return cv2.aruco.getPredefinedDictionary(
            getattr(cv2.aruco, self.dictionary_name))

    def make_detector(self, **overrides):
        """Return (dictionary, parameters) ready for `cv2.aruco.detectMarkers`."""
        unknown = set(overrides) - set(DETECTOR_DEFAULTS)
        if unknown:
            raise ValueError(f"unknown detector parameter(s): {sorted(unknown)}")
        settings = dict(DETECTOR_DEFAULTS)
        settings.update(overrides)

        parameters = cv2.aruco.DetectorParameters_create()
        for key, value in settings.items():
            if key == "cornerRefinementMethod":
                # Given as a name so the file stays readable and so a missing
                # constant fails here rather than silently disabling refinement.
                if not hasattr(cv2.aruco, value):
                    raise ValueError(f"unknown cornerRefinementMethod {value!r}")
                value = getattr(cv2.aruco, value)
            setattr(parameters, key, value)
        return self.dictionary(), parameters

    # -- (de)serialisation -------------------------------------------------

    @classmethod
    def from_dict(cls, data):
        board = data.get("board", data) if isinstance(data, dict) else {}
        return cls(
            dictionary=board.get("dictionary", "DICT_4X4_50"),
            markers_x=board.get("markers_x", 3),
            markers_y=board.get("markers_y", 3),
            marker_length=board.get("marker_length", 0.030),
            marker_separation=board.get("marker_separation", 0.010),
            first_marker_id=board.get("first_marker_id", 0),
            accepted_ids=board.get("accepted_ids"),
        )

    @classmethod
    def from_yaml(cls, path):
        import yaml
        with open(path, encoding="utf-8") as handle:
            return cls.from_dict(yaml.safe_load(handle) or {})

    def __repr__(self):
        return (f"BoardSpec({self.dictionary_name}, "
                f"{self.markers_x}x{self.markers_y}, "
                f"L={self.marker_length * 1000:.1f}mm, "
                f"S={self.marker_separation * 1000:.1f}mm, "
                f"{self.width * 1000:.1f}x{self.height * 1000:.1f}mm)")


class BoardDetection:
    """One successful detection.

    `rvec`/`tvec` are `T_camera_optical <- board`, in the board frame defined in
    the module docstring. `marker_ids` are the markers that contributed, and
    `reproj_rms_px` is the fit quality -- the number to gate on before believing
    any of the rest.
    """

    def __init__(self, rvec, tvec, reproj_rms_px, marker_ids,
                 object_points, image_points, corners, ids, inliers):
        self.rvec = np.asarray(rvec, dtype=float).reshape(3, 1)
        self.tvec = np.asarray(tvec, dtype=float).reshape(3, 1)
        self.reproj_rms_px = float(reproj_rms_px)
        self.marker_ids = list(marker_ids)
        self.object_points = object_points
        self.image_points = image_points
        #: Kept exactly as `detectMarkers` returned them: the drawing helper
        #: asserts on CV_32FC2, and a tidy-up cast to float64 breaks it.
        self.corners = corners
        self.ids = ids
        self.inliers = inliers

    @property
    def rotation(self):
        """3x3 rotation, board -> camera optical."""
        return cv2.Rodrigues(self.rvec)[0]

    @property
    def matrix(self):
        """4x4 `T_camera_optical <- board`."""
        out = np.eye(4)
        out[:3, :3] = self.rotation
        out[:3, 3] = self.tvec.ravel()
        return out

    @property
    def distance(self):
        """Board centre distance from the camera, metres."""
        return float(np.linalg.norm(self.tvec))

    def __repr__(self):
        t = self.tvec.ravel()
        return (f"BoardDetection({len(self.marker_ids)} markers, "
                f"rms={self.reproj_rms_px:.2f}px, "
                f"t=({t[0]:+.3f}, {t[1]:+.3f}, {t[2]:+.3f}) m)")


def detect(image, camera_matrix, dist_coeffs, spec, detector=None,
           use_ransac=False, min_markers=1, refine=True):
    """Find the board in `image` and solve its pose.

    Returns a `BoardDetection`, or None when the board is not found, too few of
    its markers are visible, or the solve fails. Callers gate on
    `reproj_rms_px` afterwards; this function only reports, it does not judge.

    `image` may be colour or greyscale. It must be a raw frame -- see the module
    docstring on the 180 degree roll.

    A board mounted face-down is *not* caught by geometry, and this was measured
    rather than assumed. Seen from behind, every marker is mirrored, mirrored
    patterns are not valid codes, and eight of the nine stop decoding -- but the
    survivor is enough for `solvePnP` to fit a pose that looks front-facing,
    points the right way, and has a low reprojection error. It is also wrong by
    a reflection, and no orientation test on the result can tell.

    The signal that does work is the marker count: 9 collapse to 1. That is why
    `min_markers` should be set close to the number of markers on the board, and
    why it is the gate worth trusting.
    """
    dictionary, parameters = detector or spec.make_detector()

    gray = image
    if image.ndim == 3:
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    corners, ids, _ = cv2.aruco.detectMarkers(gray, dictionary,
                                              parameters=parameters)
    if ids is None or len(ids) == 0:
        return None

    by_id = spec.object_points_by_id()
    object_points, image_points, kept, kept_corners = [], [], [], []
    for marker_corners, marker_id in zip(corners, ids.flatten()):
        marker_id = int(marker_id)
        if marker_id not in by_id:
            continue                      # not part of this board; ignore it
        object_points.append(by_id[marker_id])
        image_points.append(np.asarray(marker_corners, dtype=float).reshape(4, 2))
        kept.append(marker_id)
        kept_corners.append(marker_corners)

    if len(kept) < max(1, min_markers):
        return None

    object_points = np.vstack(object_points)
    image_points = np.vstack(image_points)

    if use_ransac:
        ok, rvec, tvec, inliers = cv2.solvePnPRansac(
            object_points, image_points, camera_matrix, dist_coeffs,
            iterationsCount=200, reprojectionError=4.0, confidence=0.99,
            flags=cv2.SOLVEPNP_ITERATIVE)
        if not ok:
            return None
        inliers = np.asarray(inliers).ravel()
    else:
        ok, rvec, tvec = cv2.solvePnP(
            object_points, image_points, camera_matrix, dist_coeffs,
            flags=cv2.SOLVEPNP_ITERATIVE)
        if not ok:
            return None
        inliers = np.arange(len(object_points))

    if refine and len(inliers) >= 4:
        try:
            rvec, tvec = cv2.solvePnPRefineLM(
                object_points[inliers], image_points[inliers],
                np.asarray(camera_matrix, dtype=float),
                np.asarray(dist_coeffs, dtype=float), rvec, tvec)
        except cv2.error:
            pass                          # keep the unrefined solve

    projected, _ = cv2.projectPoints(object_points[inliers], rvec, tvec,
                                     camera_matrix, dist_coeffs)
    residual = projected.reshape(-1, 2) - image_points[inliers]
    rms = float(np.sqrt(np.mean(np.sum(residual ** 2, axis=1))))

    return BoardDetection(rvec, tvec, rms, kept, object_points, image_points,
                          kept_corners, kept, inliers)


def annotate(image, detection, camera_matrix, dist_coeffs, axis_length=None,
             thickness=2):
    """Return a copy of `image` with corners, frame axes, and markers drawn.

    The axes are the board frame, so the red arrow points along board +X, green
    along +Y and blue along +Z -- i.e. blue should point at the camera when the
    board is facing it. That is the quickest visual check that the board has not
    been mounted face-down.
    """
    out = image.copy()
    if detection is None:
        return out
    if out.ndim == 2:
        out = cv2.cvtColor(out, cv2.COLOR_GRAY2BGR)

    cv2.aruco.drawDetectedMarkers(out, detection.corners,
                                  np.array(detection.marker_ids).reshape(-1, 1))

    if axis_length is None:
        # Scale the axes to the board so they stay visible at any distance.
        axis_length = max(0.02, 0.6 * detection.distance / 1.0)
    cv2.drawFrameAxes(out, camera_matrix, dist_coeffs,
                      detection.rvec, detection.tvec, axis_length, thickness)

    text = (f"markers {len(detection.marker_ids)}  "
            f"rms {detection.reproj_rms_px:.2f}px  "
            f"d {detection.distance * 100:.1f}cm")
    cv2.putText(out, text, (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                (0, 0, 0), 4, cv2.LINE_AA)
    cv2.putText(out, text, (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                (255, 255, 255), 1, cv2.LINE_AA)
    return out


# -- camera intrinsics ---------------------------------------------------

class CameraIntrinsics:
    """The pinhole model: a 3x3 matrix and a distortion vector."""

    def __init__(self, camera_matrix, dist_coeffs, image_size=None,
                 source=None):
        self.camera_matrix = np.asarray(camera_matrix, dtype=float).reshape(3, 3)
        self.dist_coeffs = np.asarray(dist_coeffs, dtype=float).ravel()
        self.image_size = None if image_size is None else tuple(image_size)
        self.source = source

    @property
    def fx(self):
        return float(self.camera_matrix[0, 0])

    @property
    def fy(self):
        return float(self.camera_matrix[1, 1])

    @property
    def cx(self):
        return float(self.camera_matrix[0, 2])

    @property
    def cy(self):
        return float(self.camera_matrix[1, 2])

    @classmethod
    def from_yaml(cls, path):
        """Read either the calibration output or a ROS CameraInfo YAML.

        Both shapes show up in practice -- `calibrateCamera` writes
        camera_matrix/data, a ROS camera node writes camera_matrix/rows/cols --
        so both are accepted and the caller gets one object either way.
        """
        import yaml
        with open(path, encoding="utf-8") as handle:
            data = yaml.safe_load(handle)

        if data is None:
            raise ValueError(f"{path} is empty")

        # ROS CameraInfo: camera_matrix is {rows, cols, data}. Our own output
        # and OpenCV's FileStorage dump use the same shape, so this one branch
        # covers both; a bare list is accepted too.
        node = data.get("camera_matrix", data.get("K"))
        if node is None:
            raise ValueError(f"{path}: no camera_matrix / K")
        if isinstance(node, dict):
            matrix = node.get("data", node)
            if isinstance(matrix, list) and len(matrix) == 9:
                camera_matrix = np.array(matrix, dtype=float).reshape(3, 3)
            elif isinstance(matrix, dict):        # rows/cols/data nesting
                camera_matrix = np.array(matrix["data"], dtype=float).reshape(3, 3)
            else:
                raise ValueError(f"{path}: cannot read camera_matrix")
        else:
            camera_matrix = np.array(node, dtype=float).reshape(3, 3)

        distortion = data.get("distortion_coefficients", data.get("D", []))
        if isinstance(distortion, dict):
            distortion = distortion.get("data", [])
        dist_coeffs = np.array(distortion or [], dtype=float).ravel()

        size = data.get("image_width"), data.get("image_height")
        if size[0] is None:
            size = None

        return cls(camera_matrix, dist_coeffs, size, source=path)

    @classmethod
    def from_file_or_directory(cls, path):
        """Accept a YAML file, or a directory to search for the newest one."""
        if path is None:
            raise ValueError("no calibration given")
        if os.path.isfile(path):
            return cls.from_yaml(path)
        if not os.path.isdir(path):
            raise ValueError(f"{path} is neither a file nor a directory")
        candidates = sorted(
            os.path.join(path, name) for name in os.listdir(path)
            if name.endswith((".yaml", ".yml")))
        if not candidates:
            raise ValueError(f"no .yaml/.yml in {path}")
        return cls.from_yaml(candidates[-1])

    def __repr__(self):
        return (f"CameraIntrinsics(fx={self.fx:.1f}, fy={self.fy:.1f}, "
                f"cx={self.cx:.1f}, cy={self.cy:.1f}, "
                f"{len(self.dist_coeffs)} distortion terms)")
