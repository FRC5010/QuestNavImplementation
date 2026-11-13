"""Rigid transforms and AprilTag corner geometry.

Conventions:

- Field frame: the robot code's frame (2026 WPILib, origin at the blue alliance corner, z up).
- Robot frame: +x forward, +y left, +z up, origin at the rotation center on the floor.
- Tag frame (WPILib): +x out of the tag face, +y to the viewer's right, +z up (the top edge of
  the tag). The tag center is the origin.
- Camera frame (OpenCV): +x right in the image, +y down in the image, +z forward along the view.

A Pose3 "A_from_B" maps points in frame B to frame A: ``p_A = R @ p_B + t``.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from scipy.spatial.transform import Rotation


def rot_z(theta: float) -> np.ndarray:
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def wrap_angle(theta: float) -> float:
    """Wraps an angle in radians to [-pi, pi)."""
    return (theta + np.pi) % (2 * np.pi) - np.pi


@dataclass(frozen=True)
class Pose3:
    R: np.ndarray
    t: np.ndarray

    @staticmethod
    def identity() -> "Pose3":
        return Pose3(np.eye(3), np.zeros(3))

    @staticmethod
    def planar(x: float, y: float, theta: float, z: float = 0.0) -> "Pose3":
        return Pose3(rot_z(theta), np.array([x, y, z], dtype=float))

    @staticmethod
    def from_quaternion(w: float, x: float, y: float, z: float, t) -> "Pose3":
        return Pose3(Rotation.from_quat([x, y, z, w]).as_matrix(), np.asarray(t, dtype=float))

    @staticmethod
    def from_rotvec(rotvec, t) -> "Pose3":
        return Pose3(Rotation.from_rotvec(np.asarray(rotvec, dtype=float)).as_matrix(), np.asarray(t, dtype=float))

    def rotvec(self) -> np.ndarray:
        return Rotation.from_matrix(self.R).as_rotvec()

    def compose(self, other: "Pose3") -> "Pose3":
        return Pose3(self.R @ other.R, self.R @ other.t + self.t)

    __matmul__ = compose

    def inverse(self) -> "Pose3":
        rt = self.R.T
        return Pose3(rt, -rt @ self.t)

    def apply(self, points: np.ndarray) -> np.ndarray:
        """Transforms points, shape (N, 3) or (3,)."""
        return np.asarray(points) @ self.R.T + self.t

    def yaw(self) -> float:
        return float(np.arctan2(self.R[1, 0], self.R[0, 0]))

    def planar_xyt(self) -> np.ndarray:
        """(x, y, yaw) of this pose projected onto the floor."""
        return np.array([self.t[0], self.t[1], self.yaw()])


def look_at(position, target, up=(0.0, 0.0, 1.0)) -> Pose3:
    """field_from_camera for a camera at ``position`` that looks at ``target`` (OpenCV axes)."""
    position = np.asarray(position, dtype=float)
    forward = np.asarray(target, dtype=float) - position
    forward /= np.linalg.norm(forward)
    right = np.cross(forward, up)
    right /= np.linalg.norm(right)
    down = np.cross(forward, right)
    return Pose3(np.column_stack([right, down, forward]), position)


def tag_corners(size: float) -> np.ndarray:
    """Corners of a tag in its own frame, in OpenCV ArUco order.

    The order is top-left, top-right, bottom-right, bottom-left, as a viewer in front of the tag
    sees it. ``size`` is the edge length of the black square (0.1651 m for FRC tags).
    """
    h = size / 2
    return np.array([[0.0, -h, h], [0.0, h, h], [0.0, h, -h], [0.0, -h, -h]])


# Maps OpenCV's square-marker frame (x right, y up, z toward the viewer; corners at z = 0) to the
# tag frame. Used with cv2.SOLVEPNP_IPPE_SQUARE.
TAG_FROM_MARKER = Pose3(np.array([[0.0, 0.0, 1.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]]), np.zeros(3))


def marker_corners(size: float) -> np.ndarray:
    """The same corners as :func:`tag_corners`, in OpenCV's square-marker frame."""
    h = size / 2
    return np.array([[-h, h, 0.0], [h, h, 0.0], [h, -h, 0.0], [-h, -h, 0.0]])
