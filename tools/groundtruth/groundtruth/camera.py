"""Lens model (OpenCV pinhole with 5 distortion coefficients) and a fixed camera."""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path

import cv2
import numpy as np

from .geometry import Pose3


@dataclass
class Lens:
    fx: float
    fy: float
    cx: float
    cy: float
    dist: np.ndarray = field(default_factory=lambda: np.zeros(5))  # k1, k2, p1, p2, k3
    width: int = 0
    height: int = 0

    @property
    def K(self) -> np.ndarray:
        return np.array([[self.fx, 0.0, self.cx], [0.0, self.fy, self.cy], [0.0, 0.0, 1.0]])

    def distort(self, normalized: np.ndarray) -> np.ndarray:
        """Normalized image coordinates (x/z, y/z), shape (N, 2), to pixels."""
        k1, k2, p1, p2, k3 = self.dist
        x, y = normalized[:, 0], normalized[:, 1]
        r2 = x * x + y * y
        radial = 1 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2
        xd = x * radial + 2 * p1 * x * y + p2 * (r2 + 2 * x * x)
        yd = y * radial + p1 * (r2 + 2 * y * y) + 2 * p2 * x * y
        return np.column_stack([self.fx * xd + self.cx, self.fy * yd + self.cy])

    def undistort(self, pixels: np.ndarray) -> np.ndarray:
        """Pixels, shape (N, 2), to normalized image coordinates with the distortion removed."""
        target = np.asarray(pixels, dtype=np.float64).reshape(-1, 2)
        # Start from OpenCV's fixed-point estimate, then refine with Newton steps on
        # distort(x) = pixel, so the result is exact to numerical precision.
        x = cv2.undistortPoints(target.reshape(-1, 1, 2), self.K, self.dist).reshape(-1, 2)
        eps = 1e-7
        for _ in range(10):
            f = self.distort(x) - target
            if np.abs(f).max() < 1e-10:
                break
            jx = (self.distort(x + [eps, 0.0]) - self.distort(x - [eps, 0.0])) / (2 * eps)
            jy = (self.distort(x + [0.0, eps]) - self.distort(x - [0.0, eps])) / (2 * eps)
            det = jx[:, 0] * jy[:, 1] - jy[:, 0] * jx[:, 1]
            dx = (jy[:, 1] * f[:, 0] - jy[:, 0] * f[:, 1]) / det
            dy = (-jx[:, 1] * f[:, 0] + jx[:, 0] * f[:, 1]) / det
            x = x - np.column_stack([dx, dy])
        return x

    def to_dict(self) -> dict:
        return {
            "width": self.width,
            "height": self.height,
            "camera_matrix": self.K.tolist(),
            "dist_coeffs": [float(v) for v in self.dist],
        }

    @staticmethod
    def from_dict(d: dict) -> "Lens":
        k = np.asarray(d["camera_matrix"], dtype=float)
        dist = np.zeros(5)
        coeffs = np.asarray(d["dist_coeffs"], dtype=float).ravel()
        dist[: min(5, coeffs.size)] = coeffs[:5]
        return Lens(k[0, 0], k[1, 1], k[0, 2], k[1, 2], dist, int(d.get("width", 0)), int(d.get("height", 0)))

    @staticmethod
    def load(path: str | Path) -> "Lens":
        return Lens.from_dict(json.loads(Path(path).read_text()))


@dataclass
class Camera:
    """A fixed camera. ``pose`` is field_from_camera (OpenCV camera axes)."""

    name: str
    lens: Lens
    pose: Pose3

    def to_camera(self, points_field: np.ndarray) -> np.ndarray:
        return self.pose.inverse().apply(points_field)

    def project(self, points_field: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """Pixels for field points, and a mask of the points that are in front of the camera."""
        pc = self.to_camera(points_field)
        in_front = pc[:, 2] > 1e-3
        z = np.where(in_front, pc[:, 2], 1e-3)
        return self.lens.distort(pc[:, :2] / z[:, None]), in_front
