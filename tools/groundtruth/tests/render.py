"""Renders synthetic camera images of the ChArUco board through a known lens, for tests."""

from __future__ import annotations

from functools import lru_cache

import cv2
import numpy as np
from scipy.spatial.transform import Rotation

from groundtruth.camera import Lens
from groundtruth.charuco import BoardSpec

PX_PER_SQUARE = 40


@lru_cache(maxsize=4)
def board_image(spec: BoardSpec) -> np.ndarray:
    return spec.board().generateImage((spec.squares_x * PX_PER_SQUARE, spec.squares_y * PX_PER_SQUARE),
                                      marginSize=0, borderBits=1)


_rays: dict[tuple, np.ndarray] = {}


def pixel_rays(lens: Lens) -> np.ndarray:
    key = (lens.fx, lens.fy, lens.cx, lens.cy, tuple(lens.dist), lens.width, lens.height)
    if key not in _rays:
        u, v = np.meshgrid(np.arange(lens.width), np.arange(lens.height))
        n = lens.undistort(np.column_stack([u.ravel(), v.ravel()]).astype(float))
        _rays[key] = np.column_stack([n, np.ones(len(n))])
    return _rays[key]


def board_pose(lens: Lens, spec: BoardSpec, target_px, distance: float, tilt_x_deg: float = 0.0,
               tilt_y_deg: float = 0.0, roll_deg: float = 0.0) -> tuple[np.ndarray, np.ndarray]:
    """camera_from_board (R, t) with the board center at ``target_px`` and ``distance`` squares away."""
    R = Rotation.from_euler("zyx", [roll_deg, tilt_y_deg, tilt_x_deg], degrees=True).as_matrix()
    ray = np.append(lens.undistort(np.array([target_px], dtype=float))[0], 1.0)
    center = np.array([spec.squares_x / 2, spec.squares_y / 2, 0.0])
    return R, distance * ray / np.linalg.norm(ray) - R @ center


def render(lens: Lens, spec: BoardSpec, R: np.ndarray, t: np.ndarray, blur: float = 0.7, noise: float = 1.5,
           rng=None) -> np.ndarray:
    d = pixel_rays(lens)
    dd = d @ R  # rows: R^T d
    a = R.T @ t
    s = a[2] / dd[:, 2]
    x = s * dd[:, 0] - a[0]
    y = s * dd[:, 1] - a[1]
    valid = s > 0
    mapx = np.where(valid, x * PX_PER_SQUARE - 0.5, -10).reshape(lens.height, lens.width).astype(np.float32)
    mapy = np.where(valid, y * PX_PER_SQUARE - 0.5, -10).reshape(lens.height, lens.width).astype(np.float32)
    img = cv2.remap(board_image(spec), mapx, mapy, cv2.INTER_AREA if False else cv2.INTER_LINEAR,
                    borderMode=cv2.BORDER_CONSTANT, borderValue=170)
    if blur:
        img = cv2.GaussianBlur(img, (0, 0), blur)
    if noise:
        rng = rng or np.random.default_rng(0)
        img = np.clip(img + rng.normal(0, noise, img.shape), 0, 255)
    return img.astype(np.uint8)


def true_corners(lens: Lens, spec: BoardSpec, R, t, ids) -> np.ndarray:
    pc = spec.object_points(ids) @ R.T + t
    return lens.distort(pc[:, :2] / pc[:, 2:3])
