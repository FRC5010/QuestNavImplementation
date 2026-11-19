"""ChArUco board model and a detector that works with very small markers in 4K video.

The team's board is a ThriftyBot ChArUco board: 22 x 15 squares, OpenCV ``DICT_4X4_250``
markers that are 70% of a square. Its markers are small, so at more than about 1 m from a 4K
camera the default ArUco settings cannot read them. :class:`BoardDetector` first tries the fast
default detector; if the markers are small, it crops the board, enlarges the crop and detects
again with settings tuned for small markers.
"""

from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np

A = cv2.aruco


@dataclass(frozen=True)
class BoardSpec:
    squares_x: int = 22
    squares_y: int = 15
    dictionary: str = "DICT_4X4_250"
    marker_ratio: float = 0.70
    square_m: float = 1.0  # Only positions need the real size; lens calibration does not.

    def board(self) -> "cv2.aruco.CharucoBoard":
        d = A.getPredefinedDictionary(getattr(A, self.dictionary))
        return A.CharucoBoard((self.squares_x, self.squares_y), self.square_m,
                              self.square_m * self.marker_ratio, d)

    def object_points(self, ids: np.ndarray) -> np.ndarray:
        """Board-frame (x, y, 0) of the given inner-corner IDs, shape (N, 3)."""
        return self.board().getChessboardCorners()[np.asarray(ids, dtype=int)].astype(np.float32)

    @property
    def corner_count(self) -> int:
        return (self.squares_x - 1) * (self.squares_y - 1)


THRIFTYBOT = BoardSpec()


@dataclass
class BoardView:
    """The inner corners of the board found in one frame."""

    time: float
    corners: np.ndarray  # (N, 2) float32, full-resolution pixels
    ids: np.ndarray  # (N,) int


def _small_marker_params() -> "cv2.aruco.DetectorParameters":
    p = A.DetectorParameters()
    p.minMarkerPerimeterRate = 0.005
    p.adaptiveThreshWinSizeMin = 3
    p.adaptiveThreshWinSizeMax = 53
    p.adaptiveThreshWinSizeStep = 4
    p.perspectiveRemovePixelPerCell = 8
    p.errorCorrectionRate = 0.8
    return p


class BoardDetector:
    def __init__(self, spec: BoardSpec = THRIFTYBOT, upscale: int = 3, min_corners: int = 12,
                 small_marker_px: float = 40.0):
        self.spec = spec
        self.board = spec.board()
        d = A.getPredefinedDictionary(getattr(A, spec.dictionary))
        self.fast = A.ArucoDetector(d, A.DetectorParameters())
        self.small = A.ArucoDetector(d, _small_marker_params())
        self.charuco = A.CharucoDetector(self.board)
        self.upscale = upscale
        self.min_corners = min_corners
        self.small_marker_px = small_marker_px

    def detect(self, image: np.ndarray, time: float = 0.0) -> BoardView | None:
        gray = image if image.ndim == 2 else cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.fast.detectMarkers(gray)
        if ids is None or len(ids) < 6:
            corners, ids, _ = self.small.detectMarkers(gray)
        if ids is None or len(ids) < 6:
            return None

        sides = [np.mean(np.linalg.norm(c.reshape(4, 2) - np.roll(c.reshape(4, 2), 1, axis=0), axis=1))
                 for c in corners]
        side = float(np.median(sides))
        if side >= self.small_marker_px:
            cc, ci, _, _ = self.charuco.detectBoard(gray, markerCorners=corners, markerIds=ids)
            if ci is None or len(ci) < self.min_corners:
                return None
            return BoardView(time, cc.reshape(-1, 2).astype(np.float32), ci.ravel().astype(int))

        # Small markers: crop around the board (robust to a few false markers), enlarge, detect again.
        pts = np.concatenate([c.reshape(4, 2) for c in corners])
        lo, hi = np.percentile(pts, 5, axis=0), np.percentile(pts, 95, axis=0)
        margin = 0.35 * (hi - lo) + 4 * side
        h, w = gray.shape
        x0, y0 = (np.maximum(lo - margin, 0)).astype(int)
        x1, y1 = (np.minimum(hi + margin, [w, h])).astype(int)
        crop = cv2.resize(gray[y0:y1, x0:x1], None, fx=self.upscale, fy=self.upscale,
                          interpolation=cv2.INTER_CUBIC)
        corners, ids, _ = self.small.detectMarkers(crop)
        if ids is None or len(ids) < 6:
            return None
        cc, ci, _, _ = self.charuco.detectBoard(crop, markerCorners=corners, markerIds=ids)
        if ci is None or len(ci) < self.min_corners:
            return None
        # Pixel centers: crop pixel (u, v) at scale s covers full-res (x0 + (u + 0.5) / s - 0.5).
        full = (cc.reshape(-1, 2) + 0.5) / self.upscale - 0.5 + [x0, y0]
        return BoardView(time, full.astype(np.float32), ci.ravel().astype(int))
