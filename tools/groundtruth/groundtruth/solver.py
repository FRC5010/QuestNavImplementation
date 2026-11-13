"""Robot pose (x, y, theta) from robot-tag corners seen by one or more fixed cameras.

The robot is on the floor, so each robot tag has a known height and the only unknowns are the
planar pose. This makes the distance along a camera's view come from the tag's vertical position
in the image, which is much more accurate than the tag's size.
"""

from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np
from scipy.optimize import least_squares

from .camera import Camera
from .geometry import TAG_FROM_MARKER, Pose3, marker_corners, tag_corners, wrap_angle


@dataclass
class Observation:
    """One robot tag in one camera frame.

    ``corners_px``: shape (4, 2), in ArUco order (top-left, top-right, bottom-right, bottom-left).
    ``delta``: optional (dx, dy, dtheta), the robot motion in the robot frame from the solve time
    to the time of this observation. Use it to combine frames that were not captured at the same
    time.
    """

    camera: Camera
    tag_id: int
    corners_px: np.ndarray
    delta: tuple[float, float, float] | None = None


@dataclass
class Solution:
    x: float
    y: float
    theta: float
    cov: np.ndarray
    rms_px: float
    n_observations: int
    n_tags: int
    n_cameras: int
    accepted: bool
    reason: str = ""

    @property
    def xyt(self) -> np.ndarray:
        return np.array([self.x, self.y, self.theta])

    @property
    def sigma_position_m(self) -> float:
        """Standard deviation along the worst direction of the position."""
        return float(np.sqrt(max(np.linalg.eigvalsh(self.cov[:2, :2]).max(), 0.0)))

    @property
    def sigma_theta_rad(self) -> float:
        return float(np.sqrt(max(self.cov[2, 2], 0.0)))

    @staticmethod
    def none(reason: str) -> "Solution":
        nan = float("nan")
        return Solution(nan, nan, nan, np.full((3, 3), np.nan), nan, 0, 0, 0, False, reason)


@dataclass
class Limits:
    max_rms_px: float = 1.0
    max_sigma_m: float = 0.015
    noise_floor_px: float = 0.2


def solve_robot_pose(
    observations: list[Observation],
    robot_tags: dict[int, Pose3],
    tag_size: float,
    initial: np.ndarray | None = None,
    limits: Limits = Limits(),
) -> Solution:
    """Least-squares robot pose. ``robot_tags`` maps tag ID to robot_from_tag."""
    obs = [o for o in observations if o.tag_id in robot_tags]
    if not obs:
        return Solution.none("no robot tags")

    local = tag_corners(tag_size)
    points_robot = [robot_tags[o.tag_id].apply(local) for o in obs]
    measured = [o.camera.lens.undistort(o.corners_px) for o in obs]
    deltas = [Pose3.planar(*o.delta) if o.delta is not None else None for o in obs]
    camera_from_field = [o.camera.pose.inverse() for o in obs]
    fx = [o.camera.lens.fx for o in obs]

    def residuals(p: np.ndarray) -> np.ndarray:
        field_from_robot = Pose3.planar(p[0], p[1], p[2])
        out = []
        for i in range(len(obs)):
            pose = field_from_robot @ deltas[i] if deltas[i] is not None else field_from_robot
            pc = camera_from_field[i].apply(pose.apply(points_robot[i]))
            z = np.maximum(pc[:, 2], 1e-3)
            out.append(((pc[:, :2] / z[:, None] - measured[i]) * fx[i]).ravel())
        return np.concatenate(out)

    starts = [np.asarray(initial, dtype=float)] if initial is not None else _initial_guesses(obs, robot_tags, tag_size)
    if not starts:
        return Solution.none("no initial guess")
    best = None
    for x0 in starts:
        result = least_squares(residuals, x0, method="lm")
        if best is None or result.cost < best.cost:
            best = result

    r = best.fun
    n_corners = r.size / 2
    rms = float(np.sqrt(np.sum(r * r) / n_corners))
    dof = max(r.size - 3, 1)
    variance = max(np.sum(r * r) / dof, limits.noise_floor_px ** 2)
    cov = variance * np.linalg.pinv(best.jac.T @ best.jac)

    x, y, theta = best.x
    n_tags = len({o.tag_id for o in obs})
    n_cameras = len({o.camera.name for o in obs})
    solution = Solution(float(x), float(y), wrap_angle(float(theta)), cov, rms, len(obs), n_tags, n_cameras, True)

    if len(obs) < 2:
        solution.accepted, solution.reason = False, "only one tag in one camera"
    elif rms > limits.max_rms_px:
        solution.accepted, solution.reason = False, f"RMS {rms:.2f} px is more than {limits.max_rms_px} px"
    elif solution.sigma_position_m > limits.max_sigma_m:
        solution.accepted, solution.reason = False, (
            f"position sigma {solution.sigma_position_m * 100:.1f} cm is more than {limits.max_sigma_m * 100:.1f} cm")
    return solution


def _initial_guesses(obs: list[Observation], robot_tags: dict[int, Pose3], tag_size: float) -> list[np.ndarray]:
    """Planar poses from single-tag PnP on the two largest tags (both IPPE solutions of each)."""

    def area(o: Observation) -> float:
        c = o.corners_px
        return 0.5 * abs(np.dot(c[:, 0], np.roll(c[:, 1], 1)) - np.dot(c[:, 1], np.roll(c[:, 0], 1)))

    guesses = []
    for o in sorted(obs, key=area, reverse=True)[:2]:
        lens = o.camera.lens
        n, rvecs, tvecs, _ = cv2.solvePnPGeneric(
            marker_corners(tag_size), np.asarray(o.corners_px, dtype=np.float64), lens.K, lens.dist,
            flags=cv2.SOLVEPNP_IPPE_SQUARE)
        for k in range(n):
            camera_from_marker = Pose3.from_rotvec(rvecs[k].ravel(), tvecs[k].ravel())
            camera_from_tag = camera_from_marker @ TAG_FROM_MARKER.inverse()
            field_from_robot = o.camera.pose @ camera_from_tag @ robot_tags[o.tag_id].inverse()
            if o.delta is not None:
                field_from_robot = field_from_robot @ Pose3.planar(*o.delta).inverse()
            guesses.append(field_from_robot.planar_xyt())
    return guesses
