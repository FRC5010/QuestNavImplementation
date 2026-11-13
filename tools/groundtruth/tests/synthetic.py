"""A synthetic test area: the fake hub (tags 25, 26, 24), three floor tags, a robot with four
corner tags, and two GoPro-like cameras 90 degrees apart, 2.3 m high."""

from __future__ import annotations

import numpy as np

from groundtruth.camera import Camera, Lens
from groundtruth.config import FloorTag, RobotTag, Setup
from groundtruth.geometry import Pose3, look_at, tag_corners
from groundtruth.solver import Observation
from groundtruth.survey import StaticObservation

# 4K, Linear lens: focal length about 2100 px. Small distortion, as a calibrated Linear mode leaves.
GOPRO = Lens(2100.0, 2100.0, 1920.0, 1080.0, np.array([-0.03, 0.01, 0.0005, -0.0003, 0.0]), 3840, 2160)


def make_setup() -> Setup:
    return Setup(
        robot_tags={
            40: RobotTag(40, 0.40, 0.40, 0.30, 45.0),
            41: RobotTag(41, 0.40, -0.40, 0.30, -45.0),
            42: RobotTag(42, -0.40, -0.40, 0.30, -135.0),
            43: RobotTag(43, -0.40, 0.40, 0.30, 135.0),
        },
        floor_tags={
            50: FloorTag(50, 1.7, -0.8, 0.0),
            51: FloorTag(51, 1.7, 0.8, 90.0),
            52: FloorTag(52, 3.3, 0.0, 180.0),
        },
    )


def tag_frame_point(setup: Setup, distance: float, lateral: float, z: float) -> np.ndarray:
    """A field point given in the reference tag's frame (distance out, lateral right), at height z."""
    ref = setup.layout[setup.reference_tag]
    p = ref.R @ np.array([distance, lateral, 0.0]) + ref.t
    p[2] = z
    return p


def make_cameras(setup: Setup) -> list[Camera]:
    target = tag_frame_point(setup, 2.0, 0.0, 0.5)
    a = look_at(tag_frame_point(setup, 4.6, -2.1, 2.3), target)
    b = look_at(tag_frame_point(setup, 4.6, 2.1, 2.3), target)
    return [Camera("A", GOPRO, a), Camera("B", GOPRO, b)]


def robot_pose_in_area(setup: Setup, distance: float, lateral: float, heading_deg: float) -> Pose3:
    """Robot pose from the reference tag frame; heading 0 = front faces the tag."""
    ref = setup.layout[setup.reference_tag]
    p = tag_frame_point(setup, distance, lateral, 0.0)
    yaw = ref.yaw() + np.pi + np.radians(heading_deg)
    return Pose3.planar(p[0], p[1], yaw)


def visible_corners(camera: Camera, field_from_tag: Pose3, size: float, max_angle_deg: float = 75.0):
    """Pixel corners of a tag if the camera sees its face, else None."""
    corners = field_from_tag.apply(tag_corners(size))
    to_camera = camera.pose.t - field_from_tag.t
    facing = np.dot(field_from_tag.R[:, 0], to_camera / np.linalg.norm(to_camera))
    if facing < np.cos(np.radians(max_angle_deg)):
        return None
    px, in_front = camera.project(corners)
    lens = camera.lens
    inside = (px[:, 0] > 10) & (px[:, 0] < lens.width - 10) & (px[:, 1] > 10) & (px[:, 1] < lens.height - 10)
    if not (in_front.all() and inside.all()):
        return None
    return px


def observe_robot(cameras, setup: Setup, field_from_robot: Pose3, noise_px: float = 0.0, rng=None,
                  delta_by_camera=None) -> list[Observation]:
    """Robot tag observations. ``delta_by_camera``: camera name -> (dx, dy, dtheta) of the robot
    from the solve time to that camera's capture time (the robot is drawn at the moved pose)."""
    out = []
    for cam in cameras:
        delta = None if delta_by_camera is None else delta_by_camera.get(cam.name)
        pose = field_from_robot @ Pose3.planar(*delta) if delta is not None else field_from_robot
        for tag_id, robot_from_tag in setup.robot_tag_poses().items():
            px = visible_corners(cam, pose @ robot_from_tag, setup.tag_size)
            if px is None:
                continue
            if noise_px:
                px = px + rng.normal(0.0, noise_px, px.shape)
            out.append(Observation(cam, tag_id, px, delta))
    return out


def observe_static(camera: Camera, static_tags: dict[int, Pose3], size: float, frames: int,
                   noise_px: float, rng) -> list[StaticObservation]:
    out = []
    for _ in range(frames):
        for tag_id, pose in static_tags.items():
            px = visible_corners(camera, pose, size)
            if px is not None:
                out.append(StaticObservation(tag_id, px + rng.normal(0.0, noise_px, px.shape)))
    return out
