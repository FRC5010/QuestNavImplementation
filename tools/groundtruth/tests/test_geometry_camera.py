import cv2
import numpy as np

from groundtruth.geometry import TAG_FROM_MARKER, Pose3, look_at, marker_corners, tag_corners
from groundtruth.config import load_field_layout
from synthetic import GOPRO, make_cameras, make_setup, visible_corners


def test_distort_matches_opencv():
    rng = np.random.default_rng(1)
    points = np.column_stack([rng.uniform(-1.5, 1.5, 50), rng.uniform(-1, 1, 50), rng.uniform(2, 6, 50)])
    expected, _ = cv2.projectPoints(points, np.zeros(3), np.zeros(3), GOPRO.K, GOPRO.dist)
    ours = GOPRO.distort(points[:, :2] / points[:, 2:])
    assert np.abs(ours - expected.reshape(-1, 2)).max() < 1e-6


def test_undistort_round_trip():
    rng = np.random.default_rng(2)
    normalized = np.column_stack([rng.uniform(-0.8, 0.8, 100), rng.uniform(-0.45, 0.45, 100)])
    back = GOPRO.undistort(GOPRO.distort(normalized))
    assert np.abs(back - normalized).max() < 1e-9


def test_tag_corner_order_matches_the_marker_frame():
    assert np.allclose(TAG_FROM_MARKER.apply(marker_corners(0.2)), tag_corners(0.2))
    assert np.isclose(np.linalg.det(TAG_FROM_MARKER.R), 1.0)


def test_corners_appear_in_aruco_order_to_a_viewer():
    # A camera in front of tag 25 sees top-left at the top left of the image, then clockwise.
    tag = load_field_layout()[25]
    cam_pos = tag.apply(np.array([2.0, 0.0, 0.0]))
    from groundtruth.camera import Camera
    cam = Camera("front", GOPRO, look_at(cam_pos, tag.t))
    px, _ = cam.project(tag.apply(tag_corners(0.1651)))
    tl, tr, br, bl = px
    assert tl[0] < tr[0] and bl[0] < br[0]  # left is left
    assert tl[1] < bl[1] and tr[1] < br[1]  # top is up (smaller y)


def test_look_at_is_a_rotation_and_points_forward():
    pose = look_at([0, 0, 2], [3, 1, 0])
    assert np.isclose(np.linalg.det(pose.R), 1.0)
    forward = pose.R[:, 2]
    assert np.allclose(forward, np.array([3, 1, -2]) / np.linalg.norm([3, 1, -2]))


def test_pose_compose_and_inverse():
    a = Pose3.from_rotvec([0.1, -0.2, 0.3], [1, 2, 3])
    b = Pose3.from_rotvec([-0.3, 0.1, 0.2], [0.5, -1, 2])
    p = np.array([0.3, 0.2, -0.1])
    assert np.allclose((a @ b).apply(p), a.apply(b.apply(p)))
    assert np.allclose((a @ a.inverse()).apply(p), p)


def test_synthetic_scene_sees_the_tags_it_needs():
    setup = make_setup()
    static = setup.static_tag_poses()
    for cam in make_cameras(setup):
        seen = [i for i, pose in static.items() if visible_corners(cam, pose, setup.tag_size) is not None]
        assert len(seen) >= 4, f"camera {cam.name} sees only {seen}"
