import numpy as np
import pytest

from groundtruth.geometry import wrap_angle
from groundtruth.solver import solve_robot_pose
from synthetic import make_cameras, make_setup, observe_robot, robot_pose_in_area

SETUP = make_setup()
CAMERAS = make_cameras(SETUP)
ROBOT_TAGS = SETUP.robot_tag_poses()

POSES = [(1.5, 0.0, 0.0), (2.0, 0.5, 30.0), (2.5, -0.5, -60.0), (3.0, 0.0, 90.0), (2.0, 0.0, 180.0), (2.5, 0.3, -135.0)]


def error(solution, truth):
    xyt = truth.planar_xyt()
    return np.hypot(solution.x - xyt[0], solution.y - xyt[1]), abs(wrap_angle(solution.theta - xyt[2]))


@pytest.mark.parametrize("distance,lateral,heading", POSES)
def test_two_cameras_no_noise_recovers_the_pose(distance, lateral, heading):
    truth = robot_pose_in_area(SETUP, distance, lateral, heading)
    obs = observe_robot(CAMERAS, SETUP, truth)
    assert len({o.camera.name for o in obs}) == 2
    sol = solve_robot_pose(obs, ROBOT_TAGS, SETUP.tag_size)  # no initial guess: uses single-tag PnP
    pos, ang = error(sol, truth)
    assert sol.accepted, sol.reason
    assert pos < 1e-4
    assert np.degrees(ang) < 0.01


@pytest.mark.parametrize("camera", ["A", "B"])
def test_one_camera_is_enough_without_noise(camera):
    truth = robot_pose_in_area(SETUP, 2.0, 0.0, 45.0)
    cams = [c for c in CAMERAS if c.name == camera]
    sol = solve_robot_pose(observe_robot(cams, SETUP, truth), ROBOT_TAGS, SETUP.tag_size)
    pos, ang = error(sol, truth)
    assert pos < 1e-4 and np.degrees(ang) < 0.01


def test_noise_is_inside_the_reported_uncertainty():
    """With 0.3 px corner noise, the errors follow the reported covariance."""
    rng = np.random.default_rng(7)
    truth = robot_pose_in_area(SETUP, 2.5, 0.2, 20.0)
    mahalanobis, position_errors, sigmas = [], [], []
    for _ in range(200):
        sol = solve_robot_pose(observe_robot(CAMERAS, SETUP, truth, 0.3, rng), ROBOT_TAGS, SETUP.tag_size,
                               initial=truth.planar_xyt())
        e = sol.xyt - truth.planar_xyt()
        e[2] = wrap_angle(e[2])
        mahalanobis.append(e @ np.linalg.solve(sol.cov, e))
        position_errors.append(np.hypot(e[0], e[1]))
        sigmas.append(sol.sigma_position_m)
    mahalanobis = np.array(mahalanobis)
    # chi-square with 3 degrees of freedom: mean 3, 99th percentile 11.34.
    assert 2.0 < mahalanobis.mean() < 4.5
    assert np.mean(mahalanobis < 11.34) > 0.95
    # The design expects millimeter-level noise per frame at 0.3 px.
    assert np.sqrt(np.mean(np.square(position_errors))) < 0.003
    assert np.median(sigmas) < 0.003


def test_motion_between_camera_frames_is_corrected_by_delta():
    """Camera B captures 8 ms after camera A while the robot drives 0.75 m/s sideways."""
    truth = robot_pose_in_area(SETUP, 2.0, 0.0, 0.0)
    delta = (0.0, 0.75 * 0.008, 0.0)
    obs = observe_robot(CAMERAS, SETUP, truth, delta_by_camera={"B": delta})
    with_delta = solve_robot_pose(obs, ROBOT_TAGS, SETUP.tag_size, initial=truth.planar_xyt())
    for o in obs:
        o.delta = None
    without = solve_robot_pose(obs, ROBOT_TAGS, SETUP.tag_size, initial=truth.planar_xyt())
    assert error(with_delta, truth)[0] < 1e-4
    assert error(without, truth)[0] > 1e-3


def test_one_tag_in_one_camera_is_not_accepted():
    truth = robot_pose_in_area(SETUP, 2.0, 0.0, 0.0)
    obs = observe_robot(CAMERAS, SETUP, truth)[:1]
    sol = solve_robot_pose(obs, ROBOT_TAGS, SETUP.tag_size)
    assert not sol.accepted
    assert "one tag" in sol.reason


def test_bad_corners_are_not_accepted():
    rng = np.random.default_rng(3)
    truth = robot_pose_in_area(SETUP, 2.0, 0.0, 0.0)
    obs = observe_robot(CAMERAS, SETUP, truth, 5.0, rng)
    sol = solve_robot_pose(obs, ROBOT_TAGS, SETUP.tag_size, initial=truth.planar_xyt())
    assert not sol.accepted
    assert "RMS" in sol.reason


def test_no_robot_tags():
    assert not solve_robot_pose([], ROBOT_TAGS, SETUP.tag_size).accepted
