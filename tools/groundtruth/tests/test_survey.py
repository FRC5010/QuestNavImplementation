import numpy as np

from groundtruth.config import FloorTag
from groundtruth.survey import survey_camera
from synthetic import make_cameras, make_setup, observe_static


def pose_error(found, truth):
    position = np.linalg.norm(found.t - truth.t)
    angle = np.degrees(np.linalg.norm((found.inverse() @ truth).rotvec()))
    return position, angle


def test_survey_recovers_the_camera_pose():
    setup = make_setup()
    rng = np.random.default_rng(11)
    for cam in make_cameras(setup):
        obs = observe_static(cam, setup.static_tag_poses(), setup.tag_size, frames=300, noise_px=0.5, rng=rng)
        result = survey_camera(cam.lens, obs, setup.static_tag_poses(), setup.tag_size)
        position, angle = pose_error(result.pose, cam.pose)
        assert position < 0.002, f"camera {cam.name}: {position * 1000:.2f} mm"
        assert angle < 0.01, f"camera {cam.name}: {angle:.4f} deg"
        assert result.rms_px < 0.1
        assert not result.warnings, result.warnings


def test_a_misplaced_floor_tag_shows_in_the_report():
    setup = make_setup()
    cam = make_cameras(setup)[0]
    rng = np.random.default_rng(12)
    # The real tag 51 is 3 cm farther from the wall than setup.json says.
    real = dict(setup.static_tag_poses())
    t = setup.floor_tags[51]
    real[51] = FloorTag(51, t.distance + 0.03, t.lateral, t.top_yaw_deg).field_from_tag(setup.layout[setup.reference_tag])
    obs = observe_static(cam, real, setup.tag_size, frames=100, noise_px=0.3, rng=rng)

    result = survey_camera(cam.lens, obs, setup.static_tag_poses(), setup.tag_size)

    assert max(result.tag_rms_px, key=result.tag_rms_px.get) == 51
    assert any("tag 51" in w for w in result.warnings)


def test_a_moved_camera_shows_as_spread():
    setup = make_setup()
    cam = make_cameras(setup)[0]
    rng = np.random.default_rng(13)
    obs = observe_static(cam, setup.static_tag_poses(), setup.tag_size, frames=50, noise_px=0.3, rng=rng)
    for o in obs[len(obs) // 2:]:
        o.corners_px = o.corners_px + np.array([6.0, 0.0])  # the tripod was bumped half way
    result = survey_camera(cam.lens, obs, setup.static_tag_poses(), setup.tag_size)
    assert any("did the camera move" in w for w in result.warnings)
