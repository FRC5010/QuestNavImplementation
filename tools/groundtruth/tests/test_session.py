"""End-to-end session analysis on a synthetic session (no video): two cameras with their own clocks,
a robot that does a sync wiggle and drives a loop, and a Quest track with a known error."""

import numpy as np
import pytest

from groundtruth.detect import TagDetections
from groundtruth.geometry import Pose3
from groundtruth.robotlog import RobotLog, Snapshot
from groundtruth.session import combine, compare, snapshot_table, survey_from_detections, sync_track, truth_track, write_report
from groundtruth.camera import Camera
from synthetic import make_cameras, make_setup, observe_robot, robot_pose_in_area, visible_corners

SETUP = make_setup()
CAMERAS = make_cameras(SETUP)
CLOCKS = {"A": (1000.0, 1 + 30e-6), "B": (1003.2, 1 - 20e-6)}  # robot_time = a + b * video_time
QUEST_OFFSET = np.array([0.008, -0.006])  # m, field frame
QUEST_LATE_S = 0.03  # Quest timestamps are 30 ms later than the real time of the pose
WIGGLE = (1010.0, 1017.0)


def robot_path(t: np.ndarray) -> np.ndarray:
    """(x, y, theta) in the field frame at robot time t: still, a wiggle, then a slow loop."""
    out = []
    for ti in np.atleast_1d(t):
        s = ti - 1000.0
        if s < 10.5:
            d, l, h = 2.5, 0.0, 0.0
        elif s < 16.5:  # sync wiggle: +-30 deg in place
            d, l, h = 2.5, 0.0, 30.0 * np.sin(2 * np.pi * (s - 10.5) / 3.0)
        elif s < 18:
            d, l, h = 2.5, 0.0, 0.0
        else:  # loop of 0.5 m radius, heading turning slowly
            a = 2 * np.pi * (s - 18) / 20.0
            d, l, h = 2.5 + 0.5 * np.sin(a), 0.5 * (1 - np.cos(a)) - 0.25, 20 * np.sin(a)
        out.append(robot_pose_in_area(SETUP, d, l, h).planar_xyt())
    return np.array(out)


def make_detections(cam: Camera, rng) -> TagDetections:
    a, b = CLOCKS[cam.name]
    times, ids, corners = [], [], []
    static = SETUP.static_tag_poses()
    for vt in np.arange(0.0, 50.0, 1 / 30):
        rt = a + b * vt
        if rt > 1045:
            break
        x, y, th = robot_path(rt)[0]
        pose = Pose3.planar(x, y, th)
        for o in observe_robot([cam], SETUP, pose, 0.3, rng):
            times.append(vt), ids.append(o.tag_id), corners.append(o.corners_px)
        for tag_id, tp in static.items():
            px = visible_corners(cam, tp, SETUP.tag_size)
            if px is not None:
                times.append(vt), ids.append(tag_id), corners.append(px + rng.normal(0, 0.3, px.shape))
    return TagDetections(np.array(times), np.array(ids), np.array(corners, np.float32), (3840, 2160), [cam.name])


def make_log(rng) -> RobotLog:
    tq = np.arange(1000.0, 1046.0, 1 / 90)
    P = robot_path(tq)
    quest = np.column_stack([tq + QUEST_LATE_S, P[:, 0] + QUEST_OFFSET[0] + rng.normal(0, 0.002, len(tq)),
                             P[:, 1] + QUEST_OFFSET[1] + rng.normal(0, 0.002, len(tq)), np.degrees(P[:, 2])])
    tg = np.arange(995.0, 1050.0, 1 / 250)
    G = robot_path(tg)
    gyro = np.column_stack([tg, (np.degrees(G[:, 2]) + 40 + 180) % 360 - 180])
    fused = np.column_stack([tg, G[:, 0], G[:, 1], np.degrees(G[:, 2])])
    snaps = [Snapshot(1005.0, "TurnSweep", 1, "still", tuple(robot_path(1005.0)[0, :2] + QUEST_OFFSET) +
                      (float(np.degrees(robot_path(1005.0)[0, 2])),), (0, 0, 0))]
    return RobotLog("synthetic", quest, gyro, fused, [WIGGLE], [(995.0, 1050.0)], [(0.0, "2. Drift Loop")], snaps)


@pytest.fixture(scope="module")
def session(tmp_path_factory):
    rng = np.random.default_rng(21)
    log = make_log(rng)
    tracks, surveys = [], {}
    for cam in CAMERAS:
        det = make_detections(cam, rng)
        survey = survey_from_detections(SETUP, cam.lens, det)
        surveys[cam.name] = (survey, cam)
        track = truth_track(cam.name, SETUP, Camera(cam.name, cam.lens, survey.pose), det)
        clock = sync_track(track, log)
        tracks.append((track, clock))
    truth, agreement = combine([t for t, _ in tracks])
    comp = compare(truth, log)
    out = tmp_path_factory.mktemp("report")
    from groundtruth.session import CameraResult
    results = [CameraResult(c.name, Camera(c.name, c.lens, surveys[c.name][0].pose), surveys[c.name][0], t, k)
               for c, (t, k) in zip(CAMERAS, tracks)]
    snaps = snapshot_table(truth, log, comp)
    write_report(out, SETUP, log, results, truth, agreement, comp, snaps)
    return dict(log=log, tracks=tracks, surveys=surveys, truth=truth, agreement=agreement, comp=comp, out=out, snaps=snaps)


def test_surveys_find_the_cameras(session):
    for name, (survey, cam) in session["surveys"].items():
        assert np.linalg.norm(survey.pose.t - cam.pose.t) < 0.005, name


def test_clocks_are_recovered(session):
    for track, clock in session["tracks"]:
        a, b = CLOCKS[track.name]
        for vt in (5.0, 25.0, 45.0):
            assert abs(clock.to_robot(vt) - (a + b * vt)) < 0.003, track.name


def test_truth_follows_the_robot(session):
    truth = session["truth"]
    true = robot_path(truth.t)
    err = np.hypot(truth.x - true[:, 0], truth.y - true[:, 1])
    # Most frames show one robot tag per camera (about 2 mm sigma); frames with 2+ tags are under 1 mm.
    assert np.sqrt(np.mean(err ** 2)) < 0.005
    assert np.median(err) < 0.002
    assert session["agreement"]["pairs"] > 100
    assert session["agreement"]["position_median_m"] < 0.006


def test_quest_errors_are_measured(session):
    s = session["comp"].summary
    rms = s["absolute_quest"]["all"]["rms_m"]
    assert abs(rms - np.linalg.norm(QUEST_OFFSET)) < 0.004, rms
    assert np.allclose(s["absolute_quest"]["all"]["mean_offset_m"], QUEST_OFFSET, atol=0.003)
    # Quest noise (2 mm per axis) plus truth noise.
    assert s["aligned_quest"]["all"]["rms_m"] < 0.007
    assert abs(s["latency_ms"] - QUEST_LATE_S * 1000) <= 10, s["latency_ms"]
    assert s["heading_fix_deg"] == 0


def test_snapshot_and_report_files(session):
    snaps = session["snaps"]
    assert len(snaps) == 1 and abs(snaps[0]["abs_error_m"] - np.linalg.norm(QUEST_OFFSET)) < 0.004
    for f in ("report.md", "samples.csv", "snapshots.csv", "summary.json", "path.png", "error.png"):
        assert (session["out"] / f).exists(), f
    assert "QuestNav accuracy" in (session["out"] / "report.md").read_text()
