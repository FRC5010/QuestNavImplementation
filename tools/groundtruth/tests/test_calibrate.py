import numpy as np
import pytest

from groundtruth.calibrate import calibrate, collect_views, select_views
from groundtruth.camera import Lens
from groundtruth.charuco import THRIFTYBOT, BoardDetector, BoardView
from render import board_pose, render, true_corners

# A GoPro-like Linear lens at 1280 x 720 (about 85 degrees wide), with some distortion.
LENS = Lens(700.0, 700.0, 645.0, 352.0, np.array([-0.06, 0.03, 0.0008, -0.0005, 0.0]), 1280, 720)
DETECTOR = BoardDetector(THRIFTYBOT)


@pytest.mark.parametrize("distance,max_median_px", [(14.0, 0.08), (30.0, 0.3)])
def test_detector_finds_the_board_with_large_and_tiny_markers(distance, max_median_px):
    # At 30 squares the markers are about 16 px wide: the crop-and-enlarge path is needed, and
    # corners are less precise (random, not an offset).
    R, t = board_pose(LENS, THRIFTYBOT, (640, 360), distance, tilt_x_deg=15, tilt_y_deg=-20)
    view = DETECTOR.detect(render(LENS, THRIFTYBOT, R, t))
    assert view is not None
    assert len(view.ids) > 0.8 * THRIFTYBOT.corner_count
    error = view.corners - true_corners(LENS, THRIFTYBOT, R, t, view.ids)
    assert np.abs(error.mean(axis=0)).max() < 0.03, f"corner offset {error.mean(axis=0)} px"
    median = np.median(np.linalg.norm(error, axis=1))
    assert median < max_median_px, f"median corner error {median:.3f} px"


def calibration_frames(targets, tilts, distances, rng):
    frames = []
    for k, (target, tilt, distance) in enumerate(zip(targets, tilts, distances)):
        R, t = board_pose(LENS, THRIFTYBOT, target, distance, *tilt)
        frames.append((float(k), render(LENS, THRIFTYBOT, R, t, rng=rng)))
    return frames


def test_calibration_recovers_the_lens():
    rng = np.random.default_rng(3)
    xs, ys = [240, 640, 1040], [170, 360, 550]
    targets = [(x, y) for x in xs for y in ys] * 3
    tilts = [(rng.uniform(-40, 40), rng.uniform(-40, 40), rng.uniform(-20, 20)) for _ in targets]
    distances = rng.uniform(13, 20, len(targets))
    views = collect_views(calibration_frames(targets, tilts, distances, rng), DETECTOR)
    assert len(views) >= 20

    result = calibrate(select_views(views, THRIFTYBOT, (1280, 720)), (1280, 720), THRIFTYBOT)

    print(result.report())
    assert abs(result.lens.fx - LENS.fx) / LENS.fx < 0.005
    assert abs(result.lens.fy - LENS.fy) / LENS.fy < 0.005
    assert abs(result.lens.cx - LENS.cx) < 3 and abs(result.lens.cy - LENS.cy) < 3
    assert result.rms_px < 0.3
    # The recovered lens maps points like the true lens across the image.
    grid = np.array([[x, y] for x in np.linspace(0, 1279, 9) for y in np.linspace(0, 719, 6)], dtype=float)
    assert np.abs(result.lens.distort(LENS.undistort(grid)) - grid).max() < 1.0
    assert result.passed, result.report()

    # The saved lens file loads back as the same lens.
    import json, tempfile
    from groundtruth.calibrate import save
    with tempfile.TemporaryDirectory() as d:
        path = f"{d}/lens.json"
        save(result, path, source="synthetic")
        data = json.load(open(path))
        assert data["passed"] is True and data["checks"]["rms"]["ok"] is True
        back = Lens.load(path)
        assert np.allclose(back.K, result.lens.K) and np.allclose(back.dist, result.lens.dist)


def test_board_only_in_the_middle_fails_the_checks():
    """Like run 1: the board stays near the image center and nearly face-on."""
    rng = np.random.default_rng(4)
    targets = [(640 + rng.uniform(-60, 60), 360 + rng.uniform(-40, 40)) for _ in range(20)]
    tilts = [(rng.uniform(-10, 10), rng.uniform(-10, 10), 0.0) for _ in targets]
    distances = rng.uniform(30, 34, len(targets))
    views = collect_views(calibration_frames(targets, tilts, distances, rng), DETECTOR)

    result = calibrate(views, (1280, 720), THRIFTYBOT)

    assert not result.passed
    assert not result.checks["coverage"][0]
    assert not result.checks["edges"][0]
    assert not result.checks["tilt"][0]


def test_fast_moving_board_is_dropped():
    class Fixed:
        def __init__(self, views):
            self.views = iter(views)

        def detect(self, image, t):
            return next(self.views)

    ids = np.arange(20)
    still = np.random.default_rng(0).uniform(100, 500, (20, 2)).astype(np.float32)
    views = [BoardView(0.0, still, ids), BoardView(0.1, still + 1.0, ids), BoardView(0.2, still + 60.0, ids)]
    kept = collect_views([(v.time, None) for v in views], Fixed(views), max_speed_px_s=120)
    assert [v.time for v in kept] == [0.0, 0.1]


def test_chapter_files_are_one_timeline(tmp_path):
    import cv2
    from groundtruth.calibrate import chapter_frames
    paths = []
    for c in range(2):
        path = str(tmp_path / f"chapter{c}.avi")
        out = cv2.VideoWriter(path, cv2.VideoWriter_fourcc(*"MJPG"), 10.0, (64, 48))
        for k in range(20):  # 2 s per chapter
            out.write(np.full((48, 64, 3), c * 100 + k, np.uint8))
        out.release()
        paths.append(path)
    times = [t for t, _ in chapter_frames(paths, step=5)]
    assert np.allclose(times, [0.0, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 3.5], atol=1e-6)
    window = [t for t, _ in chapter_frames(paths, start=1.2, end=2.6, step=1)]
    assert window[0] >= 1.2 - 1e-6 and window[-1] <= 2.6 + 1e-6 and any(t >= 2.0 for t in window)
