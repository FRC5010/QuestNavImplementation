import json

import numpy as np

from groundtruth.config import load_field_layout
from groundtruth.detect import TagDetections
from groundtruth.geometry import Pose3, rot_z
from groundtruth.maplayout import measure_tags, write_layout
from synthetic import make_cameras, make_setup, visible_corners
from scipy.spatial.transform import Rotation

LAYOUT = load_field_layout()


def hallway_truth():
    """The hub as built: tag 24 is 0.35 m right of its official spot, tag 26 leans 2 degrees."""
    real = {25: LAYOUT[25]}
    t24 = LAYOUT[24]
    real[24] = Pose3(t24.R, t24.t + t24.R[:, 1] * 0.348)
    t26 = LAYOUT[26]
    lean = Rotation.from_rotvec(t26.R[:, 1] * np.radians(2.0)).as_matrix()  # about the tag's horizontal axis
    real[26] = Pose3(lean @ t26.R, t26.t)
    return real


def detections(camera, real, frames, noise, rng):
    times, ids, corners = [], [], []
    for k in range(frames):
        for i, pose in real.items():
            px = visible_corners(camera, pose, 0.1651)
            if px is not None:
                times.append(k / 30)
                ids.append(i)
                corners.append(px + rng.normal(0, noise, px.shape))
    return TagDetections(np.array(times), np.array(ids), np.array(corners, np.float32), (3840, 2160), ["synthetic"])


def test_measures_moved_and_leaning_tags(tmp_path):
    rng = np.random.default_rng(5)
    real = hallway_truth()
    cam = make_cameras(make_setup())[1]  # sees 24, 25 and 26
    det = detections(cam, real, 400, 0.5, rng)

    meas = measure_tags(det, cam.lens, 25, [24, 26], LAYOUT)

    m24, m26 = meas[24], meas[26]
    assert np.linalg.norm(m24.pose.t - real[24].t) < 0.005
    assert abs(m24.along_face_m - 0.348) < 0.005
    assert np.linalg.norm(m26.pose.t - real[26].t) < 0.005
    assert abs(m26.tilt_deg - LAYOUT_TILT(real[26])) < 0.3
    assert abs(abs(m26.tilt_deg) - 2.0) < 0.3
    assert m24.sigma_position_m < 0.005 and m24.rms_px < 0.5

    out = tmp_path / "hallway.json"
    write_layout({i: m.pose for i, m in meas.items()}, out)
    data = json.loads(out.read_text())
    assert set(data) == {"tags", "field"}  # WPILib's schema only
    again = load_field_layout(out)
    assert np.allclose(again[24].t, m24.pose.t) and np.allclose(again[26].R, m26.pose.R, atol=1e-9)
    assert np.allclose(again[25].t, LAYOUT[25].t) and np.allclose(again[1].t, LAYOUT[1].t)


def LAYOUT_TILT(pose):
    return float(np.degrees(np.arcsin(np.clip(pose.R[2, 0], -1, 1))))
