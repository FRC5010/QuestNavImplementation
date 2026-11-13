"""Camera pose in the field frame from the tags that do not move (hub tags and floor tags).

The tags do not move and the camera does not move, so the corners of each tag are combined over
all frames (median) before the fit. The spread of each tag's corners over the video shows whether
the camera moved.
"""

from __future__ import annotations

from collections import defaultdict
from dataclasses import dataclass

import cv2
import numpy as np
from scipy.optimize import least_squares

from .camera import Lens
from .geometry import Pose3, tag_corners


@dataclass
class StaticObservation:
    tag_id: int
    corners_px: np.ndarray  # (4, 2), ArUco order


@dataclass
class SurveyResult:
    pose: Pose3  # field_from_camera
    rms_px: float
    tag_rms_px: dict[int, float]  # fit residual of each tag
    tag_spread_px: dict[int, float]  # RMS spread of each tag's corners over the frames
    tag_frames: dict[int, int]
    cov: np.ndarray  # 6x6, rotation vector (rad) then translation (m), of camera_from_field
    warnings: list[str]


def survey_camera(
    lens: Lens,
    observations: list[StaticObservation],
    static_tags: dict[int, Pose3],
    tag_size: float,
    max_spread_px: float = 2.0,
    max_tag_rms_px: float = 1.0,
) -> SurveyResult:
    by_tag: dict[int, list[np.ndarray]] = defaultdict(list)
    for o in observations:
        if o.tag_id in static_tags:
            by_tag[o.tag_id].append(np.asarray(o.corners_px, dtype=float))
    if not by_tag:
        raise ValueError("no static tags were seen")

    ids = sorted(by_tag)
    local = tag_corners(tag_size)
    medians, spreads = {}, {}
    for i in ids:
        stack = np.stack(by_tag[i])  # (frames, 4, 2)
        medians[i] = np.median(stack, axis=0)
        spreads[i] = float(np.sqrt(np.mean(np.sum((stack - medians[i]) ** 2, axis=2))))

    object_points = np.concatenate([static_tags[i].apply(local) for i in ids])
    image_points = np.concatenate([medians[i] for i in ids])
    measured = lens.undistort(image_points)

    ok, rvec, tvec = cv2.solvePnP(object_points, image_points, lens.K, lens.dist, flags=cv2.SOLVEPNP_SQPNP)
    if not ok:
        raise RuntimeError("solvePnP did not find a camera pose")

    def residuals(p: np.ndarray) -> np.ndarray:
        pc = Pose3.from_rotvec(p[:3], p[3:]).apply(object_points)
        z = np.maximum(pc[:, 2], 1e-3)
        return ((pc[:, :2] / z[:, None] - measured) * lens.fx).ravel()

    result = least_squares(residuals, np.concatenate([rvec.ravel(), tvec.ravel()]), method="lm")
    camera_from_field = Pose3.from_rotvec(result.x[:3], result.x[3:])

    r = result.fun.reshape(-1, 4, 2)
    tag_rms = {i: float(np.sqrt(np.mean(np.sum(r[k] ** 2, axis=1)))) for k, i in enumerate(ids)}
    rms = float(np.sqrt(np.mean(np.sum(r.reshape(-1, 2) ** 2, axis=1))))
    dof = max(result.fun.size - 6, 1)
    variance = max(np.sum(result.fun ** 2) / dof, 0.05 ** 2)
    cov = variance * np.linalg.pinv(result.jac.T @ result.jac)

    warnings = []
    if len(ids) < 2:
        warnings.append("only one static tag was seen; the survey is weak")
    for i in ids:
        if spreads[i] > max_spread_px:
            warnings.append(f"tag {i}: corners spread {spreads[i]:.1f} px over the video; did the camera move?")
        if tag_rms[i] > max_tag_rms_px:
            warnings.append(f"tag {i}: fit error {tag_rms[i]:.1f} px; is the tag where the layout or setup says?")

    return SurveyResult(camera_from_field.inverse(), rms, tag_rms, spreads,
                        {i: len(by_tag[i]) for i in ids}, cov, warnings)
