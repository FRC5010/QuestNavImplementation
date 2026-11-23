"""Measure where the hub tags really are, and write a custom field layout.

When the tags cannot be moved to their layout positions, measure them instead: one fixed camera
films the tags; one reference tag keeps its official field pose; every other tag gets its pose
relative to that tag. The result is a WPILib AprilTag layout JSON for QuestNav, the robot code
and this tool.

Usage (from tools/groundtruth):

    .venv/bin/python -m groundtruth.maplayout --lens sessions/run2/lens_gopro.json \\
        --tags sessions/run2/tags_gopro.npz --reference 25 --measure 24 26 \\
        --out ../../src/main/deploy/apriltag/hallway-5010.json
"""

from __future__ import annotations

import argparse
import json
import sys
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

from .camera import Lens
from .config import DEFAULT_LAYOUT, FRC_TAG_SIZE_M, load_field_layout
from .detect import TagDetections
from .geometry import TAG_FROM_MARKER, Pose3, marker_corners, tag_corners


@dataclass
class TagMeasurement:
    tag_id: int
    pose: Pose3  # field_from_tag
    official: Pose3
    frames: int
    rms_px: float
    sigma_position_m: float  # uncertainty, from the scatter of independent parts of the video
    sigma_angle_deg: float

    @property
    def offset_m(self) -> np.ndarray:
        """Measured minus official position, in the field frame."""
        return self.pose.t - self.official.t

    @property
    def along_face_m(self) -> float:
        """Offset along the tag face, positive to the right as you face the tag."""
        return float(self.official.R[:, 1] @ self.offset_m)

    @property
    def out_of_face_m(self) -> float:
        return float(self.official.R[:, 0] @ self.offset_m)

    @property
    def yaw_deg(self) -> float:
        return float(np.degrees(np.arctan2(self.pose.R[1, 0], self.pose.R[0, 0])))

    @property
    def tilt_deg(self) -> float:
        """Face tilted up (+) or down (-) from vertical."""
        return float(np.degrees(np.arcsin(np.clip(self.pose.R[2, 0], -1, 1))))


def _median_corners(det: TagDetections, tag_id: int, mask=None) -> np.ndarray | None:
    m = det.tag_id == tag_id
    if mask is not None:
        m &= mask
    if m.sum() < 5:
        return None
    return np.median(det.corners[m], axis=0)


def _camera_from_tag(lens: Lens, corners: np.ndarray, size: float) -> Pose3:
    n, rvecs, tvecs, errs = cv2.solvePnPGeneric(marker_corners(size), corners.astype(np.float64), lens.K, lens.dist,
                                                flags=cv2.SOLVEPNP_IPPE_SQUARE)
    best = int(np.argmin(np.ravel(errs)))
    return Pose3.from_rotvec(rvecs[best].ravel(), tvecs[best].ravel()) @ TAG_FROM_MARKER.inverse()


def _solve(lens: Lens, medians: dict[int, np.ndarray], reference: Pose3, ref_id: int, size: float):
    """Camera pose and the poses of the other tags, from median corners. The reference tag is fixed."""
    others = [i for i in medians if i != ref_id]
    cam_from_ref = _camera_from_tag(lens, medians[ref_id], size)
    field_from_cam = reference @ cam_from_ref.inverse()
    x0 = list(field_from_cam.inverse().rotvec()) + list(field_from_cam.inverse().t)
    for i in others:
        p = field_from_cam @ _camera_from_tag(lens, medians[i], size)
        x0 += list(p.rotvec()) + list(p.t)
    local = tag_corners(size)
    meas = {i: lens.undistort(c) for i, c in medians.items()}

    def residuals(x):
        cam_from_field = Pose3.from_rotvec(x[:3], x[3:6])
        out = []
        for k, i in enumerate([ref_id] + others):
            tag = reference if i == ref_id else Pose3.from_rotvec(x[6 + 6 * (k - 1): 9 + 6 * (k - 1)],
                                                                    x[9 + 6 * (k - 1): 12 + 6 * (k - 1)])
            pc = cam_from_field.apply(tag.apply(local))
            out.append(((pc[:, :2] / pc[:, 2:3] - meas[i]) * lens.fx).ravel())
        return np.concatenate(out)

    r = least_squares(residuals, np.array(x0), method="lm")
    poses = {ref_id: reference}
    for k, i in enumerate(others):
        poses[i] = Pose3.from_rotvec(r.x[6 + 6 * k: 9 + 6 * k], r.x[9 + 6 * k: 12 + 6 * k])
    res = r.fun.reshape(-1, 4, 2)
    rms = {i: float(np.sqrt(np.mean(np.sum(res[k] ** 2, axis=1)))) for k, i in enumerate([ref_id] + others)}
    return poses, rms


def measure_tags(det: TagDetections, lens: Lens, reference_id: int, measure_ids: list[int],
                 layout: dict[int, Pose3] | None = None, size: float = FRC_TAG_SIZE_M,
                 parts: int = 8) -> dict[int, TagMeasurement]:
    layout = layout or load_field_layout()
    ids = [reference_id] + [i for i in measure_ids if i != reference_id]
    medians = {i: _median_corners(det, i) for i in ids}
    missing = [i for i, c in medians.items() if c is None]
    if reference_id in missing:
        raise ValueError(f"the reference tag {reference_id} is not in the video")
    medians = {i: c for i, c in medians.items() if c is not None}
    poses, rms = _solve(lens, medians, layout[reference_id], reference_id, size)

    # Spread: solve again on independent parts of the video.
    times = np.unique(det.time)
    edges = np.linspace(times.min(), times.max() + 1e-6, parts + 1)
    part_poses = []
    for a, b in zip(edges[:-1], edges[1:]):
        mask = (det.time >= a) & (det.time < b)
        part = {i: _median_corners(det, i, mask) for i in medians}
        if any(c is None for c in part.values()):
            continue
        part_poses.append(_solve(lens, part, layout[reference_id], reference_id, size)[0])

    out = {}
    for i, pose in poses.items():
        if i == reference_id:
            continue
        if len(part_poses) >= 3:
            pos = np.array([p[i].t for p in part_poses])
            ang = [np.degrees(np.linalg.norm((p[i].inverse() @ pose).rotvec())) for p in part_poses]
            # The parts scatter as much as one part's error; the whole video averages them.
            n = len(part_poses)
            sig_p = float(np.sqrt(np.mean(np.sum((pos - pose.t) ** 2, axis=1)) / n))
            sig_a = float(np.sqrt(np.mean(np.square(ang)) / n))
        else:
            sig_p = sig_a = float("nan")
        out[i] = TagMeasurement(i, pose, layout[i], int(np.sum(det.tag_id == i)), rms[i], sig_p, sig_a)
    return out


def write_layout(measured: dict[int, Pose3], out: str | Path, base: str | Path = DEFAULT_LAYOUT) -> None:
    """The base layout with the measured tags replaced, in WPILib's JSON format (tags and field only)."""
    data = json.loads(Path(base).read_text())
    for tag in data["tags"]:
        p = measured.get(int(tag["ID"]))
        if p is None:
            continue
        x, y, z, w = Rotation.from_matrix(p.R).as_quat()
        tag["pose"] = {"translation": {"x": float(p.t[0]), "y": float(p.t[1]), "z": float(p.t[2])},
                       "rotation": {"quaternion": {"W": float(w), "X": float(x), "Y": float(y), "Z": float(z)}}}
    Path(out).parent.mkdir(parents=True, exist_ok=True)
    Path(out).write_text(json.dumps(data, indent=2) + "\n")


def report(meas: dict[int, TagMeasurement], reference_id: int) -> str:
    lines = [f"Reference tag {reference_id}: kept at its official pose.", "",
             "| Tag | Frames | Fit (px) | Along face (m) | Out of face (m) | Height (m) | Yaw (deg) | Tilt (deg) | "
             "Uncertainty pos (mm) | Uncertainty angle (deg) |", "|---|---|---|---|---|---|---|---|---|---|"]
    for i, m in sorted(meas.items()):
        lines.append(f"| {i} | {m.frames} | {m.rms_px:.2f} | {m.along_face_m:+.3f} | {m.out_of_face_m:+.3f} | "
                     f"{m.pose.t[2]:.3f} (layout {m.official.t[2]:.3f}) | {m.yaw_deg:.1f} | {m.tilt_deg:+.1f} | "
                     f"{m.sigma_position_m * 1000:.1f} | {m.sigma_angle_deg:.2f} |")
    lines.append("")
    lines.append("Along face: positive to the right as you face the tag, relative to its official position.")
    return "\n".join(lines)


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="Measure hub tag poses from video and write a custom field layout.")
    ap.add_argument("--lens", required=True)
    ap.add_argument("--tags", required=True, help="detections file from groundtruth.detect (one fixed camera)")
    ap.add_argument("--reference", type=int, required=True, help="tag that keeps its official pose")
    ap.add_argument("--measure", type=int, nargs="+", required=True, help="tags to measure")
    ap.add_argument("--out", required=True, help="layout JSON to write")
    ap.add_argument("--base", default=str(DEFAULT_LAYOUT), help="layout to start from (default: 2026 welded)")
    ap.add_argument("--tag-size", type=float, default=FRC_TAG_SIZE_M)
    args = ap.parse_args(argv)
    meas = measure_tags(TagDetections.load(args.tags), Lens.load(args.lens), args.reference, args.measure,
                        load_field_layout(args.base), args.tag_size)
    missing = sorted(set(args.measure) - set(meas) - {args.reference})
    write_layout({i: m.pose for i, m in meas.items()}, args.out, args.base)
    print(report(meas, args.reference))
    if missing:
        print(f"\nNot in the video, kept at the base layout: {missing}")
    print(f"\nWrote {args.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
