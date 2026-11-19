"""Lens calibration from a ChArUco board video.

Usage (from tools/groundtruth):

    .venv/bin/python -m groundtruth.calibrate VIDEO --out sessions/lens_gopro.json
    .venv/bin/python -m groundtruth.calibrate VIDEO --start 26 --end 40 --out lens.json
    .venv/bin/python -m groundtruth.calibrate GX010617.MP4 GX020617.MP4 GX030617.MP4 --out lens.json

Several files are one recording split into chapters (GoPro and Akaso split at about 4 GB). Give
them in order; --start and --end count from the start of the first file.

The tool reads the video, finds the board in every ``--step``-th frame, drops frames where the
board moves fast (motion blur), picks up to ``--max-views`` frames that cover the image and the
board angles as evenly as possible, and calibrates. It prints a report with pass/fail checks.
A lens file is written even if a check fails, but do not use it for truth until all checks pass.
"""

from __future__ import annotations

import argparse
import json
import sys
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Iterable

import cv2
import numpy as np

from .camera import Lens
from .charuco import THRIFTYBOT, BoardDetector, BoardSpec, BoardView

MODELS = {
    # k1, k2, p1, p2, k3 and a free image center. Default; good for a GoPro in Linear mode.
    "standard": 0,
    # k1..k6 rational model. For wide or fisheye-like lenses (for example most Akaso modes).
    "rational": cv2.CALIB_RATIONAL_MODEL,
    # k1, k2 only, center fixed at the image middle. Only for poor coverage; less accurate.
    "simple": cv2.CALIB_FIX_PRINCIPAL_POINT | cv2.CALIB_ZERO_TANGENT_DIST | cv2.CALIB_FIX_K3
    | cv2.CALIB_FIX_ASPECT_RATIO,
}


@dataclass
class Limits:
    max_rms_px: float = 1.0
    max_view_rms_px: float = 1.0  # views that fit worse than this are dropped when enough remain
    min_coverage: float = 0.5  # fraction of an 8 x 6 grid of image cells that have corners
    max_edge_gap: float = 0.12  # corners must come this close (fraction of size) to every edge
    min_max_tilt_deg: float = 30.0
    max_focal_sigma: float = 0.005  # focal length standard deviation, as a fraction of f
    min_views: int = 15


@dataclass
class CalibrationResult:
    lens: Lens
    model: str
    rms_px: float
    view_times: list[float]
    view_rms_px: list[float]
    focal_sigma_px: float
    coverage: float
    edge_gaps: dict[str, float]
    tilt_deg: tuple[float, float]
    rejected_views: int
    checks: dict[str, tuple[bool, str]] = field(default_factory=dict)

    @property
    def passed(self) -> bool:
        return all(ok for ok, _ in self.checks.values())

    def report(self) -> str:
        k = self.lens
        lines = [
            f"Model: {self.model}",
            f"Views used: {len(self.view_times)} (rejected as outliers: {self.rejected_views})",
            f"RMS reprojection error: {self.rms_px:.3f} px",
            f"Focal length: fx {k.fx:.1f}, fy {k.fy:.1f} px (sigma {self.focal_sigma_px:.1f} px)",
            f"Image center: cx {k.cx:.1f}, cy {k.cy:.1f} px (image {k.width} x {k.height})",
            f"Distortion: {np.array2string(np.asarray(k.dist), precision=4)}",
            f"Coverage: {self.coverage * 100:.0f}% of image cells; closest to edges (fraction): "
            + ", ".join(f"{n} {v:.2f}" for n, v in self.edge_gaps.items()),
            f"Board tilt from face-on: {self.tilt_deg[0]:.0f} to {self.tilt_deg[1]:.0f} deg",
            "Checks:",
        ]
        lines += [f"  [{'PASS' if ok else 'FAIL'}] {name}: {msg}" for name, (ok, msg) in self.checks.items()]
        lines.append("RESULT: " + ("PASS" if self.passed else "FAIL - record a better calibration video"))
        return "\n".join(lines)


# ----------------------------------------------------------------------------- collection

def collect_views(frames: Iterable[tuple[float, np.ndarray]], detector: BoardDetector,
                  max_speed_px_s: float = 120.0) -> list[BoardView]:
    """Board views from (time, image) pairs. A view is dropped if the board moved faster than
    ``max_speed_px_s`` since the previous view, when that view is at most 0.5 s earlier (motion blur)."""
    views, previous = [], None
    for t, image in frames:
        view = detector.detect(image, t)
        if view is None:
            previous = None
            continue
        fast = False
        # Judge speed only across short gaps; across long gaps the board may have moved and stopped.
        if previous is not None and 0 < t - previous.time <= 0.5:
            common, a, b = np.intersect1d(previous.ids, view.ids, return_indices=True)
            if common.size >= 4:
                moved = np.median(np.linalg.norm(view.corners[b] - previous.corners[a], axis=1))
                fast = moved / (t - previous.time) > max_speed_px_s
        previous = view
        if not fast:
            views.append(view)
    return views


def video_frames(path: str | Path, start: float | None = None, end: float | None = None, step: int = 10,
                 progress: bool = False):
    """(time, gray image) for every ``step``-th frame between start and end (seconds)."""
    cap = cv2.VideoCapture(str(path))
    if not cap.isOpened():
        raise FileNotFoundError(f"cannot open video {path}")
    fps = cap.get(cv2.CAP_PROP_FPS)
    total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    first = int(round((start or 0.0) * fps))
    last = min(total - 1, int(round(end * fps)) if end is not None else total - 1)
    if first:
        cap.set(cv2.CAP_PROP_POS_FRAMES, first)
    k = first
    while k <= last:
        ok = cap.grab()
        if not ok:
            break
        if (k - first) % step == 0:
            ok, image = cap.retrieve()
            if not ok:
                break
            t = cap.get(cv2.CAP_PROP_POS_MSEC) / 1000.0 or k / fps
            if progress and (k - first) % (step * 50) == 0:
                print(f"  {t:7.1f} s", file=sys.stderr)
            yield t, cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        k += 1
    cap.release()


def video_duration(path: str | Path) -> float:
    cap = cv2.VideoCapture(str(path))
    duration = cap.get(cv2.CAP_PROP_FRAME_COUNT) / cap.get(cv2.CAP_PROP_FPS)
    cap.release()
    return float(duration)


def chapter_frames(paths: list[str | Path], start: float | None = None, end: float | None = None, step: int = 10,
                   progress: bool = False):
    """Like :func:`video_frames`, for one recording that the camera split into several files
    (chapters). Times count from the start of the first file."""
    offset = 0.0
    for path in paths:
        duration = video_duration(path)
        lo = None if start is None else start - offset
        hi = None if end is None else end - offset
        if (hi is None or hi >= 0) and (lo is None or lo <= duration):
            for t, image in video_frames(path, max(lo, 0.0) if lo is not None else None, hi, step, progress):
                yield t + offset, image
        offset += duration


def video_size(path: str | Path) -> tuple[int, int]:
    cap = cv2.VideoCapture(str(path))
    size = (int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)), int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)))
    cap.release()
    return size


# ----------------------------------------------------------------------------- view selection

def _features(view: BoardView, spec: BoardSpec, size: tuple[int, int]) -> np.ndarray | None:
    obj = spec.object_points(view.ids)[:, :2]
    H, _ = cv2.findHomography(obj, view.corners)
    if H is None:
        return None
    H = H / H[2, 2]
    w, h = size
    center = view.corners.mean(axis=0)
    scale = np.sqrt(abs(np.linalg.det(H[:2, :2])))
    # H[2, 0] and H[2, 1] grow with the board tilt; scale them by the board size so they compare.
    span = max(spec.squares_x, spec.squares_y)
    return np.array([center[0] / w, center[1] / h, 0.3 * np.log(scale + 1e-9), H[2, 0] * span, H[2, 1] * span])


def select_views(views: list[BoardView], spec: BoardSpec, size: tuple[int, int], max_views: int = 60) -> list[BoardView]:
    """Up to ``max_views`` views that are as different as possible (position, size and tilt)."""
    feats, keep = [], []
    for v in views:
        f = _features(v, spec, size)
        if f is not None:
            feats.append(f)
            keep.append(v)
    if len(keep) <= max_views:
        return keep
    feats = np.array(feats)
    counts = np.array([len(v.ids) for v in keep])
    chosen = [int(np.argmax(counts))]
    dist = np.linalg.norm(feats - feats[chosen[0]], axis=1)
    while len(chosen) < max_views:
        # Farthest point, with a small preference for views with more corners.
        score = dist * (0.7 + 0.3 * counts / counts.max())
        i = int(np.argmax(score))
        chosen.append(i)
        dist = np.minimum(dist, np.linalg.norm(feats - feats[i], axis=1))
    return [keep[i] for i in sorted(chosen)]


# ----------------------------------------------------------------------------- calibration

def _run(views, spec, size, flags):
    obj = [spec.object_points(v.ids) for v in views]
    img = [v.corners.reshape(-1, 1, 2) for v in views]
    w, h = size
    if flags & cv2.CALIB_FIX_PRINCIPAL_POINT:
        K0 = np.array([[w * 0.5, 0, (w - 1) / 2], [0, w * 0.5, (h - 1) / 2], [0, 0, 1]], float)
        flags |= cv2.CALIB_USE_INTRINSIC_GUESS
    else:
        K0 = None
    dist0 = np.zeros(8 if flags & cv2.CALIB_RATIONAL_MODEL else 5)
    return cv2.calibrateCameraExtended(obj, img, size, K0, dist0, flags=flags)


def calibrate(views: list[BoardView], size: tuple[int, int], spec: BoardSpec = THRIFTYBOT,
              model: str = "standard", limits: Limits = Limits()) -> CalibrationResult:
    if len(views) < 4:
        raise ValueError(f"only {len(views)} board views; at least 4 are needed (15 or more for a good result)")
    flags = MODELS[model]
    rms, K, dist, rvecs, tvecs, sd_int, _, per_view = _run(views, spec, size, flags)

    # Drop bad views (glare, blur, a bad detection) a few at a time: at most a quarter of the views
    # in each round. Undo a round that makes the result worse or moves the focal length a lot
    # (a sign that the remaining views no longer constrain the lens).
    rejected = 0
    for _ in range(8):
        per_view = per_view.ravel()
        threshold = max(min(3 * np.median(per_view), 2 * limits.max_view_rms_px), limits.max_view_rms_px)
        order = np.argsort(-per_view)
        worst = [i for i in order if per_view[i] > threshold][: max(1, len(views) // 4)]
        if not worst or len(views) - len(worst) < limits.min_views:
            break
        keep = [v for i, v in enumerate(views) if i not in set(worst)]
        trial = _run(keep, spec, size, flags)
        if trial[0] > rms or abs(trial[1][0, 0] - K[0, 0]) > 0.05 * K[0, 0]:
            break
        views = keep
        rejected += len(worst)
        rms, K, dist, rvecs, tvecs, sd_int, _, per_view = trial
    per_view = per_view.ravel()

    w, h = size
    n_dist = 8 if flags & cv2.CALIB_RATIONAL_MODEL else 5  # OpenCV may return extra zero coefficients
    lens = Lens(float(K[0, 0]), float(K[1, 1]), float(K[0, 2]), float(K[1, 2]), dist.ravel()[:n_dist].copy(), w, h)
    corners = np.concatenate([v.corners for v in views])
    cells = set(zip(np.clip((corners[:, 0] / w * 8).astype(int), 0, 7),
                    np.clip((corners[:, 1] / h * 6).astype(int), 0, 5)))
    coverage = len(cells) / 48
    gaps = {
        "left": float(corners[:, 0].min() / w),
        "right": float((w - corners[:, 0].max()) / w),
        "top": float(corners[:, 1].min() / h),
        "bottom": float((h - corners[:, 1].max()) / h),
    }
    tilts = [float(np.degrees(np.arccos(min(1.0, abs(cv2.Rodrigues(r)[0][2, 2]))))) for r in rvecs]
    focal_sigma = float(sd_int[0, 0])

    result = CalibrationResult(lens, model, float(rms), [v.time for v in views], [float(e) for e in per_view],
                               focal_sigma, coverage, gaps, (min(tilts), max(tilts)), rejected)
    checks = {
        "views": (len(views) >= limits.min_views, f"{len(views)} views (need {limits.min_views})"),
        "rms": (rms <= limits.max_rms_px, f"{rms:.3f} px (limit {limits.max_rms_px})"),
        "coverage": (coverage >= limits.min_coverage,
                     f"{coverage * 100:.0f}% of the image (need {limits.min_coverage * 100:.0f}%)"),
        "edges": (max(gaps.values()) <= limits.max_edge_gap,
                  "board reached every edge" if max(gaps.values()) <= limits.max_edge_gap else
                  "move the board closer to the " + ", ".join(n for n, g in gaps.items() if g > limits.max_edge_gap)
                  + " edge(s)"),
        "tilt": (max(tilts) >= limits.min_max_tilt_deg,
                 f"largest tilt {max(tilts):.0f} deg (need {limits.min_max_tilt_deg:.0f})"),
        "focal": (focal_sigma <= limits.max_focal_sigma * K[0, 0],
                  f"sigma {focal_sigma:.1f} px = {100 * focal_sigma / K[0, 0]:.2f}% "
                  f"(limit {100 * limits.max_focal_sigma:.1f}%)"),
    }
    result.checks = {name: (bool(ok), msg) for name, (ok, msg) in checks.items()}
    return result


def save_views(views: list[BoardView], path: str | Path) -> None:
    np.savez(path, times=np.array([v.time for v in views]),
             **{f"c{i}": v.corners for i, v in enumerate(views)}, **{f"i{i}": v.ids for i, v in enumerate(views)})


def load_views(path: str | Path) -> list[BoardView]:
    z = np.load(path)
    return [BoardView(float(t), z[f"c{i}"], z[f"i{i}"]) for i, t in enumerate(z["times"])]


def save(result: CalibrationResult, path: str | Path, source: str = "") -> None:
    data = result.lens.to_dict()
    data.update({
        "model": result.model,
        "rms_px": result.rms_px,
        "views": len(result.view_times),
        "coverage": result.coverage,
        "tilt_deg": list(result.tilt_deg),
        "passed": bool(result.passed),
        "checks": {name: {"ok": bool(ok), "message": msg} for name, (ok, msg) in result.checks.items()},
        "source": source,
        "created": datetime.now().isoformat(timespec="seconds"),
    })
    Path(path).write_text(json.dumps(data, indent=2) + "\n")


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="Calibrate a camera lens from a ChArUco board video.")
    ap.add_argument("video", nargs="+", help="video file, or all chapter files of one recording in order")
    ap.add_argument("--out", required=True, help="lens JSON file to write")
    ap.add_argument("--start", type=float, help="first second of the video to use")
    ap.add_argument("--end", type=float, help="last second of the video to use")
    ap.add_argument("--step", type=int, default=10, help="use every Nth frame (default 10)")
    ap.add_argument("--max-views", type=int, default=60)
    ap.add_argument("--model", choices=sorted(MODELS), default="standard")
    ap.add_argument("--views", help="cache file (.npz) for the detected board views: written after detection, "
                    "and read instead of the video if it exists")
    ap.add_argument("--max-speed", type=float, default=120.0,
                    help="drop frames where the board moves faster than this, in px/s (default 120)")
    ap.add_argument("--squares", type=int, nargs=2, default=[THRIFTYBOT.squares_x, THRIFTYBOT.squares_y],
                    metavar=("X", "Y"))
    ap.add_argument("--dictionary", default=THRIFTYBOT.dictionary)
    ap.add_argument("--marker-ratio", type=float, default=THRIFTYBOT.marker_ratio)
    args = ap.parse_args(argv)

    spec = BoardSpec(args.squares[0], args.squares[1], args.dictionary, args.marker_ratio)
    detector = BoardDetector(spec)
    sizes = {video_size(v) for v in args.video}
    if len(sizes) != 1:
        ap.error(f"the videos have different sizes: {sorted(sizes)}")
    size = sizes.pop()
    if args.views and Path(args.views).exists():
        views = load_views(args.views)
        print(f"Read {len(views)} board views from {args.views}", file=sys.stderr)
    else:
        print(f"Reading {len(args.video)} file(s) ({size[0]} x {size[1]}), 1 frame in {args.step}...",
              file=sys.stderr)
        views = collect_views(chapter_frames(args.video, args.start, args.end, args.step, progress=True),
                              detector, max_speed_px_s=args.max_speed)
        if args.views:
            save_views(views, args.views)
    print(f"Board found in {len(views)} sharp frames.", file=sys.stderr)
    selected = select_views(views, spec, size, args.max_views)
    result = calibrate(selected, size, spec, args.model)
    save(result, args.out, source=", ".join(str(v) for v in args.video))
    print(result.report())
    print(f"Wrote {args.out}")
    return 0 if result.passed else 1


if __name__ == "__main__":
    sys.exit(main())
