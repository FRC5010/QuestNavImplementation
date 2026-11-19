"""AprilTag (36h11) detection in session videos.

Usage (from tools/groundtruth):

    .venv/bin/python -m groundtruth.detect GX010617.MP4 GX020617.MP4 GX030617.MP4 --out sessions/run2/tags_gopro.npz

Several files are one recording split into chapters; give them in order. Times count from the
start of the first file. The work is split into time blocks that run in parallel. The result is
a cache file that the session analysis reads; run this once for each camera and session.
"""

from __future__ import annotations

import argparse
import os
import sys
import time
from concurrent.futures import ProcessPoolExecutor
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np

from .calibrate import chapter_frames, video_duration, video_size

A = cv2.aruco


@dataclass
class TagDetections:
    """All tag detections of one camera: one row per (frame, tag)."""

    time: np.ndarray  # (N,) seconds from the start of the recording
    tag_id: np.ndarray  # (N,) int
    corners: np.ndarray  # (N, 4, 2) float32 pixels, ArUco order
    size: tuple[int, int]
    sources: list[str]

    def frame_times(self) -> np.ndarray:
        return np.unique(self.time)

    def at(self, t: float) -> dict[int, np.ndarray]:
        """{tag_id: corners} for the frame at time t (exact match)."""
        m = self.time == t
        return {int(i): c for i, c in zip(self.tag_id[m], self.corners[m])}

    def frames(self):
        """(time, {tag_id: corners}) for each frame with at least one tag, in time order."""
        order = np.argsort(self.time, kind="stable")
        t, ids, cs = self.time[order], self.tag_id[order], self.corners[order]
        bounds = np.flatnonzero(np.diff(t)) + 1
        for a, b in zip(np.r_[0, bounds], np.r_[bounds, len(t)]):
            yield float(t[a]), {int(i): c for i, c in zip(ids[a:b], cs[a:b])}

    def save(self, path: str | Path) -> None:
        np.savez_compressed(path, time=self.time, tag_id=self.tag_id, corners=self.corners,
                            size=np.array(self.size), sources=np.array(self.sources),
                            corner_order=np.array(CORNER_ORDER_VERSION))

    @staticmethod
    def load(path: str | Path) -> "TagDetections":
        z = np.load(path)
        corners = z["corners"]
        if "corner_order" not in z.files:  # older file: OpenCV's AprilTag corner order
            corners = np.roll(corners, -APRILTAG_CORNER_ROLL, axis=1)
        return TagDetections(z["time"], z["tag_id"], corners, tuple(int(v) for v in z["size"]),
                             [str(s) for s in z["sources"]])


def make_detector() -> "cv2.aruco.ArucoDetector":
    p = A.DetectorParameters()
    p.cornerRefinementMethod = A.CORNER_REFINE_SUBPIX
    p.cornerRefinementWinSize = 5
    return A.ArucoDetector(A.getPredefinedDictionary(A.DICT_APRILTAG_36h11), p)


# The official AprilTag images (printed for FRC) are OpenCV's 36h11 images turned 180 degrees, so
# for an upright printed tag OpenCV gives the corners bottom-right, bottom-left, top-left,
# top-right (measured on the hallway tags). The rest of the tool uses the ArUco order: top-left,
# top-right, bottom-right, bottom-left. Rolling by 2 converts.
APRILTAG_CORNER_ROLL = 2
CORNER_ORDER_VERSION = 2  # files without this version were saved in OpenCV's AprilTag order


def detect_image(detector, gray: np.ndarray, max_id: int = 586) -> list[tuple[int, np.ndarray]]:
    """[(tag_id, corners)] with corners top-left, top-right, bottom-right, bottom-left."""
    corners, ids, _ = detector.detectMarkers(gray)
    if ids is None:
        return []
    return [(int(i), np.roll(c.reshape(4, 2), -APRILTAG_CORNER_ROLL, axis=0).astype(np.float32))
            for c, i in zip(corners, ids.ravel()) if i <= max_id]


def _detect_block(args) -> tuple[list[float], list[int], list[np.ndarray]]:
    paths, start, end, step = args
    detector = make_detector()
    times, ids, corners = [], [], []
    for t, gray in chapter_frames(paths, start, end, step):
        if t >= end:
            break
        for tag_id, c in detect_image(detector, gray):
            times.append(t)
            ids.append(tag_id)
            corners.append(c)
    return times, ids, corners


def detect_videos(paths: list[str], step: int = 2, start: float = 0.0, end: float | None = None,
                  workers: int | None = None, block_s: float = 20.0, progress: bool = True) -> TagDetections:
    total = sum(video_duration(p) for p in paths)
    end = total if end is None else min(end, total)
    edges = list(np.arange(start, end, block_s)) + [end]
    blocks = [(paths, a, b, step) for a, b in zip(edges[:-1], edges[1:])]
    workers = workers or max(1, (os.cpu_count() or 2) - 1)
    times, ids, corners = [], [], []
    t0 = time.time()
    with ProcessPoolExecutor(workers) as pool:
        for k, (bt, bi, bc) in enumerate(pool.map(_detect_block, blocks)):
            times += bt
            ids += bi
            corners += bc
            if progress:
                print(f"  block {k + 1}/{len(blocks)} done ({time.time() - t0:.0f} s)", file=sys.stderr)
    return TagDetections(np.array(times, dtype=float), np.array(ids, dtype=int),
                         np.array(corners, dtype=np.float32).reshape(-1, 4, 2), video_size(paths[0]), list(paths))


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="Detect AprilTags in a session video (one camera).")
    ap.add_argument("video", nargs="+", help="video file, or all chapter files of one recording in order")
    ap.add_argument("--out", required=True, help="detections file to write (.npz)")
    ap.add_argument("--step", type=int, default=2, help="use 1 frame in N (default 2: 30 per second at 60 fps)")
    ap.add_argument("--start", type=float, default=0.0)
    ap.add_argument("--end", type=float)
    ap.add_argument("--workers", type=int)
    args = ap.parse_args(argv)
    det = detect_videos(args.video, args.step, args.start, args.end, args.workers)
    det.save(args.out)
    counts = {int(i): int(n) for i, n in zip(*np.unique(det.tag_id, return_counts=True))}
    print(f"{len(det.frame_times())} frames with tags; detections per tag: {counts}")
    print(f"Wrote {args.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
