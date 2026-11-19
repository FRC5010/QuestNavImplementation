"""Session analysis: truth from the camera videos, compared with QuestNav and the fused pose.

Usage (from tools/groundtruth), after ``groundtruth.detect`` for each camera:

    .venv/bin/python -m groundtruth.session --log sessions/run1/FRC_....wpilog \\
        --setup sessions/run1/setup.json \\
        --camera gopro sessions/run2/lens_gopro.json sessions/run1/tags_gopro.npz \\
        --camera akaso sessions/run2/lens_akaso.json sessions/run1/tags_akaso.npz \\
        --out sessions/run1/report

Steps for each camera: survey (camera pose from the hub and floor tags), truth for each frame
(robot pose from the robot tags), and clock sync (video time to robot time, from the sync
wiggles). Then the cameras are combined and compared with the Quest pose and the fused pose.

Two kinds of error are reported:

- **Absolute**: Quest pose minus truth, in the field frame. Valid only if QuestNav was aligned to
  the field (it saw the hub tags).
- **Aligned**: the Quest path is first moved and turned as one rigid piece to fit truth best.
  This shows drift and consistency even when QuestNav was not aligned to the field.
"""

from __future__ import annotations

import argparse
import csv
import json
import sys
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from .camera import Camera, Lens
from .config import Setup
from .detect import TagDetections
from .geometry import Pose3, tag_corners, wrap_angle
from .robotlog import RobotLog, read_log
from .solver import Limits, Observation, solve_robot_pose
from .survey import StaticObservation, SurveyResult, survey_camera
from .sync import ClockFit, sync_clock
from .mounts import MountFit, fit_mounts, heading_track
import copy

MATCH_S = 0.05  # largest gap to the nearest Quest frame when matching a truth sample


# ----------------------------------------------------------------------------- truth for one camera

@dataclass
class Track:
    """Truth samples. ``t`` is robot time (NaN until the clock sync)."""

    name: str
    video_t: np.ndarray
    x: np.ndarray
    y: np.ndarray
    theta: np.ndarray  # rad
    sigma: np.ndarray  # m, worst direction
    sigma_theta: np.ndarray  # rad
    rms: np.ndarray  # px
    n_obs: np.ndarray
    accepted: np.ndarray
    t: np.ndarray = field(default=None)

    def __post_init__(self):
        if self.t is None:
            self.t = np.full(len(self.video_t), np.nan)

    def subset(self, mask) -> "Track":
        return Track(self.name, *(getattr(self, f)[mask] for f in
                                  ("video_t", "x", "y", "theta", "sigma", "sigma_theta", "rms", "n_obs", "accepted")),
                     t=self.t[mask])

    @property
    def good(self) -> "Track":
        return self.subset(self.accepted)


@dataclass
class CameraResult:
    name: str
    camera: Camera
    survey: SurveyResult
    track: Track | None = None
    clock: ClockFit | None = None
    sync_error: str = ""
    mount_fit: MountFit | None = None
    mount_error: str = ""
    robot_survey: str = ""  # set when the camera pose came from the robot (weak static survey)

    @property
    def strong(self) -> bool:
        return len(self.survey.tag_frames) >= 2


TRUTH_LIMITS = Limits(max_rms_px=2.0)  # real corners are noisier than synthetic ones


def in_place_windows(log: RobotLog) -> list[tuple[float, float]]:
    """Robot-time windows where the robot turns in place: sync wiggles and Turn Sweep runs."""
    windows = list(log.sync_windows)
    for a, b in log.enabled_windows:
        if "Turn Sweep" in log.routine_at(a + 0.5):
            windows.append((a, b))
    return sorted(windows)


def survey_from_detections(setup: Setup, lens: Lens, det: TagDetections, max_frames: int = 900) -> SurveyResult:
    static = setup.static_tag_poses()
    mask = np.isin(det.tag_id, list(static))
    times = np.unique(det.time[mask])
    if len(times) > max_frames:
        times = times[np.linspace(0, len(times) - 1, max_frames).astype(int)]
    keep = mask & np.isin(det.time, times)
    obs = [StaticObservation(int(i), c) for i, c in zip(det.tag_id[keep], det.corners[keep])]
    return survey_camera(lens, obs, static, setup.tag_size)


def survey_from_robot(lens: Lens, det: TagDetections, clock: ClockFit, truth: Track, setup: Setup,
                      start: Pose3, max_frames: int = 1500) -> tuple[Pose3, float, int]:
    """Camera pose from the robot tags at the robot poses that another camera measured, plus the
    static tags. For a camera that sees fewer than 2 static tags."""
    from scipy.optimize import least_squares
    robot_tags = setup.robot_tag_poses()
    local = tag_corners(setup.robot_size)
    static = setup.static_tag_poses()
    order = np.argsort(truth.t)
    tt, tx, ty, tth = truth.t[order], truth.x[order], truth.y[order], np.unwrap(truth.theta[order])
    obj, img = [], []
    frames = [(t, tags) for t, tags in det.frames()]
    if len(frames) > max_frames:
        frames = [frames[i] for i in np.linspace(0, len(frames) - 1, max_frames).astype(int)]
    for vt, tags in frames:
        rt = float(clock.to_robot(vt))
        k = np.searchsorted(tt, rt)
        if k <= 0 or k >= len(tt) or tt[k] - tt[k - 1] > 0.05:
            continue
        f = (rt - tt[k - 1]) / (tt[k] - tt[k - 1])
        pose = Pose3.planar(tx[k - 1] + f * (tx[k] - tx[k - 1]), ty[k - 1] + f * (ty[k] - ty[k - 1]),
                            tth[k - 1] + f * (tth[k] - tth[k - 1]))
        for i, c in tags.items():
            if i in robot_tags:
                obj.append((pose @ robot_tags[i]).apply(local))
                img.append(c)
            elif i in static:
                obj.append(static[i].apply(tag_corners(setup.tag_size)))
                img.append(c)
    if len(obj) < 20:
        raise ValueError("too few robot tag observations at known robot poses")
    obj = np.concatenate(obj)
    meas = lens.undistort(np.concatenate(img).reshape(-1, 2))
    inv = start.inverse()

    def res(p):
        pc = Pose3.from_rotvec(p[:3], p[3:]).apply(obj)
        return ((pc[:, :2] / np.maximum(pc[:, 2:3], 1e-3) - meas) * lens.fx).ravel()

    r = least_squares(res, np.concatenate([inv.rotvec(), inv.t]), loss="soft_l1", f_scale=3.0, method="trf")
    rms = float(np.sqrt(np.mean(np.sum(r.fun.reshape(-1, 2) ** 2, axis=1))))
    return Pose3.from_rotvec(r.x[:3], r.x[3:]).inverse(), rms, len(obj) // 4


def truth_track(name: str, setup: Setup, camera: Camera, det: TagDetections, limits: Limits = Limits()) -> Track:
    robot_tags = setup.robot_tag_poses()
    rows = []
    prev, prev_t = None, -1e9
    for t, tags in det.frames():
        obs = [Observation(camera, i, c) for i, c in tags.items() if i in robot_tags]
        if not obs:
            continue
        initial = prev if prev is not None and t - prev_t < 0.25 else None
        sol = solve_robot_pose(obs, robot_tags, setup.robot_size, initial, limits)
        if not np.isfinite(sol.x):
            continue
        rows.append((t, sol.x, sol.y, sol.theta, sol.sigma_position_m, sol.sigma_theta_rad, sol.rms_px,
                     sol.n_observations, sol.accepted))
        if sol.accepted:
            prev, prev_t = sol.xyt, t
    a = np.array(rows, dtype=float).reshape(-1, 9)
    return Track(name, a[:, 0], a[:, 1], a[:, 2], a[:, 3], a[:, 4], a[:, 5], a[:, 6], a[:, 7].astype(int),
                 a[:, 8].astype(bool))


def sync_track(track: Track, log: RobotLog) -> ClockFit:
    good = track.good
    if len(good.video_t) < 20:
        raise ValueError(f"only {len(good.video_t)} accepted truth frames")
    fit = sync_clock(good.video_t, np.degrees(good.theta), log.gyro[:, 0], log.gyro[:, 1], log.sync_windows)
    track.t = fit.to_robot(track.video_t)
    return fit


def combine(tracks: list[Track], max_dt: float = 0.02) -> tuple[Track, dict]:
    """One truth track in robot time: the inverse-variance mean where two cameras have samples
    within ``max_dt``, else the single camera. Also returns how much the cameras disagree."""
    goods = [tr.good for tr in tracks if tr.t is not None and np.isfinite(tr.t).any()]
    goods = [g.subset(np.isfinite(g.t)) for g in goods]
    if not goods:
        raise ValueError("no camera has synced truth")
    if len(goods) == 1:
        g = goods[0]
        order = np.argsort(g.t)
        return g.subset(order), {}
    a, b = goods[0], goods[1]
    rows, used_b = [], np.zeros(len(b.t), bool)
    order_b = np.argsort(b.t)
    bt = b.t[order_b]
    diffs = []
    for i in range(len(a.t)):
        k = np.searchsorted(bt, a.t[i])
        cand = [j for j in (k - 1, k) if 0 <= j < len(bt) and abs(bt[j] - a.t[i]) <= max_dt]
        if cand:
            j = order_b[min(cand, key=lambda j: abs(bt[j] - a.t[i]))]
            used_b[j] = True
            wa, wb = 1 / max(a.sigma[i], 1e-4) ** 2, 1 / max(b.sigma[j], 1e-4) ** 2
            dth = wrap_angle(b.theta[j] - a.theta[i])
            diffs.append((b.x[j] - a.x[i], b.y[j] - a.y[i], dth))
            rows.append((a.t[i], (wa * a.x[i] + wb * b.x[j]) / (wa + wb), (wa * a.y[i] + wb * b.y[j]) / (wa + wb),
                         wrap_angle(a.theta[i] + dth * wb / (wa + wb)), 1 / np.sqrt(wa + wb),
                         min(a.sigma_theta[i], b.sigma_theta[j]), max(a.rms[i], b.rms[j]), a.n_obs[i] + b.n_obs[j], 2))
        else:
            rows.append((a.t[i], a.x[i], a.y[i], a.theta[i], a.sigma[i], a.sigma_theta[i], a.rms[i], a.n_obs[i], 1))
    for j in np.flatnonzero(~used_b):
        rows.append((b.t[j], b.x[j], b.y[j], b.theta[j], b.sigma[j], b.sigma_theta[j], b.rms[j], b.n_obs[j], 1))
    r = np.array(sorted(rows), dtype=float)
    track = Track("combined", r[:, 0], r[:, 1], r[:, 2], r[:, 3], r[:, 4], r[:, 5], r[:, 6], r[:, 7].astype(int),
                  np.ones(len(r), bool), t=r[:, 0])
    d = np.array(diffs).reshape(-1, 3)
    agreement = {
        "pairs": len(d),
        "position_rms_m": float(np.sqrt(np.mean(d[:, 0] ** 2 + d[:, 1] ** 2))) if len(d) else float("nan"),
        "position_median_m": float(np.median(np.hypot(d[:, 0], d[:, 1]))) if len(d) else float("nan"),
        "offset_m": [float(d[:, 0].mean()), float(d[:, 1].mean())] if len(d) else [float("nan")] * 2,
        "heading_rms_deg": float(np.degrees(np.sqrt(np.mean(d[:, 2] ** 2)))) if len(d) else float("nan"),
        "two_camera_samples": int(np.sum(r[:, 8] == 2)),
    }
    return track, agreement


# ----------------------------------------------------------------------------- comparison

def interp_pose(series: np.ndarray, t: np.ndarray, max_gap: float = MATCH_S) -> tuple[np.ndarray, np.ndarray]:
    """Pose (x, y, heading rad) from ``series`` (time, x, y, heading deg) at times ``t``, and a mask
    of the times that have a sample within ``max_gap``."""
    s = series[np.argsort(series[:, 0], kind="stable")]
    s = s[np.r_[True, np.diff(s[:, 0]) > 0]]
    if len(s) < 2:
        return np.full((len(t), 3), np.nan), np.zeros(len(t), bool)
    heading = np.unwrap(np.radians(s[:, 3]))
    out = np.column_stack([np.interp(t, s[:, 0], s[:, 1]), np.interp(t, s[:, 0], s[:, 2]),
                           np.interp(t, s[:, 0], heading)])
    k = np.clip(np.searchsorted(s[:, 0], t), 1, len(s) - 1)
    gap = np.minimum(np.abs(s[k, 0] - t), np.abs(s[k - 1, 0] - t))
    ok = (gap <= max_gap) & (t >= s[0, 0]) & (t <= s[-1, 0])
    out[:, 2] = (out[:, 2] + np.pi) % (2 * np.pi) - np.pi
    return out, ok


def rigid_fit(src: np.ndarray, dst: np.ndarray) -> tuple[float, np.ndarray]:
    """Rotation angle and translation that move 2D points ``src`` onto ``dst`` best (least squares)."""
    cs, cd = src.mean(axis=0), dst.mean(axis=0)
    a, b = src - cs, dst - cd
    phi = np.arctan2(np.sum(a[:, 0] * b[:, 1] - a[:, 1] * b[:, 0]), np.sum(a[:, 0] * b[:, 0] + a[:, 1] * b[:, 1]))
    R = np.array([[np.cos(phi), -np.sin(phi)], [np.sin(phi), np.cos(phi)]])
    return float(phi), cd - R @ cs


def apply_rigid(phi: float, shift: np.ndarray, xy: np.ndarray, heading: np.ndarray):
    R = np.array([[np.cos(phi), -np.sin(phi)], [np.sin(phi), np.cos(phi)]])
    return xy @ R.T + shift, (heading + phi + np.pi) % (2 * np.pi) - np.pi


def stats(err_xy: np.ndarray, err_heading: np.ndarray) -> dict:
    d = np.hypot(err_xy[:, 0], err_xy[:, 1]) if len(err_xy) else np.zeros(0)
    if not len(d):
        return {"n": 0}
    h = np.degrees(np.abs(err_heading))
    return {"n": int(len(d)), "rms_m": float(np.sqrt(np.mean(d ** 2))), "median_m": float(np.median(d)),
            "p95_m": float(np.percentile(d, 95)), "max_m": float(d.max()),
            "mean_offset_m": [float(err_xy[:, 0].mean()), float(err_xy[:, 1].mean())],
            "heading_rms_deg": float(np.sqrt(np.mean(h ** 2))), "heading_p95_deg": float(np.percentile(h, 95))}


@dataclass
class Comparison:
    t: np.ndarray
    truth: np.ndarray  # (N, 3) x, y, heading rad
    quest: np.ndarray
    fused: np.ndarray
    quest_ok: np.ndarray
    fused_ok: np.ndarray
    moving: np.ndarray
    routine: list[str]
    enabled: np.ndarray
    heading_fix_deg: float
    align_phi: float
    align_shift: np.ndarray
    quest_aligned: np.ndarray
    latency_ms: float | None
    summary: dict


def moving_mask(track: Track, speed=0.05, turn_deg_s=5.0) -> np.ndarray:
    t, x, y, th = track.t, track.x, track.y, np.unwrap(track.theta)
    if len(t) < 3:
        return np.zeros(len(t), bool)
    w = 5
    k = np.arange(len(t))
    a, b = np.clip(k - w, 0, len(t) - 1), np.clip(k + w, 0, len(t) - 1)
    dt = np.maximum(t[b] - t[a], 1e-3)
    v = np.hypot(x[b] - x[a], y[b] - y[a]) / dt
    om = np.degrees(np.abs(th[b] - th[a])) / dt
    return (v > speed) | (om > turn_deg_s)


def compare(truth: Track, log: RobotLog, heading_fix: str = "auto") -> Comparison:
    t = truth.t
    T = np.column_stack([truth.x, truth.y, truth.theta])
    Q, q_ok = interp_pose(log.quest, t)
    F, f_ok = interp_pose(log.fused, t, max_gap=0.1)

    moving = moving_mask(truth)
    routine = [log.routine_at(x) for x in t]
    enabled = np.array([log.enabled_at(x) for x in t])

    # Rigid alignment of the Quest path onto truth. If the path is too small to fix a rotation from
    # positions (for example a turn sweep in place), use the headings for the rotation.
    phi, shift, method = 0.0, np.zeros(2), "none"
    if q_ok.sum() >= 3:
        extent = np.percentile(np.linalg.norm(T[q_ok, :2] - np.median(T[q_ok, :2], axis=0), axis=1), 99)
        if extent >= 0.25:
            phi, shift = rigid_fit(Q[q_ok, :2], T[q_ok, :2])
            method = "positions"
        else:
            phi = float(np.angle(np.mean(np.exp(1j * (T[q_ok, 2] - Q[q_ok, 2])))))
            R = np.array([[np.cos(phi), -np.sin(phi)], [np.sin(phi), np.cos(phi)]])
            shift = T[q_ok, :2].mean(axis=0) - R @ Q[q_ok, :2].mean(axis=0)
            method = "headings (path too small for positions)"

    # The heading zero of truth comes from the reference robot tag's yaw in setup.json, which is not
    # measured for a rough setup. When the path has shape, the rotation between the Quest frame and
    # the field comes from positions alone, and the Quest heading (which agrees with its own path)
    # then gives the robot's heading zero. Heading errors are then about consistency and drift, not
    # absolute heading.
    fix = 0.0
    if heading_fix == "auto" and method == "positions":
        fix = float(np.degrees(np.angle(np.mean(np.exp(1j * (Q[q_ok, 2] + phi - T[q_ok, 2]))))))
        if abs(fix) < 0.5:
            fix = 0.0
        T[:, 2] = (T[:, 2] + np.radians(fix) + np.pi) % (2 * np.pi) - np.pi

    qa_xy, qa_h = apply_rigid(phi, shift, Q[:, :2], Q[:, 2])
    QA = np.column_stack([qa_xy, qa_h])

    def errs(P, ok, mask=None):
        m = ok if mask is None else ok & mask
        return P[m, :2] - T[m, :2], (P[m, 2] - T[m, 2] + np.pi) % (2 * np.pi) - np.pi

    summary = {"samples": int(len(t)), "quest_matched": int(q_ok.sum()), "fused_matched": int(f_ok.sum()),
               "heading_fix_deg": fix, "alignment": {"rotation_deg": float(np.degrees(phi)),
                                                     "shift_m": [float(shift[0]), float(shift[1])],
                                                     "method": method}}
    groups = {"all": np.ones(len(t), bool), "stopped": ~moving, "moving": moving}
    for name in sorted(set(r for r, e in zip(routine, enabled) if e and r != "None")):
        groups[f"routine: {name}"] = np.array([r == name for r in routine]) & enabled
    summary["absolute_quest"] = {g: stats(*errs(Q, q_ok, m)) for g, m in groups.items()}
    summary["aligned_quest"] = {g: stats(*errs(QA, q_ok, m)) for g, m in groups.items()}
    summary["absolute_fused"] = {g: stats(*errs(F, f_ok, m)) for g, m in groups.items()}

    # Latency from the heading while the robot turns. A constant time lag looks like a rotation of
    # a curved path, so positions cannot separate it from the alignment; headings can, because the
    # alignment only adds a constant to them.
    latency = None
    th = np.unwrap(T[:, 2])
    om = np.zeros(len(t))
    if len(t) > 10:
        k = np.arange(len(t))
        lo, hi = np.clip(k - 5, 0, len(t) - 1), np.clip(k + 5, 0, len(t) - 1)
        om = np.degrees(np.abs(th[hi] - th[lo])) / np.maximum(t[hi] - t[lo], 1e-3)
    turning = q_ok & (om > 10.0)
    if turning.sum() > 50:
        best = None
        for dt in np.arange(-0.15, 0.1501, 0.0025):
            Qs, ok = interp_pose(log.quest, t + dt)
            m = turning & ok
            d = np.angle(np.exp(1j * (Qs[m, 2] - T[m, 2])))
            d = np.angle(np.exp(1j * (d - np.angle(np.mean(np.exp(1j * d))))))
            e = float(np.mean(d ** 2))
            if best is None or e < best[1]:
                best = (dt, e)
        latency = float(best[0] * 1000)
        summary["latency_ms"] = latency
        summary["latency_note"] = ("Quest time shift that fits the truth heading best while turning. Positive: "
                                   "the Quest timestamps are later than the real time of the pose (the pose is "
                                   "older than its timestamp says).")
    return Comparison(t, T, Q, F, q_ok, f_ok, moving, routine, enabled, fix, phi, shift, QA, latency, summary)


def snapshot_table(truth: Track, log: RobotLog, comp: Comparison, window: float = 0.5) -> list[dict]:
    rows = []
    for s in log.snapshots:
        m = (truth.t >= s.time - window) & (truth.t <= s.time + 0.05)
        if m.sum() < 3 or s.quest is None:
            continue
        x, y = truth.x[m].mean(), truth.y[m].mean()
        th = np.angle(np.mean(np.exp(1j * (truth.theta[m] + np.radians(comp.heading_fix_deg)))))
        q = np.array(s.quest)
        qa_xy, qa_h = apply_rigid(comp.align_phi, comp.align_shift, q[None, :2], np.radians([q[2]]))
        rows.append({"time": s.time, "test": s.test, "run": s.run, "label": s.label, "samples": int(m.sum()),
                     "truth_x": x, "truth_y": y, "truth_deg": float(np.degrees(th)),
                     "quest_x": q[0], "quest_y": q[1], "quest_deg": q[2],
                     "abs_error_m": float(np.hypot(q[0] - x, q[1] - y)),
                     "aligned_error_m": float(np.hypot(qa_xy[0, 0] - x, qa_xy[0, 1] - y)),
                     "abs_heading_error_deg": float(np.degrees(wrap_angle(np.radians(q[2]) - th)))})
    return rows


# ----------------------------------------------------------------------------- report

def _fmt_stats(s: dict) -> str:
    if not s or s.get("n", 0) == 0:
        return "| – | – | – | – | – | – |"
    return (f"| {s['n']} | {s['rms_m'] * 100:.1f} | {s['median_m'] * 100:.1f} | {s['p95_m'] * 100:.1f} | "
            f"{s['max_m'] * 100:.1f} | {s['heading_rms_deg']:.2f} |")


def write_report(out: Path, setup: Setup, log: RobotLog, cams: list[CameraResult], truth: Track, agreement: dict,
                 comp: Comparison, snaps: list[dict], original: Setup | None = None) -> None:
    out.mkdir(parents=True, exist_ok=True)
    L = ["# Ground truth session report", ""]
    if setup.rough:
        L += ["> **Warning:** `setup.json` is marked `rough`: the robot tag positions are estimates. The "
              "position numbers below can be off by several centimeters. Measure the tags and run again.", ""]
    if setup.notes:
        L += [f"Setup notes: {setup.notes}", ""]
    L += [f"Log: `{log.path}`", ""]

    L += ["## Cameras", "", "| Camera | Survey tags (frames) | Survey RMS (px) | Truth frames (accepted) | "
          "Median sigma (cm) | Clock fit | Sync residual (ms) |", "|---|---|---|---|---|---|---|"]
    for c in cams:
        s, tr = c.survey, c.track
        tags = ", ".join(f"{i} ({n})" for i, n in s.tag_frames.items())
        good = tr.good
        sig = f"{np.median(good.sigma) * 100:.2f}" if len(good.sigma) else "–"
        clock = (f"robot = {c.clock.a:.3f} + {c.clock.b:.8f} × video ({len(c.clock.events)} wiggles)"
                 if c.clock else f"not synced: {c.sync_error}")
        res = ", ".join(f"{e.rms_deg:.2f}°" for e in c.clock.events) if c.clock else "–"
        L.append(f"| {c.name} | {tags} | {s.rms_px:.2f} | {len(tr.video_t)} ({tr.accepted.sum()}) | {sig} | {clock} | "
                 f"heading fit {res} |")
    for c in cams:
        for w in c.survey.warnings:
            L.append(f"- {c.name}: {w}")
        if c.robot_survey:
            L.append(f"- {c.name}: {c.robot_survey}")
    L.append("")
    fits = [c for c in cams if c.mount_fit is not None]
    if fits:
        L += ["## Robot tag mounts", "", "Fitted from the frames where the robot turns in place (sync wiggles, Turn "
              "Sweep). The robot origin is its rotation center. The heading zero comes from the reference tag's yaw "
              "in setup.json.", "", "| Tag | setup.json (x, y, z m; yaw deg) | Used for truth (x, y, z m; yaw deg) |",
              "|---|---|---|"]
        orig = original or setup
        for j in sorted((original or setup).robot_tags):
            a, b = orig.robot_tags.get(j), setup.robot_tags.get(j)
            fa = f"({a.x:.3f}, {a.y:.3f}, {a.z:.3f}; {a.yaw_deg:.1f})" if a else "–"
            fb = f"({b.x:.3f}, {b.y:.3f}, {b.z:.3f}; {b.yaw_deg:.1f})" if b else "not used"
            L.append(f"| {j} | {fa} | {fb} |")
        for c in fits:
            f = c.mount_fit
            L.append(f"\n{c.name}: fit RMS {f.rms_px:.2f} px over {f.frames} frames, reference tag {f.reference_tag}, "
                     f"observations {f.observations}, per tag RMS "
                     + ", ".join(f"{j}: {v:.2f} px" for j, v in f.stats["per_tag_rms_px"].items()))
        for c in cams:
            if c.mount_error:
                L.append(f"\n{c.name}: no mount fit ({c.mount_error})")
            if c.mount_fit is not None and c.mount_fit.rejected:
                L.append(f"\n{c.name}: tags left out: " + ", ".join(f"{j} ({why})" for j, why in c.mount_fit.rejected.items()))
        L.append("")
    if agreement:
        L += ["## Camera agreement (self-check)", "",
              f"Where both cameras saw the robot at the same time ({agreement['pairs']} samples), their truth "
              f"differs by {agreement['position_median_m'] * 100:.1f} cm (median), {agreement['position_rms_m'] * 100:.1f} cm "
              f"(RMS), with a mean offset of ({agreement['offset_m'][0] * 100:.1f}, {agreement['offset_m'][1] * 100:.1f}) cm "
              f"and {agreement['heading_rms_deg']:.2f}° heading (RMS). Truth cannot be better than this.", ""]

    S = comp.summary
    L += ["## QuestNav accuracy", "",
          f"Truth samples: {S['samples']}, matched to a Quest frame: {S['quest_matched']}.", ""]
    if comp.heading_fix_deg:
        L += [f"Truth heading was turned by {comp.heading_fix_deg:.1f}° so that its zero matches the robot heading "
              "implied by the Quest path (the rotation between the Quest frame and the field comes from positions). "
              "Heading errors below are therefore consistency and drift, not absolute heading. Positions are not "
              "affected. Measured robot tag yaws in setup.json remove the need for this.", ""]
    al = S["alignment"]
    L += [f"Rigid alignment of the Quest path onto truth: rotation {al['rotation_deg']:.2f}°, shift "
          f"({al['shift_m'][0]:.3f}, {al['shift_m'][1]:.3f}) m, fit from {al['method']}. If QuestNav was aligned to the field, these "
          "should be near 0; then the absolute errors are meaningful.", ""]
    head = ["| Group | Samples | RMS (cm) | Median (cm) | 95% (cm) | Max (cm) | Heading RMS (deg) |",
            "|---|---|---|---|---|---|---|"]
    for title, key in [("Absolute (field frame)", "absolute_quest"), ("Aligned (drift and consistency)", "aligned_quest"),
                       ("Fused pose, absolute", "absolute_fused")]:
        L += [f"### {title}", ""] + head
        for g, s in S[key].items():
            L.append(f"| {g} " + _fmt_stats(s))
        L.append("")
    if comp.latency_ms is not None:
        L += [f"Best-fit Quest time shift while turning: {comp.latency_ms:.0f} ms. {S['latency_note']} The truth "
              "clock comes from the gyro log, so this is the Quest timing relative to the gyro timing.", ""]

    if snaps:
        L += ["## Snapshots", "", "| Test | Run | Label | Truth (x, y, deg) | Quest (x, y, deg) | Absolute error (cm) | "
              "Aligned error (cm) | Heading error (deg) |", "|---|---|---|---|---|---|---|---|"]
        for r in snaps:
            L.append(f"| {r['test']} | {r['run']} | {r['label']} | ({r['truth_x']:.3f}, {r['truth_y']:.3f}, "
                     f"{r['truth_deg']:.1f}) | ({r['quest_x']:.3f}, {r['quest_y']:.3f}, {r['quest_deg']:.1f}) | "
                     f"{r['abs_error_m'] * 100:.1f} | {r['aligned_error_m'] * 100:.1f} | {r['abs_heading_error_deg']:.2f} |")
        L.append("")
    L += ["## Files", "", "- `samples.csv`: every truth sample with the matched Quest and fused pose",
          "- `snapshots.csv`: the snapshot table", "- `summary.json`: all numbers above",
          "- `path.png`, `error.png`: plots", ""]
    (out / "report.md").write_text("\n".join(L))

    with open(out / "samples.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["robot_time", "truth_x", "truth_y", "truth_deg", "truth_sigma_m", "quest_x", "quest_y",
                    "quest_deg", "quest_aligned_x", "quest_aligned_y", "fused_x", "fused_y", "fused_deg",
                    "moving", "enabled", "routine"])
        for i in range(len(comp.t)):
            q = comp.quest[i] if comp.quest_ok[i] else [np.nan] * 3
            qa = comp.quest_aligned[i] if comp.quest_ok[i] else [np.nan] * 3
            fu = comp.fused[i] if comp.fused_ok[i] else [np.nan] * 3
            w.writerow([f"{comp.t[i]:.4f}", f"{comp.truth[i, 0]:.4f}", f"{comp.truth[i, 1]:.4f}",
                        f"{np.degrees(comp.truth[i, 2]):.2f}", f"{truth.sigma[i]:.4f}", f"{q[0]:.4f}", f"{q[1]:.4f}",
                        f"{np.degrees(q[2]):.2f}", f"{qa[0]:.4f}", f"{qa[1]:.4f}", f"{fu[0]:.4f}", f"{fu[1]:.4f}",
                        f"{np.degrees(fu[2]):.2f}", int(comp.moving[i]), int(comp.enabled[i]), comp.routine[i]])
    if snaps:
        with open(out / "snapshots.csv", "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=list(snaps[0]))
            w.writeheader()
            w.writerows(snaps)
    (out / "summary.json").write_text(json.dumps({"agreement": agreement, **comp.summary}, indent=2))
    _plots(out, comp)


def _plots(out: Path, comp: Comparison) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    ok = comp.quest_ok
    fig, axes = plt.subplots(1, 2, figsize=(13, 6))
    for ax, Q, title in [(axes[0], comp.quest, "Absolute (field frame)"), (axes[1], comp.quest_aligned, "Aligned")]:
        ax.plot(comp.truth[:, 0], comp.truth[:, 1], ".", ms=2, color="#1f6f43", label="Truth (cameras)")
        ax.plot(Q[ok, 0], Q[ok, 1], ".", ms=2, color="#b4422c", label="QuestNav")
        ax.set_aspect("equal")
        ax.set_title(title)
        ax.set_xlabel("x (m)")
        ax.set_ylabel("y (m)")
        ax.legend(loc="best", markerscale=6)
        ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(out / "path.png", dpi=130)
    plt.close(fig)

    fig, axes = plt.subplots(2, 1, figsize=(13, 6), sharex=True)
    e_abs = np.hypot(*(comp.quest[ok, :2] - comp.truth[ok, :2]).T) * 100
    e_al = np.hypot(*(comp.quest_aligned[ok, :2] - comp.truth[ok, :2]).T) * 100
    h = np.degrees((comp.quest[ok, 2] - comp.truth[ok, 2] + np.pi) % (2 * np.pi) - np.pi)
    axes[0].plot(comp.t[ok], e_abs, ".", ms=2, color="#b4422c", label="Absolute")
    axes[0].plot(comp.t[ok], e_al, ".", ms=2, color="#2c5fb4", label="Aligned")
    axes[0].set_ylabel("Position error (cm)")
    axes[0].legend(loc="upper left", markerscale=6)
    axes[0].grid(alpha=0.3)
    axes[1].plot(comp.t[ok], h, ".", ms=2, color="#555555")
    axes[1].set_ylabel("Heading error (deg)")
    axes[1].set_xlabel("Robot time (s)")
    axes[1].grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(out / "error.png", dpi=130)
    plt.close(fig)


# ----------------------------------------------------------------------------- command line

def run(log_path: str, setup: Setup, cameras: list[tuple[str, str, str]], out: str, heading_fix: str = "auto",
        mounts: str = "fit", progress=print) -> tuple[Comparison, list[CameraResult], dict]:
    log = read_log(log_path)
    progress(f"Log: {len(log.quest)} Quest frames, {len(log.sync_windows)} sync wiggles, {len(log.snapshots)} snapshots")
    results, dets = [], {}
    # 1. Survey and a first clock sync from one tag's heading (no mount positions needed).
    for name, lens_path, tags_path in cameras:
        lens = Lens.load(lens_path)
        det = TagDetections.load(tags_path)
        dets[name] = det
        survey = survey_from_detections(setup, lens, det)
        cam = Camera(name, lens, survey.pose)
        progress(f"{name}: survey RMS {survey.rms_px:.2f} px from tags {sorted(survey.tag_frames)}")
        result = CameraResult(name, cam, survey)
        try:
            vt, heading, ref = heading_track(det, cam, setup)
            result.clock = sync_clock(vt, np.degrees(heading), log.gyro[:, 0], log.gyro[:, 1], log.sync_windows)
            progress(f"{name}: first clock sync from tag {ref}: robot = {result.clock.a:.3f} + "
                     f"{result.clock.b:.8f} * video")
        except ValueError as e:
            result.sync_error = str(e)
            progress(f"{name}: not synced: {e}")
        results.append(result)

    # 2. Robot tag mounts from the frames where the robot turns in place (cameras with a strong
    #    survey only: at least 2 static tags).
    used = copy.deepcopy(setup)
    strong = [r for r in results if r.strong] or results
    if mounts == "fit":
        windows = in_place_windows(log)
        fits = []
        for r in strong:
            if r.clock is None:
                continue
            vw = [(float(r.clock.to_video(a)), float(r.clock.to_video(b))) for a, b in windows]
            try:
                r.mount_fit = fit_mounts(dets[r.name], r.camera, setup, vw)
                fits.append(r.mount_fit)
                progress(f"{r.name}: mount fit RMS {r.mount_fit.rms_px:.2f} px from {r.mount_fit.frames} frames, "
                         f"tags {r.mount_fit.observations}, rejected {r.mount_fit.rejected}")
            except ValueError as e:
                r.mount_error = str(e)
                progress(f"{r.name}: no mount fit: {e}")
        # For each tag, use the fit with the most observations of it; leave out tags no fit accepted.
        for j in list(setup.robot_tags):
            best = max((f for f in fits if j in f.tags), key=lambda f: f.observations[j], default=None)
            if best is not None:
                used.robot_tags[j] = best.tags[j]
            elif any(j in (f.rejected or {}) for f in fits):
                del used.robot_tags[j]

    # 3. Truth for the strong cameras, then a second, finer clock sync from the full truth heading.
    def truth_and_sync(r):
        r.track = truth_track(r.name, used, r.camera, dets[r.name], TRUTH_LIMITS)
        progress(f"{r.name}: {len(r.track.video_t)} truth frames, {r.track.accepted.sum()} accepted")
        if r.clock is not None:
            first = r.clock
            try:
                r.clock = sync_track(r.track, log)
            except ValueError:
                r.clock = first
                r.track.t = first.to_robot(r.track.video_t)

    for r in strong:
        truth_and_sync(r)
    # 4. Weak cameras: camera pose from the robot poses that the strong cameras measured.
    for r in results:
        if r in strong:
            continue
        reference = [x.track for x in strong if x.clock is not None and x.track is not None]
        if r.clock is not None and reference:
            ref_truth, _ = combine(reference)
            try:
                pose, rms, n = survey_from_robot(r.camera.lens, dets[r.name], r.clock, ref_truth, used, r.camera.pose)
                r.camera = Camera(r.name, r.camera.lens, pose)
                r.robot_survey = f"camera pose from {n} robot tag views at known robot poses, RMS {rms:.2f} px"
                progress(f"{r.name}: {r.robot_survey}")
            except ValueError as e:
                r.robot_survey = f"not possible: {e}"
        truth_and_sync(r)
    synced = [r.track for r in results if r.clock is not None and r.track is not None]
    truth, agreement = combine(synced)
    comp = compare(truth, log, heading_fix)
    snaps = snapshot_table(truth, log, comp)
    write_report(Path(out), used, log, results, truth, agreement, comp, snaps, original=setup)
    return comp, results, agreement


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="Compare QuestNav with ground truth from camera video.")
    ap.add_argument("--log", required=True, help="robot .wpilog of the session")
    ap.add_argument("--setup", required=True, help="setup.json with the tag measurements")
    ap.add_argument("--camera", nargs=3, action="append", required=True, metavar=("NAME", "LENS", "TAGS"),
                    help="camera name, lens JSON (groundtruth.calibrate), detections (groundtruth.detect)")
    ap.add_argument("--out", required=True, help="report folder")
    ap.add_argument("--heading-fix", choices=["auto", "none"], default="auto")
    ap.add_argument("--mounts", choices=["fit", "setup"], default="fit",
                    help="fit: robot tag mounts from the turn-in-place frames (default); setup: use setup.json")
    args = ap.parse_args(argv)
    comp, _, _ = run(args.log, Setup.load(args.setup), args.camera, args.out, args.heading_fix, args.mounts,
                     progress=lambda m: print(m, file=sys.stderr))
    print((Path(args.out) / "report.md").read_text())
    return 0


if __name__ == "__main__":
    sys.exit(main())
