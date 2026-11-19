"""Robot tag mounts from video, and a mount-free heading track for the clock sync.

When the robot turns in place (sync wiggle, Turn Sweep), each robot tag moves on a circle around
the robot's rotation center. A fit of all tag corners in those frames gives each tag's mount
(x, y, z, yaw) in the robot frame, with the robot origin at the rotation center. The heading zero
cannot come from the video: it is set by keeping one tag's yaw at its value in setup.json.
"""

from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np
from scipy.optimize import least_squares
from scipy.sparse import lil_matrix

from .camera import Camera
from .config import RobotTag, Setup
from .detect import TagDetections
from .geometry import TAG_FROM_MARKER, Pose3, marker_corners, rot_z, tag_corners


def single_tag_field_pose(camera: Camera, corners: np.ndarray, size: float) -> Pose3 | None:
    """field_from_tag of an upright (vertical) tag from its 4 corners, or None."""
    lens = camera.lens
    n, rvecs, tvecs, errs = cv2.solvePnPGeneric(marker_corners(size), np.asarray(corners, np.float64), lens.K,
                                                lens.dist, flags=cv2.SOLVEPNP_IPPE_SQUARE)
    best = None
    for k in range(n):
        pose = camera.pose @ Pose3.from_rotvec(rvecs[k].ravel(), tvecs[k].ravel()) @ TAG_FROM_MARKER.inverse()
        up = pose.R[2, 2]  # tag +z (top edge) should point up
        if best is None or up > best[0]:
            best = (up, pose)
    return best[1] if best and best[0] > 0.5 else None


def tag_yaw(pose: Pose3) -> float:
    """Direction the tag face points, in the floor plane (rad)."""
    return float(np.arctan2(pose.R[1, 0], pose.R[0, 0]))


def heading_track(det: TagDetections, camera: Camera, setup: Setup) -> tuple[np.ndarray, np.ndarray, int]:
    """Robot heading over video time from the most-seen robot tag alone (no mount position needed:
    the tag turns exactly as the robot turns). Returns (video_t, heading rad, tag id)."""
    robot_ids = list(setup.robot_tags)
    ids, counts = np.unique(det.tag_id[np.isin(det.tag_id, robot_ids)], return_counts=True)
    if not len(ids):
        raise ValueError("no robot tags in the video")
    ref = int(ids[np.argmax(counts)])
    mount_yaw = np.radians(setup.robot_tags[ref].yaw_deg)
    m = det.tag_id == ref
    t, h = [], []
    for ti, c in zip(det.time[m], det.corners[m]):
        pose = single_tag_field_pose(camera, c, setup.robot_size)
        if pose is not None:
            t.append(ti)
            h.append(tag_yaw(pose) - mount_yaw)
    order = np.argsort(t)
    return np.array(t)[order], np.angle(np.exp(1j * np.array(h)[order])), ref


@dataclass
class MountFit:
    tags: dict[int, RobotTag]
    reference_tag: int
    rms_px: float
    frames: int
    observations: dict[int, int]
    centers: list[tuple[float, float]]
    stats: dict
    rejected: dict[int, str] = None


def fit_mounts(det: TagDetections, camera: Camera, setup: Setup, video_windows: list[tuple[float, float]],
               max_frames: int = 500, min_observations: int = 30, knot_s: float = 2.0) -> MountFit:
    """Fit robot tag mounts from frames in ``video_windows`` where the robot turns in place.

    The robot center may drift slowly while it turns (the routines hold the heading, not the
    position): it is linear in time between knots ``knot_s`` apart. That is too slow to follow
    the turns, so the center is still the rotation center."""
    robot_ids = set(setup.robot_tags)
    in_window = np.zeros(len(det.time), bool)
    window_of = np.full(len(det.time), -1)
    for w, (a, b) in enumerate(video_windows):
        m = (det.time >= a) & (det.time <= b)
        in_window |= m
        window_of[m] = w
    m = in_window & np.isin(det.tag_id, list(robot_ids))
    ids, counts = np.unique(det.tag_id[m], return_counts=True)
    all_tags = [int(i) for i, c in zip(ids, counts) if c >= min_observations]
    if not all_tags:
        raise ValueError("no robot tag is seen often enough while the robot turns in place")
    # Stage 1 uses the often-seen tags; rarely seen tags are fitted one at a time afterwards.
    most = max(counts[list(ids).index(i)] for i in all_tags)
    tags = [i for i in all_tags if counts[list(ids).index(i)] >= 0.3 * most]
    later = [i for i in all_tags if i not in tags]
    ref = tags[int(np.argmax([counts[list(ids).index(i)] for i in tags]))]
    frame_times = np.unique(det.time[m & np.isin(det.tag_id, tags)])
    if len(frame_times) > max_frames:
        frame_times = frame_times[np.linspace(0, len(frame_times) - 1, max_frames).astype(int)]
    frame_index = {t: k for k, t in enumerate(frame_times)}
    sel = m & np.isin(det.tag_id, tags) & np.isin(det.time, frame_times)
    obs_t, obs_id, obs_c = det.time[sel], det.tag_id[sel], det.corners[sel]
    obs_w = window_of[sel]
    windows = sorted(set(int(w) for w in obs_w))
    wpos = {w: k for k, w in enumerate(windows)}
    # Knots of the center track: for each window, times from its first to its last observation.
    knot_t, knot_w = [], []
    for w in windows:
        tw = obs_t[obs_w == w]
        n = max(2, int(np.ceil((tw.max() - tw.min()) / knot_s)) + 1)
        for kt in np.linspace(tw.min(), tw.max(), n):
            knot_t.append(kt)
            knot_w.append(w)
    knot_t, knot_w = np.array(knot_t), np.array(knot_w)

    def knot_weights(t, w):
        """Indices and weights of the two knots of window w around time t."""
        ks = np.flatnonzero(knot_w == w)
        kt = knot_t[ks]
        j = int(np.clip(np.searchsorted(kt, t), 1, len(kt) - 1))
        f = float(np.clip((t - kt[j - 1]) / max(kt[j] - kt[j - 1], 1e-9), 0, 1))
        return ks[j - 1], ks[j], f

    # Initial values: heading of each frame from the reference tag; tag field poses by single-tag PnP.
    size = setup.robot_size
    tag_pose = [single_tag_field_pose(camera, c, size) for c in obs_c]
    ref_yaw = np.radians(setup.robot_tags[ref].yaw_deg)
    theta0 = np.zeros(len(frame_times))
    have = np.zeros(len(frame_times), bool)
    yaw_of = {}  # (frame index, tag) -> tag yaw in the field
    for t, i, p in zip(obs_t, obs_id, tag_pose):
        if p is not None:
            yaw_of[(frame_index[t], int(i))] = tag_yaw(p)
    for (k, i), y in yaw_of.items():
        if i == ref:
            theta0[k] = y - ref_yaw
            have[k] = True
    if not have.any():
        raise ValueError("the reference robot tag has no usable pose")
    # Frames without the reference tag: use another tag, turned by its yaw relative to the reference
    # tag (measured in frames where both are visible). Interpolation would fail across large turns.
    for j in tags:
        if j == ref:
            continue
        rel = [np.exp(1j * (yaw_of[(k, j)] - yaw_of[(k, ref)])) for k in range(len(frame_times))
               if (k, j) in yaw_of and (k, ref) in yaw_of]
        if len(rel) < 5:
            continue
        offset = np.angle(np.mean(rel))
        for k in range(len(frame_times)):
            if not have[k] and (k, j) in yaw_of:
                theta0[k] = yaw_of[(k, j)] - offset - ref_yaw
                have[k] = True
    if (~have).any():
        theta0[~have] = np.interp(frame_times[~have], frame_times[have], np.unwrap(theta0[have]))
    theta0 = np.angle(np.exp(1j * theta0))
    # Centers: reference tag position minus its nominal mount, averaged per window.
    nominal = setup.robot_tags[ref]
    centers0 = np.zeros((len(windows), 2))
    for w in windows:
        pts = []
        for t, i, p, ow in zip(obs_t, obs_id, tag_pose, obs_w):
            if i == ref and p is not None and ow == w:
                th = theta0[frame_index[t]]
                pts.append(p.t[:2] - rot_z(th)[:2, :2] @ [nominal.x, nominal.y])
        centers0[wpos[w]] = np.median(pts, axis=0) if pts else 0.0
    mounts0 = {}
    for j in tags:
        vals = []
        for t, i, p, ow in zip(obs_t, obs_id, tag_pose, obs_w):
            if i == j and p is not None:
                th = theta0[frame_index[t]]
                c = centers0[wpos[ow]]
                rel = rot_z(-th)[:2, :2] @ (p.t[:2] - c)
                vals.append((rel[0], rel[1], p.t[2], np.angle(np.exp(1j * (tag_yaw(p) - th)))))
        v = np.array(vals)
        mounts0[j] = (np.median(v[:, 0]), np.median(v[:, 1]), np.median(v[:, 2]),
                      np.angle(np.mean(np.exp(1j * v[:, 3]))))

    # Parameter vector: [mounts (ref: x, y, z; others: x, y, z, yaw)] [theta per frame] [center per window]
    mount_slices, x0 = {}, []
    for j in tags:
        mx, my, mz, myaw = mounts0[j]
        if j == ref:
            mount_slices[j] = slice(len(x0), len(x0) + 3)
            x0 += [mx, my, mz]
        else:
            mount_slices[j] = slice(len(x0), len(x0) + 4)
            x0 += [mx, my, mz, myaw]
    theta_off = len(x0)
    x0 += list(np.unwrap(theta0))
    center_off = len(x0)
    x0 += list(np.concatenate([centers0[wpos[w]] for w in knot_w]))
    x0 = np.array(x0)

    local = tag_corners(size)
    meas = camera.lens.undistort(obs_c.reshape(-1, 2)).reshape(-1, 4, 2)
    cam_from_field = camera.pose.inverse()
    fx = camera.lens.fx
    obs_k = np.array([frame_index[t] for t in obs_t])
    obs_cw = np.array([wpos[w] for w in obs_w])
    obs_knots = [knot_weights(t, w) for t, w in zip(obs_t, obs_w)]

    def mount_pose(p, j):
        s = p[mount_slices[j]]
        yaw = ref_yaw if j == ref else s[3]
        return Pose3(rot_z(yaw), np.array([s[0], s[1], s[2]]))

    def residuals(p):
        out = np.empty((len(obs_t), 8))
        mounts = {j: mount_pose(p, j).apply(local) for j in tags}
        for n in range(len(obs_t)):
            th = p[theta_off + obs_k[n]]
            ka, kb, f = obs_knots[n]
            c = (1 - f) * p[center_off + 2 * ka: center_off + 2 * ka + 2] + f * p[center_off + 2 * kb: center_off + 2 * kb + 2]
            pc = cam_from_field.apply(Pose3.planar(c[0], c[1], th).apply(mounts[int(obs_id[n])]))
            out[n] = ((pc[:, :2] / np.maximum(pc[:, 2:3], 1e-3) - meas[n]) * fx).ravel()
        return out.ravel()

    sparsity = lil_matrix((8 * len(obs_t), len(x0)), dtype=int)
    for n in range(len(obs_t)):
        rows = slice(8 * n, 8 * n + 8)
        sl = mount_slices[int(obs_id[n])]
        sparsity[rows, sl.start:sl.stop] = 1
        sparsity[rows, theta_off + obs_k[n]] = 1
        ka, kb, _ = obs_knots[n]
        sparsity[rows, center_off + 2 * ka: center_off + 2 * ka + 2] = 1
        sparsity[rows, center_off + 2 * kb: center_off + 2 * kb + 2] = 1
    result = least_squares(residuals, x0, jac_sparsity=sparsity, method="trf", loss="soft_l1", f_scale=2.0,
                           x_scale="jac", max_nfev=200)
    r = result.fun.reshape(-1, 4, 2)
    rms = float(np.sqrt(np.mean(np.sum(r ** 2, axis=2))))
    fitted = {}
    for j in tags:
        pose = mount_pose(result.x, j)
        fitted[j] = RobotTag(j, float(pose.t[0]), float(pose.t[1]), float(pose.t[2]),
                             float(np.degrees(np.arctan2(pose.R[1, 0], pose.R[0, 0]))))
    per_tag = {j: float(np.sqrt(np.mean(np.sum(r[obs_id == j] ** 2, axis=2)))) for j in tags}
    knots = result.x[center_off:].reshape(-1, 2)
    centers = [tuple(knots[knot_w == w].mean(axis=0)) for w in windows]
    drift = max((float(np.ptp(knots[knot_w == w], axis=0).max()) for w in windows), default=0.0)
    observations = {j: int(np.sum(obs_id == j)) for j in tags}
    rejected = {}

    # Stage 2: each rarely seen tag alone, with the frame headings and centers of stage 1 fixed.
    frame_theta = {t: result.x[theta_off + k] for t, k in frame_index.items()}
    frame_center = {}
    for t, (ka, kb, f) in zip(obs_t, obs_knots):
        frame_center[t] = (1 - f) * knots[ka] + f * knots[kb]
    for j in later:
        mj = (det.tag_id == j) & in_window & np.isin(det.time, list(frame_center))
        if mj.sum() < 10:
            rejected[j] = f"seen in only {int(mj.sum())} frames with a known robot pose"
            continue
        jt, jc = det.time[mj], det.corners[mj]
        jmeas = camera.lens.undistort(jc.reshape(-1, 2)).reshape(-1, 4, 2)
        robots = [Pose3.planar(frame_center[t][0], frame_center[t][1], frame_theta[t]) for t in jt]
        init = []
        for t, c, rp in zip(jt, jc, robots):
            p = single_tag_field_pose(camera, c, size)
            if p is not None:
                m_ = rp.inverse() @ p
                init.append((m_.t[0], m_.t[1], m_.t[2], tag_yaw(m_)))
        if not init:
            rejected[j] = "no usable single-tag pose"
            continue
        v = np.array(init)
        q0 = np.array([np.median(v[:, 0]), np.median(v[:, 1]), np.median(v[:, 2]),
                       np.angle(np.mean(np.exp(1j * v[:, 3])))])

        def res_j(q):
            pts = Pose3(rot_z(q[3]), q[:3]).apply(local)
            out = []
            for rp, mm in zip(robots, jmeas):
                pc = cam_from_field.apply(rp.apply(pts))
                out.append(((pc[:, :2] / np.maximum(pc[:, 2:3], 1e-3) - mm) * fx).ravel())
            return np.concatenate(out)

        rj = least_squares(res_j, q0, loss="soft_l1", f_scale=2.0, method="trf")
        rms_j = float(np.sqrt(np.mean(np.sum(rj.fun.reshape(-1, 4, 2) ** 2, axis=2))))
        observations[j] = int(mj.sum())
        per_tag[j] = rms_j
        if rms_j > 3.0:
            rejected[j] = f"fit RMS {rms_j:.1f} px is more than 3 px"
            continue
        fitted[j] = RobotTag(j, float(rj.x[0]), float(rj.x[1]), float(rj.x[2]),
                             float(np.degrees(np.angle(np.exp(1j * rj.x[3])))))
    for j in tags:
        if per_tag[j] > 3.0:
            rejected[j] = f"fit RMS {per_tag[j]:.1f} px is more than 3 px"
            fitted.pop(j, None)
    return MountFit(fitted, ref, rms, len(frame_times), observations, centers,
                    {"per_tag_rms_px": per_tag, "converged": bool(result.success), "message": result.message,
                     "max_center_drift_m": drift, "knots": int(len(knot_t))},
                    rejected)
