"""Video clock to robot clock, from the robot heading seen in the video and the gyro in the log.

Model: ``robot_time = a + b * video_time``. Each sync wiggle gives one point of this line.

1. Coarse: cross-correlate the turn rates of the whole recording (20 ms steps) to find one shift.
2. Fine: for each sync wiggle window from the log, find the shift that makes the video heading
   and the gyro heading agree best. The headings can start from different zeros, so the fit
   removes the mean difference.
3. Fit a and b to the wiggle results. With one wiggle, b = 1.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
from scipy.optimize import minimize_scalar
from scipy.signal import correlate, correlation_lags


@dataclass
class SyncEvent:
    robot_window: tuple[float, float]
    video_center: float
    shift: float  # robot_time - video_time at the center of the wiggle
    rms_deg: float  # heading mismatch after the fit
    samples: int


@dataclass
class ClockFit:
    a: float
    b: float
    coarse_shift: float
    events: list[SyncEvent] = field(default_factory=list)
    residuals_s: np.ndarray = field(default_factory=lambda: np.zeros(0))

    def to_robot(self, video_time):
        return self.a + self.b * np.asarray(video_time)

    def to_video(self, robot_time):
        return (np.asarray(robot_time) - self.a) / self.b


def unwrap_deg(heading_deg) -> np.ndarray:
    return np.degrees(np.unwrap(np.radians(np.asarray(heading_deg, dtype=float))))


def active_windows(times, values) -> list[tuple[float, float]]:
    """(start, end) of each run of true values in a logged boolean."""
    times = np.asarray(times, dtype=float)
    values = np.asarray(values, dtype=bool)
    windows, start = [], None
    for t, v in zip(times, values):
        if v and start is None:
            start = t
        elif not v and start is not None:
            windows.append((start, t))
            start = None
    if start is not None:
        windows.append((start, times[-1]))
    return windows


def _rate_on_grid(t, heading_deg, step):
    grid = np.arange(t[0], t[-1], step)
    h = np.interp(grid, t, unwrap_deg(heading_deg))
    return grid, np.gradient(h, step)


def coarse_shift(video_t, video_heading_deg, robot_t, gyro_heading_deg, step: float = 0.02) -> float:
    """Shift s with robot_time ≈ video_time + s, from the turn rates of the whole recording."""
    vg, vr = _rate_on_grid(np.asarray(video_t, float), video_heading_deg, step)
    rg, rr = _rate_on_grid(np.asarray(robot_t, float), gyro_heading_deg, step)
    corr = correlate(rr, vr, mode="full", method="fft")
    lags = correlation_lags(rr.size, vr.size, mode="full")
    k = lags[int(np.argmax(corr))]
    return float(rg[0] + k * step - vg[0])


def fine_shift(video_t, video_heading_deg, robot_t, gyro_heading_deg, video_window, guess: float,
               search: float = 0.15) -> tuple[float, float, int]:
    """Best shift in ``guess ± search`` over one window. Returns (shift, rms_deg, samples)."""
    video_t = np.asarray(video_t, float)
    mask = (video_t >= video_window[0]) & (video_t <= video_window[1])
    vt = video_t[mask]
    vh = unwrap_deg(video_heading_deg)[mask]
    rt = np.asarray(robot_t, float)
    gh = unwrap_deg(gyro_heading_deg)
    if vt.size < 10:
        raise ValueError("too few video samples in the sync window")

    def cost(s: float) -> float:
        d = vh - np.interp(vt + s, rt, gh)
        return float(np.mean((d - d.mean()) ** 2))

    grid = np.arange(guess - search, guess + search, 0.001)
    best = grid[int(np.argmin([cost(s) for s in grid]))]
    fine = minimize_scalar(cost, bounds=(best - 0.002, best + 0.002), method="bounded",
                           options={"xatol": 1e-6})
    return float(fine.x), float(np.sqrt(fine.fun)), int(vt.size)


def sync_clock(video_t, video_heading_deg, robot_t, gyro_heading_deg,
               robot_windows: list[tuple[float, float]]) -> ClockFit:
    """Clock fit from the video heading, the gyro heading and the logged wiggle windows."""
    if not robot_windows:
        raise ValueError("no sync wiggle windows in the log")
    coarse = coarse_shift(video_t, video_heading_deg, robot_t, gyro_heading_deg)
    events = []
    for r0, r1 in robot_windows:
        window = (r0 - coarse, r1 - coarse)
        shift, rms, n = fine_shift(video_t, video_heading_deg, robot_t, gyro_heading_deg, window, coarse)
        events.append(SyncEvent((r0, r1), 0.5 * (window[0] + window[1]), shift, rms, n))

    centers = np.array([e.video_center for e in events])
    robot_times = centers + np.array([e.shift for e in events])
    if len(events) == 1:
        a, b = float(robot_times[0] - centers[0]), 1.0
    else:
        b, a = np.polyfit(centers, robot_times, 1)
    residuals = robot_times - (a + b * centers)
    return ClockFit(float(a), float(b), coarse, events, residuals)
