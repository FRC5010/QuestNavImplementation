import numpy as np
import pytest

from groundtruth.sync import active_windows, sync_clock

WIGGLE = [30, -30, 30, -30, 0]


def heading_profile(t: np.ndarray, wiggle_starts, other_turns) -> np.ndarray:
    """Robot heading (deg) over robot time: still, wiggles, and some other turns."""
    keys_t, keys_h = [0.0], [0.0]
    h = 0.0
    events = [(s, "wiggle") for s in wiggle_starts] + [(s, a) for s, a in other_turns]
    for start, kind in sorted(events):
        keys_t.append(start)
        keys_h.append(h)
        if kind == "wiggle":
            time = start + 0.5
            for target in WIGGLE:
                time += 1.0 + 0.2 * abs(target - keys_h[-1]) / 60
                keys_t.append(time)
                keys_h.append(h + target)
            keys_t.append(time + 0.5)
            keys_h.append(h)
        else:
            keys_t.append(start + 2.0)
            h += kind
            keys_h.append(h)
    keys_t.append(t[-1])
    keys_h.append(h)
    # Smooth the corners a little, like a real turn.
    raw = np.interp(t, keys_t, keys_h)
    kernel = np.ones(25) / 25
    return np.convolve(raw, kernel, mode="same")


@pytest.mark.parametrize("a_true,b_true", [(87.3, 1 + 40e-6), (-12.0, 1 - 25e-6), (300.0, 1.0)])
def test_sync_recovers_offset_and_rate(a_true, b_true):
    rng = np.random.default_rng(5)
    robot_t = np.arange(0.0, 600.0, 0.004)  # 250 Hz gyro
    wiggles = [100.0, 500.0]
    turns = [(150.0, 45.0), (200.0, -90.0), (260.0, 60.0), (330.0, -30.0), (420.0, 120.0)]
    heading = heading_profile(robot_t, wiggles, turns)
    gyro = (heading + 180) % 360 - 180  # the log has a wrapped heading

    # Video: 59.94 fps, starts after the robot log, a few dropped frames, heading from truth with
    # 0.1 deg noise and a different zero.
    video_t = np.arange(0.0, 560.0, 1 / 59.94)
    video_t = np.delete(video_t, rng.choice(video_t.size, 300, replace=False))
    robot_at_frame = a_true + b_true * video_t
    keep = (robot_at_frame > 1.0) & (robot_at_frame < 599.0)
    video_t, robot_at_frame = video_t[keep], robot_at_frame[keep]
    video_heading = np.interp(robot_at_frame, robot_t, heading) + 17.0 + rng.normal(0, 0.1, video_t.size)

    # The log's "Sync Wiggle Active" windows, about 7 s each.
    flag_t = np.arange(0.0, 600.0, 0.02)
    flag = np.zeros(flag_t.size, dtype=bool)
    for w in wiggles:
        flag[(flag_t >= w) & (flag_t <= w + 7.0)] = True
    windows = active_windows(flag_t, flag)
    windows = [w for w in windows if a_true + 1 < w[0] and w[1] < a_true + b_true * 550]
    assert windows

    fit = sync_clock(video_t, video_heading, robot_t, gyro, windows)

    for e in fit.events:
        assert e.rms_deg < 0.3
    check = np.linspace(video_t[0], video_t[-1], 20)
    error = np.abs(fit.to_robot(check) - (a_true + b_true * check))
    assert error.max() < 0.002, f"max clock error {error.max() * 1000:.2f} ms"


def test_active_windows():
    t = np.arange(10.0)
    v = [0, 1, 1, 0, 0, 1, 1, 1, 0, 1]
    assert active_windows(t, v) == [(1.0, 3.0), (5.0, 8.0), (9.0, 9.0)]
