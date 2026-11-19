"""Reads what the session analysis needs from a robot .wpilog (robot time, seconds)."""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

QUEST_FRAME = "NT:/QuestNavLog/Frame"
RAW_HEADING = "NT:/DriveState/RawHeading (deg)"
FUSED_POSE = "NT:/DriveState/Pose"
SYNC_ACTIVE = "NT:/SmartDashboard/Test/Sync Wiggle Active"
ROUTINE = "NT:/SmartDashboard/Test Routine/active"
ENABLED = "DS:enabled"


@dataclass
class Snapshot:
    time: float
    test: str
    run: int
    label: str
    quest: tuple[float, float, float] | None  # x, y, heading deg
    fused: tuple[float, float, float]


@dataclass
class RobotLog:
    path: str
    quest: np.ndarray  # (N, 4): capture time, x, y, heading deg (raw Quest robot pose)
    gyro: np.ndarray  # (N, 2): time, heading deg (wrapped)
    fused: np.ndarray  # (N, 4): time, x, y, heading deg
    sync_windows: list[tuple[float, float]]
    enabled_windows: list[tuple[float, float]]
    routines: list[tuple[float, str]]  # (time, chooser selection) changes
    snapshots: list[Snapshot] = field(default_factory=list)

    def routine_at(self, t: float) -> str:
        name = "None"
        for start, value in self.routines:
            if start <= t:
                name = value
        return name

    def enabled_at(self, t: float) -> bool:
        return any(a <= t <= b for a, b in self.enabled_windows)


def _windows(changes: list[tuple[float, bool]], end: float) -> list[tuple[float, float]]:
    out, start = [], None
    for t, v in changes:
        if v and start is None:
            start = t
        elif not v and start is not None:
            out.append((start, t))
            start = None
    if start is not None:
        out.append((start, end))
    return out


def read_log(path: str | Path) -> RobotLog:
    from wpiutil.log import DataLogReader

    entries: dict[int, tuple[str, str]] = {}
    quest, gyro, fused = [], [], []
    sync, enabled, routines, header, snaps = [], [], [], None, []
    last = 0.0
    for rec in DataLogReader(str(path)):
        if rec.isStart():
            d = rec.getStartData()
            entries[d.entry] = (d.name, d.type)
            continue
        if rec.isControl() or rec.getEntry() not in entries:
            continue
        name, typ = entries[rec.getEntry()]
        t = rec.getTimestamp() / 1e6
        last = t
        if name == QUEST_FRAME:
            v = rec.getDoubleArray()
            if len(v) >= 4:
                quest.append(v[:4])
        elif name == RAW_HEADING:
            gyro.append((t, rec.getDouble()))
        elif name == FUSED_POSE and typ.startswith("struct:Pose2d"):
            x, y, r = np.frombuffer(bytes(rec.getRaw())[:24], "<f8")
            fused.append((t, x, y, np.degrees(r)))
        elif name == SYNC_ACTIVE:
            sync.append((t, rec.getBoolean()))
        elif name == ENABLED:
            enabled.append((t, rec.getBoolean()))
        elif name == ROUTINE and typ == "string":
            routines.append((t, rec.getString()))
        elif name == "console":
            for line in rec.getString().splitlines():
                if line.startswith("SNAPSHOT_HEADER,"):
                    header = line.split(",")[1:]
                elif line.startswith("SNAPSHOT,") and header:
                    snaps.append(_snapshot(header, line.split(",")[1:]))
    return RobotLog(str(path), np.array(quest, float).reshape(-1, 4), np.array(gyro, float).reshape(-1, 2),
                    np.array(fused, float).reshape(-1, 4), _windows(sync, last), _windows(enabled, last),
                    routines, [s for s in snaps if s is not None])


def _snapshot(header: list[str], values: list[str]) -> Snapshot | None:
    d = dict(zip(header, values))
    try:
        quest = None
        if d.get("questX") not in (None, "", "NaN", "nan"):
            quest = (float(d["questX"]), float(d["questY"]), float(d["questDeg"]))
        return Snapshot(float(d["time"]), d["test"], int(d["run"]), d["label"], quest,
                        (float(d["fusedX"]), float(d["fusedY"]), float(d["fusedDeg"])))
    except (KeyError, ValueError):
        return None
