"""Field layout and the per-setup measurements (``setup.json``)."""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from .geometry import Pose3, rot_z

DATA_DIR = Path(__file__).resolve().parent.parent / "data"
DEFAULT_LAYOUT = DATA_DIR / "2026-rebuilt-welded.json"
FRC_TAG_SIZE_M = 0.1651


def load_field_layout(path: str | Path = DEFAULT_LAYOUT) -> dict[int, Pose3]:
    """Tag poses (field_from_tag) from a WPILib AprilTag layout JSON."""
    data = json.loads(Path(path).read_text())
    tags = {}
    for tag in data["tags"]:
        tr = tag["pose"]["translation"]
        q = tag["pose"]["rotation"]["quaternion"]
        tags[int(tag["ID"])] = Pose3.from_quaternion(q["W"], q["X"], q["Y"], q["Z"], [tr["x"], tr["y"], tr["z"]])
    return tags


@dataclass
class RobotTag:
    """A tag on the robot. Position of the tag center in the robot frame; ``yaw_deg`` is the
    direction the tag face points, counter-clockwise from robot forward. The top edge is up."""

    id: int
    x: float
    y: float
    z: float
    yaw_deg: float

    def robot_from_tag(self) -> Pose3:
        return Pose3(rot_z(np.radians(self.yaw_deg)), np.array([self.x, self.y, self.z]))


@dataclass
class FloorTag:
    """A tag flat on the floor, measured from the reference hub tag.

    ``distance``: meters straight out from the reference tag face. ``lateral``: meters to the right
    as you face the reference tag. ``top_yaw_deg``: direction of the tag's top edge, counter-clockwise
    (seen from above) from "straight away from the wall". ``z``: height of the tag surface.
    """

    id: int
    distance: float
    lateral: float
    top_yaw_deg: float = 0.0
    z: float = 0.0

    def field_from_tag(self, reference: Pose3) -> Pose3:
        center = reference.R @ np.array([self.distance, self.lateral, 0.0]) + reference.t
        center[2] = self.z
        top_yaw = reference.yaw() + np.radians(self.top_yaw_deg)
        x_t = np.array([0.0, 0.0, 1.0])  # the face points up
        z_t = np.array([np.cos(top_yaw), np.sin(top_yaw), 0.0])
        y_t = np.cross(z_t, x_t)
        return Pose3(np.column_stack([x_t, y_t, z_t]), center)


@dataclass
class Setup:
    tag_size: float = FRC_TAG_SIZE_M
    reference_tag: int = 25
    hub_tag_ids: list[int] = field(default_factory=lambda: [25, 26, 24])
    robot_tags: dict[int, RobotTag] = field(default_factory=dict)
    floor_tags: dict[int, FloorTag] = field(default_factory=dict)
    layout: dict[int, Pose3] = field(default_factory=load_field_layout)

    def robot_tag_poses(self) -> dict[int, Pose3]:
        """robot_from_tag for each robot tag."""
        return {i: t.robot_from_tag() for i, t in self.robot_tags.items()}

    def static_tag_poses(self) -> dict[int, Pose3]:
        """field_from_tag for each tag that does not move: the hub tags and the floor tags."""
        poses = {i: self.layout[i] for i in self.hub_tag_ids}
        reference = self.layout[self.reference_tag]
        poses.update({i: t.field_from_tag(reference) for i, t in self.floor_tags.items()})
        return poses

    @staticmethod
    def from_dict(d: dict) -> "Setup":
        layout = load_field_layout(d["layout"]) if "layout" in d else load_field_layout()
        return Setup(
            tag_size=float(d.get("tag_size_m", FRC_TAG_SIZE_M)),
            reference_tag=int(d.get("reference_tag", 25)),
            hub_tag_ids=[int(i) for i in d.get("hub_tag_ids", [25, 26, 24])],
            robot_tags={int(t["id"]): RobotTag(int(t["id"]), t["x"], t["y"], t["z"], t["yaw_deg"])
                        for t in d.get("robot_tags", [])},
            floor_tags={int(t["id"]): FloorTag(int(t["id"]), t["distance"], t["lateral"],
                                               t.get("top_yaw_deg", 0.0), t.get("z", 0.0))
                        for t in d.get("floor_tags", [])},
            layout=layout,
        )

    @staticmethod
    def load(path: str | Path) -> "Setup":
        return Setup.from_dict(json.loads(Path(path).read_text()))
