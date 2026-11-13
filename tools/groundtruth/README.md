# Ground-truth tool

Finds the true robot pose from GoPro video of AprilTags, on a laptop, after a test. The design
is in [docs/ground-truth-design.md](../../docs/ground-truth-design.md).

Status: phase 1. The math (solver, camera survey, clock sync) is done and tested with synthetic
data. Lens calibration, tag detection in video, joint truth from the log, and the report are
phase 3.

## Install (macOS)

You need [uv](https://docs.astral.sh/uv/) or Python 3.11 or later.

```bash
cd tools/groundtruth
uv venv --python 3.11 .venv
uv pip install --python .venv/bin/python -r requirements.txt
```

## Run the tests

```bash
cd tools/groundtruth
.venv/bin/python -m pytest
```

The tests use a synthetic hallway: the fake hub (tags 25, 26 and 24), three floor tags, a robot
with a tag on each bumper corner, and two GoPro-like cameras 90° apart and 2.3 m high.

## Modules

| Module | Job |
|---|---|
| `geometry.py` | `Pose3` transforms, frame conventions, tag corner order |
| `camera.py` | `Lens` (OpenCV model, 5 distortion coefficients) and `Camera` (a lens plus a fixed pose in the field) |
| `config.py` | The field layout (`data/2026-rebuilt-welded.json`, the same file the robot code uses) and `setup.json` |
| `solver.py` | Robot pose (x, y, θ) from robot-tag corners in one or two cameras, with a covariance and accept checks |
| `survey.py` | Camera pose from the hub and floor tags, with a fit error and a spread for each tag |
| `sync.py` | Video clock to robot clock from the sync wiggles |

## Frames

- **Field:** the robot code's 2026 frame. The origin is the blue alliance corner.
- **Robot:** +x forward, +y left, +z up. The origin is the rotation center on the floor.
- **Tag:** +x out of the tag face, +y to your right as you face the tag, +z up.
- **Camera:** OpenCV. +x right in the image, +y down, +z forward.

Tag corners are in OpenCV ArUco order: top-left, top-right, bottom-right, bottom-left, as you see
the tag from the front.

## setup.json

Copy `setup.example.json` to `setup.json` and replace the values with your measurements.

| Field | Meaning |
|---|---|
| `tag_size_m` | Edge of the black square of the printed tags. Measure a print |
| `reference_tag` | The hub tag that the floor tag positions are measured from (25) |
| `hub_tag_ids` | The hub tags on the hallway walls |
| `robot_tags` | Each robot tag: center position in the robot frame (m), and `yaw_deg`, the direction the face points, counter-clockwise from robot forward |
| `floor_tags` | Each floor tag: `distance` straight out from the reference tag face (m), `lateral` to your right as you face that tag (m), and `top_yaw_deg`, the direction of the tag's top edge, counter-clockwise (seen from above) from "straight away from the wall" |

## What the robot logs for this tool

| Topic in the .wpilog | Content |
|---|---|
| `NT:/QuestNavLog/Frame` | `[capture time (s), x (m), y (m), heading (deg)]` for each Quest tracking frame, raw robot pose |
| `NT:/DriveState/RawHeading (deg)` | Gyro heading, wrapped to ±180° |
| `NT:/DriveState/Pose`, `NT:/DriveState/Speeds` | Fused pose and robot speeds |
| `NT:/SmartDashboard/Test/Sync Wiggle Active` | True during each sync wiggle |
| Console `SYNC_WIGGLE,begin\|end,<label>,<time>` | The same, as text |
