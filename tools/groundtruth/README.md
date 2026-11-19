# Ground-truth tool

Finds the true robot pose from GoPro video of AprilTags, on a laptop, after a test. The design
is in [docs/ground-truth-design.md](../../docs/ground-truth-design.md).

Status: the full pipeline works on real sessions: lens calibration, tag detection, camera survey,
robot tag mounts from video, clock sync, truth, and the QuestNav comparison report. First results
are for runs 1 and 2 (GoPro only; see "Known limits").

## Analyze a session

1. Detect the tags in each camera's video (once per video; about real time for 4K):

   ```bash
   .venv/bin/python -m groundtruth.detect GX010617.MP4 GX020617.MP4 GX030617.MP4 --out sessions/run2/tags_gopro.npz
   ```

2. Write `sessions/<run>/setup.json` (see `setup.example.json`). List only the hub tags that are
   up and in their layout position in `hub_tag_ids`; give a tag that is somewhere else in
   `tag_overrides` (`id`, `x`, `y`, `z`, `yaw_deg`).

3. Run the analysis (keep the Mac awake; about 1 to 2 minutes):

   ```bash
   .venv/bin/python -m groundtruth.session --log sessions/run2/FRC_....wpilog --setup sessions/run2/setup.json \
       --camera gopro sessions/run2/lens_gopro.json sessions/run2/tags_gopro.npz --out sessions/run2/report
   ```

   Add `--camera` once for each camera. The report folder gets `report.md`, `samples.csv`,
   `snapshots.csv`, `summary.json`, `path.png` and `error.png`.

What the analysis does for each camera: survey (camera pose from the hub tags), a first clock
sync from one robot tag's heading, robot tag mounts fitted from the turn-in-place frames (sync
wiggles and Turn Sweep; the robot center may drift slowly), truth for every frame, and a second
clock sync. A camera that sees fewer than 2 hub tags gets its pose from the robot poses that the
other camera measured. Use `--mounts setup` to use the mounts in `setup.json` instead of fitting.

Errors are reported two ways: **absolute** (field frame; valid only if QuestNav was aligned to
the field) and **aligned** (the Quest path is moved and turned as one piece onto truth; shows
drift and consistency).

### Known limits

- Without measured robot tag yaws, the robot's heading zero is not known from the video. With a
  path that has shape (Drift Loop), the analysis takes it from the Quest path; heading errors are
  then consistency, not absolute heading.
- The latency estimate is not consistent yet between sessions (+35 ms and -25 ms).
- The Akaso is not usable yet: it sees only one hub tag, and its lens calibration does not pass.

## Lens calibration

Each camera needs a lens file, made from a short video of the ChArUco board, recorded with the
**same camera settings** as the test sessions. The camera position does not matter.

Record (1 to 2 minutes for each camera):

1. Mount the board flat on a stiff backing.
2. Hold it 0.4 to 0.7 m from the camera (closer for the Akaso). Its markers are small.
3. Move slowly. Stop for a moment at each position.
4. Cover the center, the four corners and the four edges of the image.
5. At several positions, tilt the board 30° to 45° left, right, up and down.

Then run:

```bash
.venv/bin/python -m groundtruth.calibrate path/to/board_video.mp4 --out sessions/lens_gopro.json
```

Options: `--start`/`--end` (seconds) to use part of a video, `--step` (default 10: use 1 frame in
10), `--model standard|rational|simple` (`rational` for a wide, fisheye-like Akaso view). The
default board is the team's ThriftyBot board (22 × 15 squares, `DICT_4X4_250`, markers 0.70 of a
square); `--squares`, `--dictionary` and `--marker-ratio` change it.

The tool prints a report and writes the lens file. Use the file only if all checks pass:

| Check | Limit |
|---|---|
| Views | 15 or more sharp board views |
| RMS reprojection error | 0.5 px or less |
| Coverage | Corners in 50% or more of the image (8 × 6 grid) |
| Edges | Corners within 12% of every image edge |
| Tilt | Largest board tilt 30° or more |
| Focal length | Standard deviation 0.5% or less |

If a check fails, the report says what to change (for example, "move the board closer to the
left edge").

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

The tests use a synthetic hallway: the fake hub (tags 24 and 21 facing the test area, 25 and 26 on the other face), three floor tags, a robot
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
| `charuco.py` | The calibration board model and a detector for its small markers (crop, enlarge, detect again) |
| `calibrate.py` | Lens calibration from a board video, with view selection, outlier removal and pass/fail checks |
| `detect.py` | AprilTag detection in session videos (chapters, parallel blocks), saved to a cache file |
| `robotlog.py` | Reads the Quest frames, gyro, fused pose, wiggle and routine windows, and snapshots from a `.wpilog` |
| `mounts.py` | Robot tag mounts from the turn-in-place frames, and a mount-free heading track for the first clock sync |
| `session.py` | The session analysis and report |

## Frames

- **Field:** the robot code's 2026 frame. The origin is the blue alliance corner.
- **Robot:** +x forward, +y left, +z up. The origin is the rotation center on the floor.
- **Tag:** +x out of the tag face, +y to your right as you face the tag, +z up.
- **Camera:** OpenCV. +x right in the image, +y down, +z forward.

Tag corners are in OpenCV ArUco order: top-left, top-right, bottom-right, bottom-left, as you see
the tag from the front. The printed FRC AprilTags are OpenCV's 36h11 images turned 180°, so
`detect.py` reorders OpenCV's corners for them.

## setup.json

Copy `setup.example.json` to `setup.json` and replace the values with your measurements.

| Field | Meaning |
|---|---|
| `tag_size_m` | Edge of the black square of the printed tags. Measure a print |
| `reference_tag` | The hub tag that the test area is in front of, and that the floor tag positions are measured from (24) |
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
