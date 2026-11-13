# Ground truth from GoPro video: design

Status: phase 1 is built (robot logging, sync wiggle, and the tool's solver, survey and sync,
in `tools/groundtruth/`). Phases 2 to 4 are not built. Coordinates in this document use the 2026 field frame, because the
robot code uses it (see the 2026 frame note in the testing guide).

## Goal

Find the true robot pose during a test, without the headset, so that we can compare the
QuestNav pose with it. The true pose is called **truth** in this document.

| Item | Target |
|---|---|
| Position error of truth, robot stopped | 1 cm or less |
| Position error of truth, robot moving at 0.75 m/s | 2 cm or less |
| Heading error of truth | 0.3° or less |
| Measuring with the laser | Once per setup, not at each mark |

Truth is calculated **on a laptop, after the test**, from GoPro video and the robot's `.wpilog`.
Nothing about truth runs on the robot, and truth never goes into the drivetrain pose estimator.

## Summary

1. A fake hub on an outside corner of the hallway has tags 24 and 21 on one face and tags 25 and
   26 on the other face. The headset uses these tags.
2. Two GoPros on tripods, 2.0 m to 2.5 m high, look down at the test area from directions about
   90° apart. They record the whole test.
3. The robot has four AprilTags, one on each bumper corner at 45°. Their IDs are not in the
   field layout.
4. At the start and the end of each test, the robot does a **sync wiggle**: it turns left and
   right in place. The gyro records the wiggle in the `.wpilog`, and the GoPros see it. This
   aligns the video clocks to the robot clock.
5. A Python tool on the laptop finds the tags in each video frame, calculates truth, aligns it
   with the log, and writes a report and a new `.wpilog` with truth in it for AdvantageScope.

Processing after the test, not during it, because:

- The GoPros record full-quality video. A live GoPro stream is lower in quality and has a delay.
- No network between the cameras and the robot is necessary.
- A test can be processed again later with a better calibration or a better solver.

## Why the cameras are high and 90° apart

A camera that sees a tag measures well **across** its view and badly **along** its view (the
distance from the camera). Example: a GoPro at 4K in Linear lens mode has a focal length of about
2100 px. For a 16.5 cm tag 3 m away, the error for 1 px of corner error is:

| Method | Error for 1 px |
|---|---|
| Across the view: r / f | about 1.4 mm |
| Along the view, from the tag size (normal PnP): r² / (f·s) | about 2.6 cm |
| Along the view, from the known tag height (camera 1.5 m above the tags): (h² + r²) / (h·f) | about 3.6 mm |

- **Known height.** The robot is on the floor, so each robot tag is at a known height. A camera
  that looks down sees the distance as a vertical pixel position. This is much more accurate
  than the tag size. So the solver has only three unknowns: x, y and θ.
- **Two directions.** The "across" direction of one camera is the "along" direction of the other
  camera. A joint solution uses the best direction of each camera.

## Hardware

| Item | Quantity | Notes |
|---|---|---|
| Camera | 2 | A GoPro HERO13 Black and an Akaso. See [Cameras](#cameras) |
| Tripod, 2.0 m to 2.5 m | 2 | Must not move during a session. Tape the legs to the floor |
| AprilTags, 36h11, 165.1 mm, flat (on aluminum or foam board) | 4 robot + 4 hub + 3 floor | Print at 100% and measure the printed size |
| ChArUco calibration board | 1 | For the lens calibration. Flat and stiff |
| External power for the GoPros | 2 | A session can be longer than one battery |
| Lights, if the hallway is dark | as needed | A fast shutter needs light |

### Tag IDs

| Use | IDs | Where |
|---|---|---|
| Hub | 24, 21 on the face toward the test area. 25, 26 on the other face | Centers 1.124 m high. 24 and 25 are 0.248 m from the corner. 21 and 26 are about 0.60 m from the corner |
| Robot | 1, 2, 3, 4 (from the FRC set) | Bumper corners, at 45°. Measure which ID is on which corner and enter it in `setup.json` |
| Floor survey | 50, 51, 52 | Flat on the floor in the test area, at laser-measured points |

The robot tags (1 to 4) are in the field layout, but this is not a problem: the headset is on the
same robot, so it can never see them, and the tool uses them only as robot tags. Do not put
tags 1 to 4 anywhere else in the hallway.

The floor tags (50 to 52) are not in the field layout, so QuestNav must ignore them. The headset
sees them only at a very low angle, if at all. Make sure of this on the headset before the first
test.

### Cameras

The team has a **GoPro HERO13 Black** and an **Akaso** action camera (model unknown). The Akaso
records 4K at 60 fps, and its stabilization can be turned off.
The two cameras can be different: each camera has its own lens calibration and its own clock
sync.

- **GoPro HERO13 Black:** the main camera. It has a Linear lens mode and manual (Protune) shutter
  and ISO.
- **Akaso:** the second camera. Most Akaso models have a fisheye lens, a smaller choice of manual
  settings, and electronic stabilization (EIS). See [Akaso settings](#akaso-settings).

Use the same settings for the lens calibration and for every test. If a setting changes, do the
lens calibration again. The tool reads the time of each frame from the video file, not from the
frame number, so 59.94 fps and dropped frames are not a problem.

### GoPro HERO13 Black settings

The names are from the HERO13 menus. Select **Pro** controls to see the Protune settings.

| Setting | Value | Reason |
|---|---|---|
| Resolution, aspect, frame rate | 4K, 16:9, 60 fps | Resolution gives accuracy. 60 fps gives timing |
| Lens | Linear (not "Linear + Horizon Leveling" or "Linear + Horizon Lock") | Nearly straight lines. Horizon leveling rotates the image |
| HyperSmooth | **Off** | Stabilization moves the image. Then the camera pose is not fixed |
| Zoom | 1× | Zoom changes the lens model |
| Lens mod | None | A lens mod changes the lens model |
| Shutter | 1/480 s (or 1/960 s) | Prevents smear on a moving robot |
| ISO min / max | 100 / 1600 | Allows the fast shutter. Add light if the image is noisy |
| White balance | A fixed value, for example 5500K (not Auto) | Prevents color changes during a run |
| Sharpness | Low | Strong sharpening moves tag edges |
| Bit rate | High | Fewer compression artifacts on the tag edges |
| Orientation | Locked | Prevents a rotated recording |
| Auto power off | Never | A session can be long. Use USB-C power |

### Akaso settings

The menus differ between Akaso models. Set these if the camera has them:

| Setting | Value | Reason |
|---|---|---|
| Stabilization (EIS, gyro, anti-shake) | **Off** | Stabilization moves the image |
| Field of view | One fixed value. Use "Narrow" or "Medium" if the test area still fits | A narrower view has less fisheye distortion and more pixels on each tag |
| Distortion correction | On or off, but the same every time | It changes the lens model |
| Resolution, frame rate | 4K, 60 fps | The same as the GoPro |
| Exposure, ISO, shutter | Manual and fast, if the camera allows it | Prevents smear |
| Date stamp, watermark | Off | A stamp can cover a tag |

Some low-cost cameras make 4K by enlarging a smaller image, or make 60 fps by repeating frames.
Both are possible on an unknown Akaso model. Phase 2 checks this: the lens calibration error
shows a soft image, and the detection step reports repeated (identical) frames. If the 4K60 mode
fails the check, use the highest mode that passes.

If the Akaso has no manual shutter, the robot tags smear when the robot moves fast in a dim
hallway. Then use the Akaso for the stopped snapshots, and use the GoPro alone (2 tags or more)
for the moving data. Add light to the test area if you can.

**Lens model.** The tool now uses the OpenCV model with 5 distortion coefficients. This is good for
the GoPro in Linear mode. A fisheye Akaso view (more than about 120°) needs the OpenCV fisheye
model. Add it to `camera.py` in phase 2 if the Akaso calibration error is more than 0.5 px with
the 5-coefficient model.

### Hallway layout

The hallway is a T junction. The hub is built on the outside corner where the two hallways meet.

- Tags 24 and 21 are on the face that looks across the **16 ft (4.88 m)** hallway.
- Tags 25 and 26 are on the face that looks across the **11 ft (3.35 m)** hallway (the one with
  the mural).

The test area is in front of **tag 24**, because only the 16 ft hallway has space for marks out
to 4 m. Set `Test/Settings/Tag ID` and `questnav/Target Tag ID` to 24. The robot code does not
need a change.

```text
                 far wall of the 16 ft hallway
 [camera L]                                          [camera R]
  2.3 m high,                                         2.3 m high,
  2.1 m left of                                       2.1 m right of
  the tag 24 line          test area                  the tag 24 line
                       (marks 1 m to 4 m
                        out from tag 24)              4.88 m (16 ft)
                                                      from the hub face
 ====================[21]=====[24]==+               +======
               hub                  |[25]   [26]    |
                                    |  11 ft hallway |
                                    |   (mural)      |
```

Seen from the test area, facing tag 24: tag 21 is on your left, the corner is on your right, and
tags 25 and 26 are around the corner on the right.

| Item | Where |
|---|---|
| Tag 24 | Center 0.248 m (9.8 in) from the corner, 1.124 m (44.25 in) high |
| Tag 21 | Same face, center 0.604 m (23.8 in) from the corner, 1.124 m high |
| Tag 25 | Other face, center 0.248 m (9.8 in) from the corner, 1.124 m high |
| Tag 26 | Other face, center 0.603 m (23.7 in) from the corner, 1.124 m high |
| Test area | In front of tags 24 and 21, from 1 m to 4 m out. At 4 m, the bumper is about 0.44 m from the far wall |
| Camera L | Against the far wall, 2.1 m (6.9 ft) left of the line straight out from tag 24. It sees 24 and 21 |
| Camera R | Against the far wall, 2.1 m (6.9 ft) right of the same line, near the mouth of the 11 ft hallway. It sees 24, 21, 25 and 26 |
| Floor tags 50 to 52 | Flat on the floor in the test area, for example at (1.7 m out, 0.8 m left), (1.7 m out, 0.8 m right) and (3.3 m out, on the line). See [Floor tags](#floor-tags) |

From the center of the test area (2.5 m out), the two cameras are 90° apart. The synthetic scene
in the tool tests uses this placement.

What the headset sees:

- Straight out from tag 24, the headset sees 24 and 21 only. The other face is edge-on. Two
  tags are enough for QuestNav to align to the field (Phase 1), but not for its later
  corrections, which need 3 tags.
- Toward the corner (to the right of the tag 24 line), the headset sees both faces: up to 4 tags.

### Floor tags

Floor tags are three extra AprilTags (IDs 50, 51 and 52) printed and taped **flat on the floor**
in the test area. Measure their positions one time per setup, from tag 24 with the laser. They do
not move during a session.

They are for the cameras only, not for QuestNav. The survey uses them to find the exact position
and tilt of each GoPro:

- The camera tilt (pitch) is the largest error source: 0.1° of pitch error moves truth about
  1.3 cm at 3 m.
- The hub tags are far from the cameras (about 5 m) and are all at one height, on one or two
  walls. Camera L sees only 24 and 21, which are on one flat face. That fixes the tilt badly.
- Floor tags are in the area where the robot drives, closer to the cameras, and on the floor
  plane that truth is calculated on. They fix the tilt well.

Without floor tags, the tool still works, but the truth error can be several centimeters. Leave
the floor tags in place for the whole session; the robot can drive over flat paper tags. If the
robot covers a tag in a frame, the survey uses the other frames.

### Camera placement

- Put each GoPro 2 m to 4 m from the center of the test area.
- Seen from the center of the test area, the two GoPros must be 60° to 120° apart. 90° is best.
- Point each GoPro down about 25° to 35°, so that the whole test area is in view.
- Each GoPro must also see at least 2 hub tags or 3 floor survey tags.
- The tripods must not block the headset's view of the hub.

## Session procedure

1. Set up the GoPros. Do not touch them again until the session ends.
2. Start recording on both GoPros.
3. Run the test routine. Each routine starts and ends with the sync wiggle.
4. Stop recording. One recording can contain several routines.
5. Copy the videos and the `.wpilog` to the laptop.
6. Run the tool.

## Robot code changes (small)

| Change | Why |
|---|---|
| `/QuestNavLog/Frame` gets `[capture time (s), x, y, heading (deg)]` for each tracking frame: the raw robot pose (no soft reset) | The log entry time is the loop time. The tool needs the Quest capture time |
| `/DriveState/RawHeading (deg)` gets the gyro heading at the odometry rate | The gyro heading for the sync |
| New `syncWiggle` command: hold 0.5 s, turn to +30°, −30°, +30°, −30°, 0° from the start heading, hold 0.5 s (about 7 s) | The sync event |
| `Test/Sync Wiggle Active` is true during each wiggle. The console gets `SYNC_WIGGLE,begin\|end,<label>,<time>` | The tool finds the wiggles in the log |
| Each test routine does the wiggle at its start and end, if `Test/Settings/Sync Wiggle` is true (default true) | Two sync events give the clock offset and the clock rate |
| New "0. Sync Wiggle" option in the "Test Routine" chooser | For a manual sync |

Nothing else on the robot changes.

## Laptop tool

Python 3.11 or later, in `tools/groundtruth/`. The laptop is a MacBook Air (macOS); see the
tool README for the install. Python, not Java, because it reads video easily (OpenCV), it
has a good least-squares solver (SciPy), and it reads and writes `.wpilog` files (`robotpy-wpiutil`).

| Package | Use |
|---|---|
| `opencv-python` | Video read, AprilTag 36h11 detection (`cv2.aruco`), ChArUco calibration |
| `numpy`, `scipy` | Math, `least_squares` |
| `robotpy-wpiutil` | Read and write `.wpilog` |
| `matplotlib` | Report plots |

### Steps

| Step | Script | Input | Output |
|---|---|---|---|
| 1. Lens calibration | `calibrate.py` | A video of the ChArUco board, moved around the view | `lens_<camera>.json`. Accept if the RMS error is 0.5 px or less |
| 2. Detection | `detect.py` | A test video | `corners_<camera>.npz`: frame time, tag ID and 4 corner pixels for each tag in each frame. This is the slow step (about 20 to 40 minutes for 5 minutes of 4K video). It runs one time for each video |
| 3. Survey | `survey.py` | Corners of the hub and floor tags, `setup.json` | `camera_<camera>.json`: the camera pose in the field frame, and the RMS error for each tag |
| 4. Single-camera truth | `solve.py` | Corners of the robot tags, lens, camera pose, robot tag mounts | Truth for each frame of each camera, in video time |
| 5. Sync | `sync.py` | Single-camera truth heading, `DriveState/RawHeading` from the log | For each camera: robot time = a + b × video time |
| 6. Joint truth | `solve.py --joint` | Both cameras, sync results, `DriveState/Speeds` from the log | Truth in robot time, from both cameras |
| 7. Compare | `compare.py` | Joint truth, the `.wpilog` | A report (HTML with plots and tables) and a new `.wpilog` with `GroundTruth/` entries |

`setup.json` has the measurements for one setup: the floor survey tag positions, the robot tag
mounts and heights, and the printed tag size.

### Solver

The unknowns are the robot pose p = (x, y, θ). For each robot tag corner seen by a camera:

1. Robot frame: the corner position comes from the tag mount and the tag size.
2. Field frame: transform it by p.
3. Camera frame: transform it by the inverse of the surveyed camera pose.
4. Image: project with the lens model.
5. Residual: the measured pixel minus the predicted pixel.

`scipy.optimize.least_squares` minimizes the residuals. The start value is the result of the last
frame. A frame result is **accepted** only if all of these are true:

- At least 2 tags, or 1 tag on each of 2 cameras.
- RMS residual of 1.0 px or less.
- Position σ from the covariance of 1.5 cm or less.

### Joint truth with a moving robot

The two GoPros do not record frames at the same time. For each frame of camera A, the tool:

1. Converts the frame time to robot time with the sync result.
2. Finds the nearest frame of camera B (8 ms or less away at 60 fps).
3. Moves the camera B observation to the time of the camera A frame. The motion comes from
   `DriveState/Speeds` in the log. Over 8 ms at 0.75 m/s the robot moves 6 mm, and this
   correction makes the remaining error less than 1 mm.
4. Solves with the corners of both cameras.

A GoPro reads the image one row at a time (rolling shutter). The robot fills only a small part of
the image height, so the time difference across the robot is about 1 ms. At 0.75 m/s this is
less than 1 mm, so the tool does not correct it.

### Sync

1. Get the heading from the single-camera truth, and the gyro heading from the log.
2. Calculate the turn rate (deg/s) of each one. The rate does not depend on where each heading
   starts.
3. For each wiggle, find the time shift that makes the two turn rates agree best
   (cross-correlation, then a fine fit). Expected precision: 2 to 5 ms.
4. Two wiggles (start and end) give a and b. b corrects the clock rate difference between the
   GoPro and the roboRIO (typically 20 to 50 parts per million, which is 12 to 30 ms over
   10 minutes).

The tool reports the sync residual. If it is more than 10 ms, the report shows a warning.

### Survey

The survey finds the pose of each GoPro in the field frame. It is the largest error source:

| Survey error | Truth error at 3 m (camera 1.5 m above the tags) |
|---|---|
| Camera pitch 0.1° | about 1.3 cm along the view |
| Camera yaw 0.1° | about 0.5 cm across the view |

The hub and floor tags do not move, so the survey uses the corners from all frames of the video
and fits the camera pose (6 values) to all of them. The robot sometimes covers a tag; that frame
then has fewer corners. The report shows the RMS error for each tag. A large error on one tag
means that the tag is not where the layout or `setup.json` says.

The floor survey tags are on the floor, in the area where the robot drives, so they fix the
camera pitch well. Measure their positions from the hub walls with the laser, one time per setup.

### Robot tag mounts

Measure each tag mount from the bumper frame (to 2 mm or better) and put the values in
`setup.json`. The sync wiggle also checks them: when the robot turns in place, each tag must
move on a circle around the same center. The report shows the circle radius and the angle for
each tag, compared with `setup.json`.

A tag height error δz gives a distance error of about δz × r / h. At r = 3 m and h = 1.5 m,
a 2 mm height error gives 4 mm. Measure the tag heights carefully.

### Comparison and report

For each accepted truth sample, the tool interpolates the Quest pose (from `QuestNav/Frame`, at
its own capture time) and the fused pose (`DriveState/Pose`) to the same robot time.

The report has:

- **Snapshots.** For each `Test/Snapshot` in the log: truth (average over the last 0.5 s
  before the snapshot), Quest error and fused error. This replaces the laser at the marks.
- **Over time.** Plots of the Quest and fused position error and heading error.
- **Statistics.** RMS, 95th percentile and maximum error, stopped and moving.
- **Quest latency.** The time shift of the Quest pose (−100 ms to +100 ms) that gives the
  smallest error during motion.
- **Quality.** Sync residual, survey RMS for each tag, and the percentage of accepted frames.

The new `.wpilog` has all entries of the robot log, plus `GroundTruth/Pose` (Pose2d struct),
`GroundTruth/Error/Quest XY (m)`, `GroundTruth/Error/Quest Heading (deg)` and
`GroundTruth/Error/Fused XY (m)`, in robot time. Open it in AdvantageScope to see truth, Quest
and fused poses on the same field view.

### Tests

The tool has tests with synthetic data. No video is necessary:

- Solver: project the corners of a known pose with known cameras. Recover the pose to 1 mm with
  no noise. With 0.3 px noise, the error must be inside 3σ.
- Survey: synthetic hub and floor tags. Recover a known camera pose.
- Sync: a synthetic gyro heading and a shifted, rate-changed copy. Recover a and b.
- Joint truth: two cameras with offset frame times and a moving robot.

On the robot side, `QuestNavTestRoutinesTest` checks that each routine starts and ends with the
wiggle, and that `QuestNav/Frame` and `DriveState/RawHeading` are published.

## Phases

| Phase | Work | Done when |
|---|---|---|
| 1 | Robot code changes. Tool: solver, survey, sync, synthetic tests | All tests pass |
| 2 | GoPro settings, lens calibration, tag printing and mounting, `setup.json` | Lens RMS 0.5 px or less. The detection finds all tags in a test video |
| 3 | Tool: detection, joint truth, report, `.wpilog` output | A full test gives a report |
| 4 | Check truth with the laser | Robot stopped at 4 laser-measured points: truth agrees with the laser to 1 cm |

After phase 4, use the laser only for spot checks.

## Phase 1 results (synthetic)

The tool tests use a synthetic hallway with the camera placement from this design (cameras 2.3 m
high, 90° apart). With 0.3 px corner noise:

| Case | Position error for one frame |
|---|---|
| Two cameras | 0.2 to 0.5 mm RMS |
| One camera that sees 2 tags | 0.3 to 0.4 mm RMS |
| One camera that sees 1 tag | Rejected (about 3 mm, and not checked by a second tag) |
| Camera survey position, 300 frames at 0.5 px | 0.1 to 0.4 mm |

Noise is not the main error source. The survey, the tag mounts and the lens calibration are (see
the error budget). Some robot poses near the edge of the view were seen by only one camera with
one tag. When the GoPros are in place, drive the robot through the test area and check that
both GoPros see it everywhere.

## Error budget (robot stopped, 3 m, after averaging)

| Source | Estimate |
|---|---|
| Corner noise (averaged over 0.5 s, 30 frames) | under 1 mm |
| Camera survey (pitch 0.03°, yaw 0.05°) | 3 to 4 mm |
| Robot tag mount and height | 2 to 4 mm |
| Lens calibration bias | 2 to 3 mm |
| Floor not flat, hub tag placement | 2 to 5 mm |
| **Total (root sum of squares)** | **about 6 to 8 mm** |

When the robot moves, add the sync error: 5 ms at 0.75 m/s is about 4 mm. These values are
estimates. Phase 4 measures the real value.

## Limits

- The cameras and QuestNav both use the hub tags. If a hub tag is not at its layout position,
  both systems have the same error, and the comparison does not show it. The comparison measures
  the robot relative to the hub, which is what an auto needs.
- Truth covers only the area that the GoPros see.
- If a tripod moves during a session, the survey for that video is wrong. The survey RMS error
  shows this.
- Results come after the test, not during it.

## Open questions

1. **Akaso manual settings.** Does the Akaso have a manual shutter, and a field of view setting
   (Wide, Medium, Narrow)? The model name, if you find it, answers both.
2. **QuestNav and unknown IDs.** Does the QuestNav beta ignore tag IDs that are not in the
   field layout (the floor tags)? Check this on the headset.
3. **Robot tag size.** Are the robot tags the same size as the hub tags (165.1 mm black
   square)? If not, the tool needs a separate size for the robot tags.
