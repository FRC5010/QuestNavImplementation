# QuestNav test routines

The robot code has three test routines for QuestNav. They run in **Test** mode. Each routine
drives the robot the same way every time and records labeled snapshots of the pose.

| Routine | What it measures | Measuring needed |
|---|---|---|
| 1. Turn Sweep | Position and heading error when the robot turns in place | No |
| 2. Drift Loop | Drift with the tag out of view, and the correction when the tag comes back | No |
| 3. Stop At Marks | Accuracy at known points in front of the tag | Yes, a laser or tape at each mark |

Do the [floor setup](test-setup.md) and the [offset calibration](questnav-offset-calibration.md)
before you use these routines.

## Safety

> **Warning:** In Test mode the robot moves without driver input. Keep people out of the test
> area. To stop the robot, disable it on the Driver Station.

- The routines limit the speed to `Test/Settings/Max Speed (mps)` (0.75 m/s by default).
- A routine stops if the next drive target is more than 6 m away. This usually means that the
  robot pose is not in field coordinates. `Test/Status` shows the reason.

## Before each session

1. Turn on the headset and let it track for 1 minute.
2. Put the robot 1.5 m to 2 m in front of the tag.
3. Let the headset see the tag. This puts the robot pose in field coordinates.
4. Make sure that `questnav/Distance To Target Tag` agrees approximately with a tape measure.
5. Measure the outside length and width of the robot, with bumpers.
6. Set `Test/Settings/Bumper Length (m)` and `Test/Settings/Bumper Width (m)`.

## Run a routine

1. Select the routine in the `Test Routine` chooser on the dashboard.
2. Select **Test** mode on the Driver Station.
3. Enable the robot. The routine starts.
4. Watch `Test/Status` for the current step and the result.
5. When the routine is done, disable the robot.

To run the routine again, disable and enable the robot again. Each run gets the next run number.

## Routine 1: Turn Sweep

The robot stays on one point and turns to these angles: 0°, 15°, 30°, 45°, 60°, 0°, −15°,
−30°, −45°, −60°, 0°. 0° is the heading where the headset points at the tag. At each angle, the
robot stops, waits `Settle Seconds`, and records a snapshot.

The center of the robot does not move. Thus a change in the reported position is error.

**Results:**

- `Test/Result/TurnSweep Max Shift (m)`: the largest change of the Quest position from the
  first snapshot.
- `Test/Result/TurnSweep Max Heading Error (deg)`: the largest difference between the Quest
  heading change and the gyro heading change.

**How to read the results:**

- If the shift increases with the angle, the headset offset (`ROBOT_TO_QUEST`) is not correct.
  Do the offset calibration again.
- If the shift changes only at large angles, the headset possibly cannot see the tag at those
  angles.

## Routine 2: Drift Loop

1. The robot turns so that the headset points away from the tag.
2. The robot drives a square around its start point for `Drift Seconds`. The headset continues
   to point away from the tag.
3. The robot goes back to the start point and records the `before tag` snapshot.
4. The robot turns to the tag and waits `Tag View Seconds`.
5. The robot turns away again and records the `after tag` snapshot.

During steps 4 and 5 the robot only turns in place. Thus the change between `before tag` and
`after tag` is the drift that the tag corrected.

**Results:**

- `Test/Result/Drift Correction (m)`: the position correction.
- `Test/Result/Drift Heading Correction (deg)`: the heading correction, compared with the gyro.

**Settings:**

| Setting | Default | Notes |
|---|---|---|
| `Drift Seconds` | 30 | Use 30, 60 and 150 (the length of a match) |
| `Drift Square (m)` | 1.0 | Side length of the square. Keep it smaller than the hallway |
| `Tag View Seconds` | 3.0 | Time that the headset looks at the tag |

## Routine 3: Stop At Marks

The robot drives to each mark, stops, records a snapshot, and then waits for you.

| Mark | Distance out from the tag | Lateral |
|---|---|---|
| 1 | 1.0 m | 0 |
| 2 | 2.0 m | 0 |
| 3 | 3.0 m | 0 |
| 4 | 4.0 m | 0 |
| 5 | 2.0 m | 0.5 m right |
| 6 | 2.0 m | 0.5 m left |

"Right" and "left" are as you stand in front of the tag and look at it.

At each mark:

1. Wait until `Test/Status` says "measure".
2. Read `Test/Last Snapshot`. It gives the side of the bumper that faces the tag, and the
   predicted distances.
3. Measure the distance from the tag plane to the left corner of that bumper side.
4. Measure the distance from the tag plane to the right corner of that bumper side.
5. Measure the lateral distance from the centerline to the middle of that bumper side.
6. Write the measured values next to the predicted values.
7. Press **Y** on the controller, or set `Test/Continue` to true. The robot drives to the next
   mark.

By default the headset points at the tag at each mark. The headset faces the right side of the
robot, so the **right** bumper faces the tag. At marks 5 and 6 the robot is at an angle to the
tag, so the two corner distances are different. To make the front bumper face the tag, set
`Test/Settings/Marks: Front Faces Tag` to true. Then the headset cannot see the tag.

## Where the data goes

Each snapshot goes to three places:

- **Dashboard:** `Test/Last Snapshot` shows the latest snapshot as text.
- **NetworkTables:** `Test/Snapshot/` has each value. The `.wpilog` records these values, so you
  can see them in AdvantageScope.
- **Console:** one CSV line that starts with `SNAPSHOT,`. The first line is
  `SNAPSHOT_HEADER,` with the column names. The `.wpilog` also records the console.

Snapshot values:

| Value | Meaning |
|---|---|
| QuestPose | Robot pose from the headset (no soft reset) |
| FusedPose | Robot pose from the drivetrain estimator (wheels, gyro and Quest) |
| Gyro Heading | Raw gyro heading |
| Center Distance / Center Lateral | Robot center in the tag frame |
| Heading To Tag | 0° when the front of the robot points at the tag |
| Facing Side | Bumper side nearest to the tag: Front, Left, Back or Right |
| Left / Right Corner Distance | Tag plane to each corner of the facing side |
| Side Lateral | Centerline to the middle of the facing side |

QuestPose and FusedPose are in the 2026 field frame, because the robot code is on WPILib 2026.
In the 2026 frame, the origin is the blue alliance corner and +X points toward the red wall. The
WPILib 2027 frame has its origin at the center of the field and +X toward the blue wall. To change
a 2026 pose to a 2027 pose:

- x<sub>2027</sub> = 8.271 − x<sub>2026</sub>
- y<sub>2027</sub> = 4.035 − y<sub>2026</sub>
- θ<sub>2027</sub> = θ<sub>2026</sub> + 180°

Example: tag 25 is at (4.02, 4.39), facing 180°, in the 2026 frame, and at (4.25, −0.36), facing
0°, in the 2027 frame. The other snapshot values and the test results do not depend on the field
origin.

The tag frame uses the floor mark below the tag as its origin. Distance is straight out from the
tag face. Lateral is positive to your right as you face the tag. All tag-frame values come from
the Quest pose.

## All settings

All settings are under `Test/Settings/` on the dashboard.

| Setting | Default | Notes |
|---|---|---|
| `Tag ID` | 25 | The tag that the routines use |
| `Max Speed (mps)` | 0.75 | Maximum drive speed |
| `Settle Seconds` | 2.0 | Wait time before each snapshot |
| `Drift Seconds` | 30 | Routine 2 only |
| `Drift Square (m)` | 1.0 | Routine 2 only |
| `Tag View Seconds` | 3.0 | Routine 2 only |
| `Bumper Length (m)` | 0.88 | Outside length with bumpers. Measure your robot |
| `Bumper Width (m)` | 0.88 | Outside width with bumpers. Measure your robot |
| `Marks: Front Faces Tag` | false | Routine 3 only |
