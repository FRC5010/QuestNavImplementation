# Quick run: test routines

Do [Quick setup](quick-setup.md) first.

> **Warning:** In Test mode the robot moves without driver input. Keep people out of the test
> area. To stop the robot, disable it on the driver station.

## Run a routine

1. Select the routine in the **Test Routine** chooser on the dashboard.
2. On the driver station, select **Test** mode.
3. Enable the robot. The routine starts.
4. Watch `Test/Status`. It shows the current step and the result.
5. When `Test/Status` says "done", disable the robot.

To run the same routine again, disable and enable again.

If a camera records video: start the recording before you enable. Do not touch the camera until
you stop the recording.

## The routines

| Routine | Time | What you do | Result on the dashboard |
|---|---|---|---|
| 0. Sync Wiggle | 7 s | Nothing | Nothing. It aligns video with the robot log |
| 1. Turn Sweep | 1 to 2 min | Nothing | `Test/Result/TurnSweep Max Shift (m)` and `TurnSweep Max Heading Error (deg)` |
| 2. Drift Loop | About 1 min with the default 30 s | Nothing | `Test/Result/Drift Correction (m)` and `Drift Heading Correction (deg)` |
| 3. Stop At Marks | 5 to 10 min | Measure at each mark | One snapshot for each mark |

Start each routine with the robot about 2 m in front of tag 24.

### 1. Turn Sweep

The robot turns in place to 0°, ±15°, ±30°, ±45° and ±60° and takes a snapshot at each angle.

- Good: `Max Shift` is less than 0.03 m.
- If the shift gets larger with the angle, do the offset calibration again (hold the right
  bumper on the controller).

### 2. Drift Loop

The robot turns the headset away from the tags, drives a square for 30 s, comes back, looks at
the tags, and turns away again.

- `Drift Correction` is how far the headset drifted while it could not see the tags.
- For a longer test, set `Test/Settings/Drift Seconds` to 60 or 150.

### 3. Stop At Marks

The robot drives to 6 marks in front of tag 24 and stops at each one:
1 m, 2 m, 3 m and 4 m straight out, then 2 m out at 0.5 m right and 0.5 m left.

At each mark:

1. Wait until `Test/Status` says "measure".
2. Read `Test/Last Snapshot`. It shows which bumper side faces the tag and the expected distances.
3. With the laser, measure from the wall to the left corner and the right corner of that bumper
   side.
4. Write the measured and the expected values on the record sheet.
5. Press **Y** on the controller. The robot drives to the next mark.

## After the session

1. Disable the robot.
2. Get the log. In AdvantageScope: **File → Download Logs**, from `10.50.10.2`. The log is also
   on the robot USB stick in `/logs`, if one is plugged in.
3. Get the videos from the cameras, if you recorded.
4. Put the headset on the charger.

## Problems

| `Test/Status` or symptom | Fix |
|---|---|
| "Stopped: the next target is ... m away" | The robot pose is not in field coordinates. Do [step 7 of Quick setup](quick-setup.md#7-field-alignment) again |
| The robot does not move | Make sure that the driver station is in **Test** mode, not Teleop, and that a routine is selected |
| The robot drives toward a wall | Disable at once. Check `Test/Settings/Tag ID` (24) and the field alignment |
| Stop At Marks does not continue | Press **Y** again (a new press), or set `Test/Continue` to true |
| A routine stops before the end | `Test/Status` gives the reason. Disable, fix, and enable again |
| AdvantageScope cannot connect while the iPhone is connected | Unplug the iPhone for the download, or use Ethernet for the internet |

## Settings

All under `Test/Settings/`. The values reset when the robot code restarts.

| Setting | Default | Change it when |
|---|---|---|
| `Tag ID` | 24 | Only if the test area moves |
| `Max Speed (mps)` | 0.75 | Slower for the first runs |
| `Settle Seconds` | 2.0 | Rarely |
| `Drift Seconds` | 30 | 60 or 150 for longer drift tests |
| `Drift Square (m)` | 1.0 | Smaller if there are people or objects near |
| `Bumper Length (m)`, `Bumper Width (m)` | 0.88 | Set to the measured robot size, each session |
| `Sync Wiggle` | true | false if no camera records |
