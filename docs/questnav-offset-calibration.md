# QuestNav headset offset calibration

This procedure finds where the Quest headset is on the robot. The result goes into
`ROBOT_TO_QUEST` in `src/main/java/frc/robot/subsystems/QuestNavSubsystem.java`.

## How it works

The robot origin (0, 0) is the rotation center: the middle of the four swerve modules.
When the robot spins in place, the origin does not move, but the headset moves in an arc.
The calibration command spins the robot and calculates the headset position from that arc.

Each cycle of the command:

1. Sets the Quest pose to zero.
2. Spins the robot counter-clockwise at 0.5 rad/s for 3 seconds (about 86°).
3. Calculates one offset sample.
4. Adds the sample to a running average.

The command publishes the average to `questnav/Quest Calculated Offset to Robot Center`
as `[x, y]` in meters, in the robot frame (+X forward, +Y left).

## Before you start

- [ ] Look at the headset on the robot. Make sure that the yaw in `ROBOT_TO_QUEST` agrees
      with the direction the headset faces. The code now uses -90°: the headset faces the
      right side of the robot. The calibration does not find yaw. If the yaw is wrong, the
      result is rotated by the same angle.
- [ ] Do a yaw check. Put a straight tape line on the floor. Drive the robot forward
      2 m along the line. The heading of the Quest robot pose must agree with the
      direction of travel.
- [ ] Write down the current x and y values of `ROBOT_TO_QUEST`.
- [ ] Find an open area with a minimum of 1.5 m of clear floor around the robot.
- [ ] Cover the AprilTag, or turn the tag stand away from the robot. If the headset sees
      a tag during a spin, the pose can jump and give a bad sample.
- [ ] Turn on the headset and let it track for 1 minute before the first run.
- [ ] Open the dashboard and find `questnav/Quest Calculated Offset to Robot Center`.

## Procedure

1. Put the robot in the open area.
2. Enable the robot in teleop.
3. Push and hold the **right bumper**. The robot spins in short steps.
4. Hold the bumper for a minimum of 10 cycles (about 30 seconds).
5. Release the bumper. The robot stops.
6. Record the `[x, y]` value from the dashboard.
7. Turn the robot to a new start heading.
8. Do steps 3 to 7 again until you have 3 runs.

> **Note:** Each push of the bumper starts a new average. Do not release the bumper
> during a run.

## Calculate the new offset

1. Compare the 3 runs. If they are not within about 1 cm of each other, look at the
   [troubleshooting](#troubleshooting) section and do the runs again.
2. Calculate the average x and the average y of the 3 runs.
3. Calculate the new values:
   - new x = current `ROBOT_TO_QUEST` x + average x
   - new y = current `ROBOT_TO_QUEST` y + average y
4. Put the new x and y into `ROBOT_TO_QUEST`. Do not change z or the rotation.
5. Deploy the code.

## Verify the result

1. Do one more run with the new code.
2. Make sure that the result is about `[0, 0]` (within about 1 cm).
3. Measure from the robot center to the headset with a tape measure. The result must
   agree with your new values to within a few centimeters. The tracking point is inside
   the headset, so a tape measure cannot give an exact value.

> **Caution:** The calibration sets the Quest pose to zero, so the pose is not in field
> coordinates after a run. Restart the robot code before you do AprilTag distance tests.

## Record sheet

| Run | Start heading | Cycles | Tag covered | x (m) | y (m) | Notes |
|-----|---------------|--------|-------------|-------|-------|-------|
| 1   |               |        |             |       |       |       |
| 2   |               |        |             |       |       |       |
| 3   |               |        |             |       |       |       |
| Avg |               |        |             |       |       |       |
| Verify |            |        |             |       |       |       |

## Troubleshooting

- **The runs do not agree.** Look for wheel slip, or for a robot that moves across the
  floor during the spin. Look at the run in the `.wpilog` in AdvantageScope. The log shows
  each change of the average, so you can see which samples are bad.
- **The verify run is not near [0, 0].** Do the yaw check again. A wrong yaw gives an
  x/y result that is rotated.
- **The value does not change on the dashboard.** Make sure that the dashboard shows the
  Quest as connected and tracking (`QuestNav/Connected` and `QuestNav/Tracking`).
