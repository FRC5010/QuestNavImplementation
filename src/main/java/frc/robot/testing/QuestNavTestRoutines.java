package frc.robot.testing;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import com.ctre.phoenix6.swerve.SwerveRequest.TargetDirectionPerspectiveValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.QuestNavSubsystem;

/**
 * Repeatable QuestNav test routines. Run them in Test mode from the "Test Routine" chooser.
 *
 * <ul>
 *   <li>{@link #turnSweep()}: turns in place through a set of angles to the tag. The robot does
 *       not move, so any change in the reported position is error.</li>
 *   <li>{@link #driftLoop()}: drives loops with the headset facing away from the tag, then looks
 *       at the tag. The pose change across the look is the drift the tag corrected.</li>
 *   <li>{@link #stopAtMarks()}: drives to marks in front of the tag and pauses at each one so a
 *       person can measure the bumper with a laser.</li>
 *   <li>{@link #syncWiggle()}: turns left and right in place. The gyro log and the ground-truth
 *       video both see it, so the laptop tool can align the video clocks with the robot clock.
 *       Each routine also does it at its start and end, if the "Sync Wiggle" setting is on.</li>
 * </ul>
 *
 * <p>The robot steers with the fused drivetrain pose. Snapshots record the Quest pose and the
 * fused pose.
 */
public class QuestNavTestRoutines {
    public static final String TURN_SWEEP = "TurnSweep";
    public static final String DRIFT = "Drift";
    public static final String MARKS = "Marks";
    public static final String SYNC = "SyncWiggle";

    /** True on the dashboard while a sync wiggle runs. The ground-truth tool finds the wiggles with it. */
    public static final String SYNC_ACTIVE = "Test/Sync Wiggle Active";
    /** Number of sync wiggles finished since the robot code started. */
    public static final String SYNC_COUNT = "Test/Sync Wiggle Count";

    /** Headings of the sync wiggle, relative to the heading at its start. */
    static final double[] SYNC_OFFSETS_DEG = {30, -30, 30, -30, 0};
    private static final double SYNC_STILL_S = 0.5;

    /** Angles from "headset faces the tag", in the order the turn sweep visits them. */
    static final double[] TURN_OFFSETS_DEG = {0, 15, 30, 45, 60, 0, -15, -30, -45, -60, 0};

    /** Marks as {distance out from the tag, lateral (+ = right as you face the tag)} in meters. */
    static final double[][] MARKS_M = {{1, 0}, {2, 0}, {3, 0}, {4, 0}, {2, 0.5}, {2, -0.5}};

    private static final double HEADING_KP = 6.0;
    private static final double MAX_ROTATION_RAD_PER_S = 1.5;
    private static final double TRANSLATION_KP = 2.0;
    private static final double HEADING_TOLERANCE_DEG = 1.5;
    private static final double POSITION_TOLERANCE_M = 0.03;
    private static final double TURN_TIMEOUT_S = 6.0;
    private static final double DRIVE_TIMEOUT_S = 15.0;
    /** A drive target farther than this stops the routine: the pose is probably not in field coordinates. */
    static final double MAX_DRIVE_DISTANCE_M = 6.0;

    private final CommandSwerveDrivetrain drivetrain;
    private final QuestNavSubsystem questNav;
    private final TestSettings settings;
    private final TestSnapshotLogger logger;
    private final BooleanSupplier continueButton;

    private final SwerveRequest.FieldCentricFacingAngle facing = new SwerveRequest.FieldCentricFacingAngle()
            .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance)
            .withTargetDirectionPerspective(TargetDirectionPerspectiveValue.BlueAlliance)
            .withDriveRequestType(DriveRequestType.Velocity)
            .withHeadingPID(HEADING_KP, 0, 0)
            .withMaxAbsRotationalRate(MAX_ROTATION_RAD_PER_S);
    private final PIDController xController = new PIDController(TRANSLATION_KP, 0, 0);
    private final PIDController yController = new PIDController(TRANSLATION_KP, 0, 0);
    private boolean aborted = false;

    public QuestNavTestRoutines(
            CommandSwerveDrivetrain drivetrain,
            QuestNavSubsystem questNav,
            TestSettings settings,
            TestSnapshotLogger logger,
            BooleanSupplier continueButton) {
        this.drivetrain = drivetrain;
        this.questNav = questNav;
        this.settings = settings;
        this.logger = logger;
        this.continueButton = continueButton;
        facing.HeadingController.enableContinuousInput(-Math.PI, Math.PI);
    }

    public TestSnapshotLogger logger() {
        return logger;
    }

    // ---------------------------------------------------------------- routines

    /** Routine 1: turn in place through {@link #TURN_OFFSETS_DEG}, snapshot at each angle. */
    public Command turnSweep() {
        Rotation2d[] anchor = {Rotation2d.kZero};
        List<Command> steps = new ArrayList<>();
        steps.add(logger.startRun(TURN_SWEEP));
        steps.add(optionalSyncWiggle(TURN_SWEEP + " start"));
        steps.add(Commands.runOnce(() -> anchor[0] = headingFacingTag(drivetrain.getPose().getTranslation())));
        for (double offset : TURN_OFFSETS_DEG) {
            Supplier<Rotation2d> heading = () -> anchor[0].plus(Rotation2d.fromDegrees(offset));
            steps.add(turnTo(heading));
            steps.add(hold(heading, settings::settleSeconds));
            steps.add(logger.snapshot(String.format(Locale.US, "offset %+.0f deg", offset)));
        }
        steps.add(optionalSyncWiggle(TURN_SWEEP + " end"));
        steps.add(Commands.runOnce(this::reportTurnSweep));
        return withEnd(Commands.sequence(steps.toArray(Command[]::new)), TURN_SWEEP);
    }

    /** Routine 2: loops with the tag out of view, then a look at the tag. */
    public Command driftLoop() {
        Pose2d[] start = {Pose2d.kZero};
        Rotation2d[] away = {Rotation2d.kZero};
        Rotation2d[] toward = {Rotation2d.kZero};
        double[] loopStart = {0};
        Supplier<Rotation2d> awayHeading = () -> away[0];
        Supplier<Rotation2d> towardHeading = () -> toward[0];

        Command loops = Commands.repeatingSequence(
                driveTo(() -> squareCorner(start[0], away[0], 1, 1)),
                driveTo(() -> squareCorner(start[0], away[0], -1, 1)),
                driveTo(() -> squareCorner(start[0], away[0], -1, -1)),
                driveTo(() -> squareCorner(start[0], away[0], 1, -1)))
                .until(() -> Timer.getFPGATimestamp() - loopStart[0] >= settings.driftSeconds());

        return withEnd(Commands.sequence(
                logger.startRun(DRIFT),
                optionalSyncWiggle(DRIFT + " start"),
                Commands.runOnce(() -> {
                    start[0] = drivetrain.getPose();
                    toward[0] = headingFacingTag(start[0].getTranslation());
                    away[0] = toward[0].plus(Rotation2d.k180deg);
                }),
                turnTo(awayHeading),
                hold(awayHeading, settings::settleSeconds),
                logger.snapshot("start"),
                logger.status("Drift: driving loops with the tag out of view"),
                Commands.runOnce(() -> loopStart[0] = Timer.getFPGATimestamp()),
                loops,
                driveTo(() -> new Pose2d(start[0].getTranslation(), away[0])),
                hold(awayHeading, settings::settleSeconds),
                logger.snapshot("before tag"),
                logger.status("Drift: looking at the tag"),
                turnTo(towardHeading),
                hold(towardHeading, settings::tagViewSeconds),
                logger.snapshot("seeing tag"),
                turnTo(awayHeading),
                hold(awayHeading, settings::settleSeconds),
                logger.snapshot("after tag"),
                optionalSyncWiggle(DRIFT + " end"),
                Commands.runOnce(this::reportDrift)), DRIFT);
    }

    /** Routine 3: drive to each mark in {@link #MARKS_M}, snapshot, then wait for a person to measure. */
    public Command stopAtMarks() {
        List<Command> steps = new ArrayList<>();
        steps.add(logger.startRun(MARKS));
        steps.add(optionalSyncWiggle(MARKS + " start"));
        for (double[] mark : MARKS_M) {
            double distance = mark[0];
            double lateral = mark[1];
            String name = String.format(Locale.US, "mark %.1f m, lateral %+.1f m", distance, lateral);
            Pose2d[] goal = {Pose2d.kZero};
            Supplier<Rotation2d> heading = () -> goal[0].getRotation();
            steps.add(Commands.runOnce(() -> goal[0] = markPose(distance, lateral)));
            steps.add(logger.status("Marks: driving to " + name));
            steps.add(driveTo(() -> goal[0]));
            steps.add(hold(heading, settings::settleSeconds));
            steps.add(logger.snapshot(name));
            steps.add(logger.status("Marks: measure at " + name + ", then press Y or set Test/Continue"));
            steps.add(waitForContinue(heading));
        }
        steps.add(optionalSyncWiggle(MARKS + " end"));
        return withEnd(Commands.sequence(steps.toArray(Command[]::new)), MARKS);
    }

    /** Routine 0: one sync wiggle on its own. */
    public Command syncWiggle() {
        return withEnd(syncWiggleBody(SYNC), SYNC);
    }

    // ---------------------------------------------------------------- results

    private void reportTurnSweep() {
        List<TestSnapshotLogger.Snapshot> run = logger.currentRun();
        Optional<Translation2d> first = run.stream()
                .flatMap(s -> s.questPose().stream()).map(Pose2d::getTranslation).findFirst();
        if (first.isEmpty()) {
            logger.setStatus("TurnSweep: no Quest pose, no result");
            return;
        }
        double maxShift = run.stream()
                .flatMap(s -> s.questPose().stream())
                .mapToDouble(p -> p.getTranslation().getDistance(first.get()))
                .max().orElse(0);
        TestSnapshotLogger.Snapshot ref = run.stream().filter(s -> s.questPose().isPresent()).findFirst().get();
        double maxHeadingError = run.stream()
                .filter(s -> s.questPose().isPresent())
                .mapToDouble(s -> Math.abs(headingChangeVersusGyro(ref, s)))
                .max().orElse(0);
        SmartDashboard.putNumber("Test/Result/TurnSweep Max Shift (m)", maxShift);
        SmartDashboard.putNumber("Test/Result/TurnSweep Max Heading Error (deg)", maxHeadingError);
        logger.setStatus(String.format(Locale.US,
                "TurnSweep done: max position shift %.3f m, max heading error vs gyro %.2f deg",
                maxShift, maxHeadingError));
    }

    private void reportDrift() {
        Optional<TestSnapshotLogger.Snapshot> before = logger.find("before tag").filter(s -> s.questPose().isPresent());
        Optional<TestSnapshotLogger.Snapshot> after = logger.find("after tag").filter(s -> s.questPose().isPresent());
        if (before.isEmpty() || after.isEmpty()) {
            logger.setStatus("Drift: no Quest pose, no result");
            return;
        }
        double correction = before.get().questPose().get().getTranslation()
                .getDistance(after.get().questPose().get().getTranslation());
        // The robot does not return to exactly the same heading, so compare with the gyro.
        double headingCorrection = headingChangeVersusGyro(before.get(), after.get());
        SmartDashboard.putNumber("Test/Result/Drift Correction (m)", correction);
        SmartDashboard.putNumber("Test/Result/Drift Heading Correction (deg)", headingCorrection);
        logger.setStatus(String.format(Locale.US,
                "Drift done: tag corrected %.3f m and %.1f deg", correction, headingCorrection));
    }

    /**
     * How much more the Quest heading changed than the gyro heading between two snapshots (deg).
     * The gyro does not drift much over one test, so this is the Quest heading error.
     */
    private static double headingChangeVersusGyro(TestSnapshotLogger.Snapshot a, TestSnapshotLogger.Snapshot b) {
        Rotation2d quest = b.questPose().get().getRotation().minus(a.questPose().get().getRotation());
        Rotation2d gyro = b.gyroHeading().minus(a.gyroHeading());
        return quest.minus(gyro).getDegrees();
    }

    // ---------------------------------------------------------------- building blocks

    /** Field heading that points the headset (per ROBOT_TO_QUEST) at the tag from this point. */
    Rotation2d headingFacingTag(Translation2d from) {
        Translation2d tag = tagPose().getTranslation();
        Rotation2d bearing = tag.minus(from).getAngle();
        Rotation2d headsetYaw = questNav.getRobotToQuest().getRotation().toRotation2d();
        return bearing.minus(headsetYaw);
    }

    /** Field pose for a mark, with the headset (or the front, per the setting) facing the tag. */
    Pose2d markPose(double distance, double lateral) {
        Pose2d tag = tagPose();
        Translation2d point = TagFrame.fieldPoint(tag, distance, lateral, Rotation2d.kZero).getTranslation();
        Rotation2d heading = settings.marksFrontFacesTag()
                ? tag.getTranslation().minus(point).getAngle()
                : headingFacingTag(point);
        return new Pose2d(point, heading);
    }

    private Pose2d tagPose() {
        return TagFrame.tagPose(settings.tagId()).orElse(Pose2d.kZero);
    }

    /** A corner of the drift square, centered on the start point and aligned with the field axes. */
    private Pose2d squareCorner(Pose2d center, Rotation2d heading, int signX, int signY) {
        double half = settings.driftSquareMeters() / 2;
        return new Pose2d(center.getTranslation().plus(new Translation2d(signX * half, signY * half)), heading);
    }

    /** A sync wiggle if the "Sync Wiggle" setting is on, else nothing. */
    private Command optionalSyncWiggle(String label) {
        return Commands.either(syncWiggleBody(label), Commands.none(), settings::syncWiggle);
    }

    /**
     * Holds still, turns to each of {@link #SYNC_OFFSETS_DEG} from the start heading, and holds
     * still again. {@link #SYNC_ACTIVE} is true for the whole wiggle, and the console gets a
     * {@code SYNC_WIGGLE,begin|end,<label>,<FPGA time>} line at each end.
     */
    private Command syncWiggleBody(String label) {
        Rotation2d[] anchor = {Rotation2d.kZero};
        Supplier<Rotation2d> start = () -> anchor[0];
        List<Command> steps = new ArrayList<>();
        steps.add(Commands.runOnce(() -> {
            anchor[0] = drivetrain.getPose().getRotation();
            SmartDashboard.putBoolean(SYNC_ACTIVE, true);
            System.out.printf(Locale.US, "SYNC_WIGGLE,begin,%s,%.6f%n", label, Timer.getFPGATimestamp());
        }));
        steps.add(logger.status("Sync wiggle: " + label));
        steps.add(hold(start, () -> SYNC_STILL_S));
        for (double offset : SYNC_OFFSETS_DEG) {
            steps.add(turnTo(() -> anchor[0].plus(Rotation2d.fromDegrees(offset))));
        }
        steps.add(hold(start, () -> SYNC_STILL_S));
        return Commands.sequence(steps.toArray(Command[]::new))
                .finallyDo(interrupted -> {
                    SmartDashboard.putBoolean(SYNC_ACTIVE, false);
                    System.out.printf(Locale.US, "SYNC_WIGGLE,end,%s,%.6f%n", label, Timer.getFPGATimestamp());
                    if (!interrupted) {
                        SmartDashboard.putNumber(SYNC_COUNT, SmartDashboard.getNumber(SYNC_COUNT, 0) + 1);
                    }
                });
    }

    /** Turns in place to a field heading. Ends when on target and nearly stopped. */
    private Command turnTo(Supplier<Rotation2d> heading) {
        Rotation2d[] target = {Rotation2d.kZero};
        return drivetrain.applyRequest(() -> facing.withVelocityX(0).withVelocityY(0).withTargetDirection(target[0]))
                .beforeStarting(() -> target[0] = heading.get())
                .until(() -> Math.abs(drivetrain.getPose().getRotation().minus(target[0]).getDegrees()) < HEADING_TOLERANCE_DEG
                        && Math.abs(drivetrain.getRobotRelativeSpeeds().omegaRadiansPerSecond) < 0.05)
                .withTimeout(TURN_TIMEOUT_S);
    }

    /** Holds still at a field heading for a number of seconds. */
    private Command hold(Supplier<Rotation2d> heading, Supplier<Double> seconds) {
        double[] end = {0};
        Rotation2d[] target = {Rotation2d.kZero};
        return drivetrain.applyRequest(() -> facing.withVelocityX(0).withVelocityY(0).withTargetDirection(target[0]))
                .beforeStarting(() -> {
                    target[0] = heading.get();
                    end[0] = Timer.getFPGATimestamp() + seconds.get();
                })
                .until(() -> Timer.getFPGATimestamp() >= end[0]);
    }

    /** Drives to a field pose with the fused pose. Ends when within tolerance. */
    private Command driveTo(Supplier<Pose2d> target) {
        Pose2d[] goal = {Pose2d.kZero};
        return drivetrain.applyRequest(() -> {
            if (aborted) {
                return facing.withVelocityX(0).withVelocityY(0).withTargetDirection(drivetrain.getPose().getRotation());
            }
            Pose2d pose = drivetrain.getPose();
            double vx = xController.calculate(pose.getX(), goal[0].getX());
            double vy = yController.calculate(pose.getY(), goal[0].getY());
            double speed = Math.hypot(vx, vy);
            double max = settings.maxSpeedMps();
            if (speed > max) {
                vx *= max / speed;
                vy *= max / speed;
            }
            return facing.withVelocityX(vx).withVelocityY(vy).withTargetDirection(goal[0].getRotation());
        })
                .beforeStarting(() -> {
                    goal[0] = target.get();
                    xController.reset();
                    yController.reset();
                    double distance = drivetrain.getPose().getTranslation().getDistance(goal[0].getTranslation());
                    if (distance > MAX_DRIVE_DISTANCE_M) {
                        aborted = true;
                        logger.setStatus(String.format(Locale.US,
                                "Stopped: the next target is %.1f m away. Make sure the robot pose is in field "
                                        + "coordinates (the headset has seen the tag) and the robot is near the tag.",
                                distance));
                    }
                })
                .until(() -> {
                    if (aborted) {
                        return true;
                    }
                    Pose2d pose = drivetrain.getPose();
                    return pose.getTranslation().getDistance(goal[0].getTranslation()) < POSITION_TOLERANCE_M
                            && Math.abs(pose.getRotation().minus(goal[0].getRotation()).getDegrees()) < HEADING_TOLERANCE_DEG;
                })
                .withTimeout(DRIVE_TIMEOUT_S);
    }

    /** Holds still until Y is pressed (a new press, not a held button) or Test/Continue is set to true. */
    private Command waitForContinue(Supplier<Rotation2d> heading) {
        boolean[] released = {false};
        Rotation2d[] target = {Rotation2d.kZero};
        return drivetrain.applyRequest(() -> facing.withVelocityX(0).withVelocityY(0).withTargetDirection(target[0]))
                .beforeStarting(() -> {
                    target[0] = heading.get();
                    released[0] = false;
                    settings.clearContinue();
                })
                .until(() -> {
                    boolean y = continueButton.getAsBoolean();
                    if (!y) {
                        released[0] = true;
                    }
                    return settings.continueRequested() || (released[0] && y);
                })
                .finallyDo(settings::clearContinue);
    }

    private Command withEnd(Command routine, String name) {
        return routine
                .until(() -> aborted)
                .beforeStarting(() -> aborted = false)
                .finallyDo(interrupted -> {
                    if (interrupted && !aborted) {
                        logger.setStatus(name + " stopped before the end");
                    }
                })
                .withName(name);
    }
}
