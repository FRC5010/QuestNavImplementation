package frc.robot.testing;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.RobotContainer;
import frc.robot.Telemetry;
import frc.robot.subsystems.QuestNavSubsystem;
import frc.robot.subsystems.QuestNavSim;

/**
 * Runs each QuestNav test routine on the simulated robot and headset (HAL + CTRE simulation,
 * real time). Settle and drift times are shortened so the whole class takes about a minute.
 */
class QuestNavTestRoutinesTest {
    private RobotContainer container;
    private QuestNavSim sim;
    private QuestNavTestRoutines routines;
    private Pose2d tag;

    @BeforeEach
    void setUp() throws InterruptedException {
        assertTrue(HAL.initialize(500, 0));
        DriverStationSim.resetData();
        DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
        container = new RobotContainer();
        routines = container.testRoutines;
        sim = container.questNavSubsystem.getSim().orElseThrow();
        sim.setLatencyMs(0);
        sim.setNoise(0, 0);
        sim.setTracking(true);
        long deadline = System.nanoTime() + 5_000_000_000L;
        while (!sim.isHeadsetConnected() && System.nanoTime() < deadline) {
            Thread.sleep(5);
        }
        assertTrue(sim.isHeadsetConnected(), "fake headset should connect to the NT server");

        SmartDashboard.putNumber(TestSettings.SETTLE_SECONDS, 0.3);
        SmartDashboard.putNumber(TestSettings.TAG_VIEW_SECONDS, 0.3);
        SmartDashboard.putNumber(TestSettings.DRIFT_SECONDS, 2.0);
        SmartDashboard.putNumber(TestSettings.DRIFT_SQUARE, 0.5);
        SmartDashboard.putNumber(TestSettings.MAX_SPEED, 1.5);
        SmartDashboard.putNumber(TestSettings.TAG_ID, 25);
        SmartDashboard.putNumber(TestSettings.BUMPER_LENGTH, 0.88);
        SmartDashboard.putNumber(TestSettings.BUMPER_WIDTH, 0.88);
        SmartDashboard.putBoolean(TestSettings.MARKS_FRONT_FACES_TAG, false);
        SmartDashboard.putBoolean(TestSettings.SYNC_WIGGLE, false);
        tag = TagFrame.tagPose(25).orElseThrow();

        DriverStationSim.setAutonomous(false);
        DriverStationSim.setTest(true);
        DriverStationSim.setEnabled(true);
        DriverStationSim.notifyNewData();
    }

    @AfterEach
    void tearDown() {
        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().unregisterAllSubsystems();
        CommandScheduler.getInstance().clearComposedCommands();
        sim.close();
        container.drivetrain.close();
        DriverStationSim.setEnabled(false);
        DriverStationSim.setTest(false);
        DriverStationSim.notifyNewData();
    }

    /** Puts the simulated headset exactly where the code thinks it is. */
    private void useCorrectMount() {
        sim.setMountTransform(container.questNavSubsystem.getRobotToQuest());
    }

    private void startInFrontOfTag(double distance) throws InterruptedException {
        container.drivetrain.resetPose(new Pose2d(tag.getX() - distance, tag.getY(), Rotation2d.kZero));
        runLoops(0.5);
    }

    private void runLoops(double seconds) throws InterruptedException {
        long end = System.nanoTime() + (long) (seconds * 1e9);
        while (System.nanoTime() < end) {
            CommandScheduler.getInstance().run();
            Thread.sleep(20);
        }
    }

    private void runToEnd(Command command, double timeoutSeconds, boolean pressContinue) throws InterruptedException {
        CommandScheduler.getInstance().schedule(command);
        long end = System.nanoTime() + (long) (timeoutSeconds * 1e9);
        while (CommandScheduler.getInstance().isScheduled(command) && System.nanoTime() < end) {
            if (pressContinue) {
                SmartDashboard.putBoolean(TestSettings.CONTINUE, true);
            }
            CommandScheduler.getInstance().run();
            Thread.sleep(20);
        }
        assertFalse(CommandScheduler.getInstance().isScheduled(command), command.getName() + " did not finish in time");
    }

    @Test
    void turnSweepWithACorrectMountReportsNoShift() throws InterruptedException {
        useCorrectMount();
        startInFrontOfTag(2.0);
        // Also covers the optional sync wiggles at the start and end of a routine.
        SmartDashboard.putBoolean(TestSettings.SYNC_WIGGLE, true);
        double wigglesBefore = SmartDashboard.getNumber(QuestNavTestRoutines.SYNC_COUNT, 0);

        runToEnd(routines.turnSweep(), 90, false);

        assertEquals(wigglesBefore + 2, SmartDashboard.getNumber(QuestNavTestRoutines.SYNC_COUNT, 0),
                "the routine should start and end with a sync wiggle");

        List<TestSnapshotLogger.Snapshot> run = routines.logger().currentRun();
        assertEquals(QuestNavTestRoutines.TURN_OFFSETS_DEG.length, run.size());
        double shift = SmartDashboard.getNumber("Test/Result/TurnSweep Max Shift (m)", -1);
        assertTrue(shift >= 0 && shift < 0.03, "shift was " + shift);
        double headingError = SmartDashboard.getNumber("Test/Result/TurnSweep Max Heading Error (deg)", -1);
        assertTrue(headingError >= 0 && headingError < 1.0, "heading error was " + headingError);
        // At offset 0 the headset faces the tag, so the right side of the robot faces it.
        assertEquals("Right", run.get(0).tagFrame().orElseThrow().facingSide());
    }

    @Test
    void turnSweepShowsAWrongMount() throws InterruptedException {
        // The simulated headset is mounted 0.3 m from where the code assumes.
        sim.setMountTransform(container.questNavSubsystem.getRobotToQuest()
                .plus(new Transform3d(0.3, 0, 0, Rotation3d.kZero)));
        startInFrontOfTag(2.0);

        runToEnd(routines.turnSweep(), 60, false);

        double shift = SmartDashboard.getNumber("Test/Result/TurnSweep Max Shift (m)", -1);
        assertTrue(shift > 0.2, "a 0.3 m mount error should show as a large shift, was " + shift);
    }

    @Test
    void driftLoopReturnsAndReportsTheCorrection() throws InterruptedException {
        useCorrectMount();
        startInFrontOfTag(2.0);

        runToEnd(routines.driftLoop(), 60, false);

        TestSnapshotLogger logger = routines.logger();
        for (String label : List.of("start", "before tag", "seeing tag", "after tag")) {
            assertTrue(logger.find(label).isPresent(), "missing snapshot " + label);
        }
        Pose2d start = logger.find("start").orElseThrow().fusedPose();
        Pose2d before = logger.find("before tag").orElseThrow().fusedPose();
        assertEquals(0.0, start.getTranslation().getDistance(before.getTranslation()), 0.05,
                "the robot should come back to its start point");
        // The simulated headset has no drift, so there is nothing for the tag to correct.
        double correction = SmartDashboard.getNumber("Test/Result/Drift Correction (m)", -1);
        assertTrue(correction >= 0 && correction < 0.03, "correction was " + correction);
        double headingCorrection = SmartDashboard.getNumber("Test/Result/Drift Heading Correction (deg)", 99);
        assertEquals(0.0, headingCorrection, 1.0);
    }

    @Test
    void marksStopWhenTheTargetIsFarAway() throws InterruptedException {
        // The robot is at the field origin, about 5.6 m from the tag; mark 1 is more than 6 m away.
        useCorrectMount();
        container.drivetrain.resetPose(new Pose2d(-3.0, 0.0, Rotation2d.kZero));
        runLoops(0.5);

        runToEnd(routines.stopAtMarks(), 5, true);

        assertEquals(0, routines.logger().currentRun().size(), "no mark should be reached");
        Pose2d pose = container.drivetrain.getPose();
        assertEquals(-3.0, pose.getX(), 0.05, "the robot should not drive toward the far target");
    }

    @Test
    void stopAtMarksSnapshotsEachMark() throws InterruptedException {
        useCorrectMount();
        startInFrontOfTag(1.5);

        runToEnd(routines.stopAtMarks(), 90, true);

        List<TestSnapshotLogger.Snapshot> run = routines.logger().currentRun();
        assertEquals(QuestNavTestRoutines.MARKS_M.length, run.size());
        for (int i = 0; i < run.size(); i++) {
            double[] mark = QuestNavTestRoutines.MARKS_M[i];
            TagFrame.Reading r = run.get(i).tagFrame().orElseThrow();
            assertEquals(mark[0], r.centerDistance(), 0.05, "distance at mark " + i);
            assertEquals(mark[1], r.centerLateral(), 0.05, "lateral at mark " + i);
            assertEquals("Right", r.facingSide(), "headset (right side) should face the tag at mark " + i);
            if (mark[1] == 0) {
                // On the centerline the headset direction is square to the tag. At side marks the
                // robot turns so the headset points at the tag, and the corners show that angle.
                assertEquals(r.leftCorner(), r.rightCorner(), 0.03, "bumper should be square to the tag at mark " + i);
            }
        }
    }

    @Test
    void syncWiggleTurnsBothWaysAndLogsTheGroundTruthTopics() throws InterruptedException {
        useCorrectMount();
        startInFrontOfTag(2.0);
        NetworkTableInstance nt = NetworkTableInstance.getDefault();
        DoubleArraySubscriber frames = nt.getDoubleArrayTopic(QuestNavSubsystem.FRAME_TOPIC).subscribe(new double[0]);
        DoubleSubscriber rawHeading = nt.getDoubleTopic(Telemetry.RAW_HEADING_TOPIC).subscribe(Double.NaN);
        double startDeg = container.drivetrain.getPose().getRotation().getDegrees();
        double wigglesBefore = SmartDashboard.getNumber(QuestNavTestRoutines.SYNC_COUNT, 0);

        Command wiggle = routines.syncWiggle();
        CommandScheduler.getInstance().schedule(wiggle);
        double min = 0;
        double max = 0;
        boolean sawActive = false;
        long end = System.nanoTime() + 30_000_000_000L;
        while (CommandScheduler.getInstance().isScheduled(wiggle) && System.nanoTime() < end) {
            CommandScheduler.getInstance().run();
            double turned = container.drivetrain.getPose().getRotation().minus(Rotation2d.fromDegrees(startDeg)).getDegrees();
            min = Math.min(min, turned);
            max = Math.max(max, turned);
            sawActive |= SmartDashboard.getBoolean(QuestNavTestRoutines.SYNC_ACTIVE, false);
            Thread.sleep(20);
        }

        assertFalse(CommandScheduler.getInstance().isScheduled(wiggle), "the wiggle did not finish in time");
        assertTrue(sawActive, "Sync Wiggle Active should be true during the wiggle");
        assertFalse(SmartDashboard.getBoolean(QuestNavTestRoutines.SYNC_ACTIVE, true), "and false after it");
        assertEquals(wigglesBefore + 1, SmartDashboard.getNumber(QuestNavTestRoutines.SYNC_COUNT, 0));
        assertEquals(30, max, 3, "should turn about +30 deg");
        assertEquals(-30, min, 3, "should turn about -30 deg");
        double endTurn = container.drivetrain.getPose().getRotation().minus(Rotation2d.fromDegrees(startDeg)).getDegrees();
        assertEquals(0, endTurn, 2, "should end at the start heading");

        double[] frame = frames.get();
        assertEquals(4, frame.length, "Quest frames should be logged as [t, x, y, deg]");
        Pose2d pose = container.drivetrain.getPose();
        assertEquals(pose.getX(), frame[1], 0.05);
        assertEquals(pose.getY(), frame[2], 0.05);
        assertTrue(Double.isFinite(rawHeading.get()), "the raw gyro heading should be published");
        frames.close();
        rawHeading.close();
    }
}
