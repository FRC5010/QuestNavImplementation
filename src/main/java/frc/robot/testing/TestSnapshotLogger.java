package frc.robot.testing;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

/**
 * Records labeled pose snapshots during test routines.
 *
 * <p>Each snapshot goes to NetworkTables under {@code Test/Snapshot/} (so the .wpilog records it)
 * and to the console as one CSV line that starts with {@code SNAPSHOT,} (the .wpilog records the
 * console too). {@code Test/Last Snapshot} shows the latest one in readable form on the dashboard.
 */
public class TestSnapshotLogger {
    public record Snapshot(
            String test,
            int run,
            int step,
            String label,
            double timestamp,
            Optional<Pose2d> questPose,
            Pose2d fusedPose,
            Rotation2d gyroHeading,
            Optional<TagFrame.Reading> tagFrame) {}

    private static final String CSV_HEADER = "SNAPSHOT_HEADER,test,run,step,label,time,"
            + "questX,questY,questDeg,fusedX,fusedY,fusedDeg,gyroDeg,"
            + "tagId,centerDistance,centerLateral,headingDeg,facingSide,leftCorner,rightCorner,sideLateral";

    private final Supplier<Optional<Pose2d>> questPose;
    private final Supplier<Pose2d> fusedPose;
    private final Supplier<Rotation2d> gyroHeading;
    private final TestSettings settings;

    private final Map<String, Integer> runCounts = new HashMap<>();
    private final List<Snapshot> currentRun = new ArrayList<>();
    private String currentTest = "None";
    private int currentRunNumber = 0;
    private int count = 0;
    private boolean headerPrinted = false;

    private final StringPublisher statusPub;
    private final StringPublisher lastPub;
    private final IntegerPublisher countPub;
    private final StringPublisher testPub;
    private final IntegerPublisher runPub;
    private final IntegerPublisher stepPub;
    private final StringPublisher labelPub;
    private final StructPublisher<Pose2d> questPub;
    private final StructPublisher<Pose2d> fusedPub;
    private final DoublePublisher gyroPub;
    private final DoublePublisher centerDistancePub;
    private final DoublePublisher centerLateralPub;
    private final DoublePublisher headingPub;
    private final StringPublisher facingSidePub;
    private final DoublePublisher leftCornerPub;
    private final DoublePublisher rightCornerPub;
    private final DoublePublisher sideLateralPub;

    /**
     * @param gyroHeading raw gyro heading; it only has to be steady over a test, so the robot
     *                    can compare Quest heading changes with it
     */
    public TestSnapshotLogger(
            Supplier<Optional<Pose2d>> questPose,
            Supplier<Pose2d> fusedPose,
            Supplier<Rotation2d> gyroHeading,
            TestSettings settings) {
        this.questPose = questPose;
        this.fusedPose = fusedPose;
        this.gyroHeading = gyroHeading;
        this.settings = settings;

        NetworkTable test = NetworkTableInstance.getDefault().getTable("Test");
        NetworkTable snap = test.getSubTable("Snapshot");
        statusPub = test.getStringTopic("Status").publish();
        lastPub = test.getStringTopic("Last Snapshot").publish();
        countPub = snap.getIntegerTopic("Count").publish();
        testPub = snap.getStringTopic("Test").publish();
        runPub = snap.getIntegerTopic("Run").publish();
        stepPub = snap.getIntegerTopic("Step").publish();
        labelPub = snap.getStringTopic("Label").publish();
        questPub = snap.getStructTopic("QuestPose", Pose2d.struct).publish();
        fusedPub = snap.getStructTopic("FusedPose", Pose2d.struct).publish();
        gyroPub = snap.getDoubleTopic("Gyro Heading (deg)").publish();
        centerDistancePub = snap.getDoubleTopic("Center Distance (m)").publish();
        centerLateralPub = snap.getDoubleTopic("Center Lateral (m)").publish();
        headingPub = snap.getDoubleTopic("Heading To Tag (deg)").publish();
        facingSidePub = snap.getStringTopic("Facing Side").publish();
        leftCornerPub = snap.getDoubleTopic("Left Corner Distance (m)").publish();
        rightCornerPub = snap.getDoubleTopic("Right Corner Distance (m)").publish();
        sideLateralPub = snap.getDoubleTopic("Side Lateral (m)").publish();
        setStatus("Idle");
    }

    /** Shows a short message on the dashboard at {@code Test/Status} and in the console. */
    public void setStatus(String status) {
        statusPub.set(status);
        System.out.println("TEST STATUS: " + status);
    }

    public Command status(String status) {
        return Commands.runOnce(() -> setStatus(status));
    }

    /** Starts a new numbered run of the named test. Snapshots after this belong to the run. */
    public Command startRun(String test) {
        return Commands.runOnce(() -> {
            currentTest = test;
            currentRunNumber = runCounts.merge(test, 1, Integer::sum);
            currentRun.clear();
            setStatus(test + " run " + currentRunNumber + " started");
        });
    }

    public Command snapshot(String label) {
        return Commands.runOnce(() -> take(label));
    }

    /** Snapshots taken since the last {@link #startRun}. */
    public List<Snapshot> currentRun() {
        return List.copyOf(currentRun);
    }

    /** The latest snapshot in the current run with this label. */
    public Optional<Snapshot> find(String label) {
        for (int i = currentRun.size() - 1; i >= 0; i--) {
            if (currentRun.get(i).label().equals(label)) {
                return Optional.of(currentRun.get(i));
            }
        }
        return Optional.empty();
    }

    public Snapshot take(String label) {
        Optional<Pose2d> quest = questPose.get();
        Pose2d fused = fusedPose.get();
        Optional<TagFrame.Reading> reading = quest.flatMap(p -> TagFrame.compute(
                p, settings.tagId(), settings.bumperLengthMeters(), settings.bumperWidthMeters()));
        Snapshot s = new Snapshot(currentTest, currentRunNumber, currentRun.size() + 1, label,
                Timer.getFPGATimestamp(), quest, fused, gyroHeading.get(), reading);
        currentRun.add(s);
        publish(s);
        return s;
    }

    private void publish(Snapshot s) {
        count++;
        testPub.set(s.test());
        runPub.set(s.run());
        stepPub.set(s.step());
        labelPub.set(s.label());
        s.questPose().ifPresent(questPub::set);
        fusedPub.set(s.fusedPose());
        gyroPub.set(s.gyroHeading().getDegrees());
        s.tagFrame().ifPresent(r -> {
            centerDistancePub.set(r.centerDistance());
            centerLateralPub.set(r.centerLateral());
            headingPub.set(r.headingDeg());
            facingSidePub.set(r.facingSide());
            leftCornerPub.set(r.leftCorner());
            rightCornerPub.set(r.rightCorner());
            sideLateralPub.set(r.sideLateral());
        });
        // Last, so a reader that watches Count sees a complete snapshot.
        countPub.set(count);

        lastPub.set(readable(s));
        if (!headerPrinted) {
            System.out.println(CSV_HEADER);
            headerPrinted = true;
        }
        System.out.println(csv(s));
    }

    private static String readable(Snapshot s) {
        String head = String.format(Locale.US, "%s #%d step %d [%s]", s.test(), s.run(), s.step(), s.label());
        if (s.tagFrame().isEmpty()) {
            return head + ": no Quest pose";
        }
        TagFrame.Reading r = s.tagFrame().get();
        return head + String.format(Locale.US,
                ": center %.3f m out, %.3f m lateral, heading %.1f deg | %s bumper: left %.3f m, right %.3f m, lateral %.3f m",
                r.centerDistance(), r.centerLateral(), r.headingDeg(),
                r.facingSide(), r.leftCorner(), r.rightCorner(), r.sideLateral());
    }

    private static String csv(Snapshot s) {
        StringBuilder b = new StringBuilder("SNAPSHOT,");
        b.append(s.test()).append(',').append(s.run()).append(',').append(s.step()).append(',')
                .append(s.label().replace(',', ' ')).append(',')
                .append(String.format(Locale.US, "%.3f", s.timestamp())).append(',');
        if (s.questPose().isPresent()) {
            Pose2d q = s.questPose().get();
            b.append(String.format(Locale.US, "%.4f,%.4f,%.2f,", q.getX(), q.getY(), q.getRotation().getDegrees()));
        } else {
            b.append(",,,");
        }
        Pose2d f = s.fusedPose();
        b.append(String.format(Locale.US, "%.4f,%.4f,%.2f,%.2f,",
                f.getX(), f.getY(), f.getRotation().getDegrees(), s.gyroHeading().getDegrees()));
        if (s.tagFrame().isPresent()) {
            TagFrame.Reading r = s.tagFrame().get();
            b.append(String.format(Locale.US, "%d,%.4f,%.4f,%.2f,%s,%.4f,%.4f,%.4f",
                    r.tagId(), r.centerDistance(), r.centerLateral(), r.headingDeg(),
                    r.facingSide(), r.leftCorner(), r.rightCorner(), r.sideLateral()));
        } else {
            b.append(",,,,,,,");
        }
        return b.toString();
    }
}
