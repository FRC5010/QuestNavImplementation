package frc.robot.testing;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Dashboard knobs for the QuestNav test routines, under {@code Test/Settings/}. Values are read
 * each time a routine needs them, so changes on the dashboard apply to the next routine or step.
 */
public class TestSettings {
    private static final String ROOT = "Test/Settings/";

    static final String TAG_ID = ROOT + "Tag ID";
    static final String MAX_SPEED = ROOT + "Max Speed (mps)";
    static final String SETTLE_SECONDS = ROOT + "Settle Seconds";
    static final String DRIFT_SECONDS = ROOT + "Drift Seconds";
    static final String DRIFT_SQUARE = ROOT + "Drift Square (m)";
    static final String TAG_VIEW_SECONDS = ROOT + "Tag View Seconds";
    static final String BUMPER_LENGTH = ROOT + "Bumper Length (m)";
    static final String BUMPER_WIDTH = ROOT + "Bumper Width (m)";
    static final String MARKS_FRONT_FACES_TAG = ROOT + "Marks: Front Faces Tag";
    static final String SYNC_WIGGLE = ROOT + "Sync Wiggle";

    /** Set to true on the dashboard (or press Y) to continue after a measurement pause. */
    public static final String CONTINUE = "Test/Continue";

    public TestSettings() {
        SmartDashboard.setDefaultNumber(TAG_ID, 25);
        SmartDashboard.setDefaultNumber(MAX_SPEED, 0.75);
        SmartDashboard.setDefaultNumber(SETTLE_SECONDS, 2.0);
        SmartDashboard.setDefaultNumber(DRIFT_SECONDS, 30.0);
        SmartDashboard.setDefaultNumber(DRIFT_SQUARE, 1.0);
        SmartDashboard.setDefaultNumber(TAG_VIEW_SECONDS, 3.0);
        // Outside dimensions with bumpers. Measure the real robot and set these.
        SmartDashboard.setDefaultNumber(BUMPER_LENGTH, 0.88);
        SmartDashboard.setDefaultNumber(BUMPER_WIDTH, 0.88);
        // false: the headset faces the tag at each mark. true: the front bumper faces the tag.
        SmartDashboard.setDefaultBoolean(MARKS_FRONT_FACES_TAG, false);
        // true: each routine starts and ends with a sync wiggle, for the ground-truth video sync.
        SmartDashboard.setDefaultBoolean(SYNC_WIGGLE, true);
        SmartDashboard.putBoolean(CONTINUE, false);
    }

    public boolean syncWiggle() {
        return SmartDashboard.getBoolean(SYNC_WIGGLE, true);
    }

    public int tagId() {
        return (int) SmartDashboard.getNumber(TAG_ID, 25);
    }

    public double maxSpeedMps() {
        return SmartDashboard.getNumber(MAX_SPEED, 0.75);
    }

    public double settleSeconds() {
        return SmartDashboard.getNumber(SETTLE_SECONDS, 2.0);
    }

    public double driftSeconds() {
        return SmartDashboard.getNumber(DRIFT_SECONDS, 30.0);
    }

    public double driftSquareMeters() {
        return SmartDashboard.getNumber(DRIFT_SQUARE, 1.0);
    }

    public double tagViewSeconds() {
        return SmartDashboard.getNumber(TAG_VIEW_SECONDS, 3.0);
    }

    public double bumperLengthMeters() {
        return SmartDashboard.getNumber(BUMPER_LENGTH, 0.88);
    }

    public double bumperWidthMeters() {
        return SmartDashboard.getNumber(BUMPER_WIDTH, 0.88);
    }

    public boolean marksFrontFacesTag() {
        return SmartDashboard.getBoolean(MARKS_FRONT_FACES_TAG, false);
    }

    public boolean continueRequested() {
        return SmartDashboard.getBoolean(CONTINUE, false);
    }

    public void clearContinue() {
        SmartDashboard.putBoolean(CONTINUE, false);
    }
}
