package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import frc.robot.lib.BLine.FlippingUtil;

/**
 * Which game field the autos are drawn for. Change {@link #FIELD} to switch years; everything
 * else (field size for red-alliance flipping) is read from that field's AprilTag layout.
 *
 * <p>Why it matters: BLine flips blue paths to red by mirroring across the field. Its default
 * field size is 2026 (16.54 x 8.07 m). The 2025 field is about a metre longer, so with the wrong
 * size every red path would end up about a metre off in X.
 *
 * <p>Paths are drawn for one field. The 2026 paths (hubtorighttrench etc.) still load, but on the
 * 2025 field they sit next to the reef and their red flips land about a metre off.
 */
public final class FieldConfig {
    /** 2025 Reefscape, AndyMark field. For 2026 use {@code k2026RebuiltAndymark}. */
    public static final AprilTagFields FIELD = AprilTagFields.k2025ReefscapeAndyMark;

    public static final AprilTagFieldLayout LAYOUT = AprilTagFieldLayout.loadField(FIELD);

    private FieldConfig() {}

    /** Tells BLine how big the field is (it flips rotationally, which is right for 2025 and 2026). */
    public static void applyToBLine() {
        FlippingUtil.fieldSizeX = LAYOUT.getFieldLength();
        FlippingUtil.fieldSizeY = LAYOUT.getFieldWidth();
    }
}
