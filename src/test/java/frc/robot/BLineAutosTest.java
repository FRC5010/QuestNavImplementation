package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.lib.BLine.Path;

/** Pure-logic tests: no HAL or robot hardware simulation required. */
class BLineAutosTest {
    @Test
    void discoversEveryPathInDeployDirectory() {
        List<String> names = BLineAutos.discoverPathNames();
        assertTrue(names.containsAll(List.of(
                "1-meter-right", "hubtorighttrench", "lefttrenchtohub", "righttrenchtolefttrench")),
                "discovered: " + names);
    }

    @Test
    void everyDiscoveredPathLoadsAndIsValid() {
        for (String name : BLineAutos.discoverPathNames()) {
            assertTrue(new Path(name).isValid(), name + " should be a valid BLine path");
        }
    }

    @Test
    void redAllianceOverlayIsFlippedAndDoesNotMutateTheOriginal() {
        Path blue = new Path("1-meter-right");
        Pose2d before = blue.getStartPose();

        Path red = BLineAutos.pathForAlliance(blue, true);

        assertNotEquals(before.getX(), red.getStartPose().getX(), 1e-6, "red overlay should be flipped");
        assertEquals(before, blue.getStartPose(), "original path must be unchanged");
        assertEquals(before, BLineAutos.pathForAlliance(blue, false).getStartPose());
    }

    @Test
    void redFlipUsesTheConfiguredFieldSize() {
        // 2025 Reefscape is 17.548 x 8.042 m; BLine's built-in default is the 2026 field (16.54 x 8.07).
        Pose2d blue = new Path("2025-start-to-reef-face").getStartPose();
        Pose2d red = BLineAutos.pathForAlliance(new Path("2025-start-to-reef-face"), true).getStartPose();

        assertEquals(FieldConfig.LAYOUT.getFieldLength() - blue.getX(), red.getX(), 1e-6);
        assertEquals(FieldConfig.LAYOUT.getFieldWidth() - blue.getY(), red.getY(), 1e-6);
        assertEquals(17.548, FieldConfig.LAYOUT.getFieldLength(), 1e-3, "FieldConfig should be the 2025 field");
    }
}
