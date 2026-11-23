package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;

class FieldLayoutTest {
    private static final AprilTagFieldLayout OFFICIAL = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    @Test
    void hallwayLayoutIsLoaded() {
        // Tag 25 is the reference: it keeps its official pose.
        Pose3d official25 = OFFICIAL.getTagPose(25).orElseThrow();
        Pose3d hallway25 = QuestNavSubsystem.FIELD_LAYOUT.getTagPose(25).orElseThrow();
        assertEquals(0.0, official25.getTranslation().getDistance(hallway25.getTranslation()), 1e-9);

        // Tag 24 is measured on the wall, about 0.37 m along the face from its official spot.
        Pose3d official24 = OFFICIAL.getTagPose(24).orElseThrow();
        Pose3d hallway24 = QuestNavSubsystem.FIELD_LAYOUT.getTagPose(24).orElseThrow();
        double moved = official24.getTranslation().getDistance(hallway24.getTranslation());
        assertTrue(moved > 0.3 && moved < 0.45, "tag 24 moved " + moved + " m");
        assertEquals(OFFICIAL.getFieldLength(), QuestNavSubsystem.FIELD_LAYOUT.getFieldLength(), 1e-9);
    }
}
