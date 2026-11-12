package frc.robot.testing;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

class TagFrameTest {
    private static final double BUMPER = 0.88;
    private static final double HALF = BUMPER / 2;

    // Tag 25 (2026 layout) is near (4.02, 4.39) and faces field -X.
    private final Pose2d tag = TagFrame.tagPose(25).orElseThrow();

    @Test
    void robotSquareInFrontOfTheTagWithItsFrontFacingIt() {
        Pose2d robot = new Pose2d(tag.getX() - 2.0, tag.getY(), Rotation2d.kZero);

        TagFrame.Reading r = TagFrame.compute(robot, 25, BUMPER, BUMPER).orElseThrow();

        assertEquals(2.0, r.centerDistance(), 1e-9);
        assertEquals(0.0, r.centerLateral(), 1e-9);
        assertEquals(0.0, r.headingDeg(), 1e-9);
        assertEquals("Front", r.facingSide());
        assertEquals(2.0 - HALF, r.leftCorner(), 1e-9);
        assertEquals(2.0 - HALF, r.rightCorner(), 1e-9);
        assertEquals(0.0, r.sideLateral(), 1e-9);
    }

    @Test
    void lateralIsPositiveToYourRightAsYouFaceTheTag() {
        // Facing the tag means facing field +X, so your right is field -Y.
        Pose2d robot = new Pose2d(tag.getX() - 2.0, tag.getY() - 0.5, Rotation2d.kZero);

        TagFrame.Reading r = TagFrame.compute(robot, 25, BUMPER, BUMPER).orElseThrow();

        assertEquals(0.5, r.centerLateral(), 1e-9);
        assertEquals(0.5, r.sideLateral(), 1e-9);
        assertEquals(robot.getTranslation(),
                TagFrame.fieldPoint(tag, 2.0, 0.5, Rotation2d.kZero).getTranslation());
    }

    @Test
    void headsetFacingTheTagPutsTheRightSideTowardIt() {
        // Headset yaw is -90 deg, so the robot faces +90 deg to point the headset at the tag.
        Pose2d robot = new Pose2d(tag.getX() - 2.0, tag.getY(), Rotation2d.kCCW_90deg);

        TagFrame.Reading r = TagFrame.compute(robot, 25, BUMPER, BUMPER).orElseThrow();

        assertEquals("Right", r.facingSide());
        assertEquals(90.0, r.headingDeg(), 1e-9);
        assertEquals(2.0 - HALF, r.leftCorner(), 1e-9);
        assertEquals(2.0 - HALF, r.rightCorner(), 1e-9);
    }

    @Test
    void cornerDistancesShowAHeadingError() {
        // Turned 5 deg CCW (seen from above): the right corner (as you face the tag) swings closer.
        Pose2d robot = new Pose2d(tag.getX() - 2.0, tag.getY(), Rotation2d.fromDegrees(5));

        TagFrame.Reading r = TagFrame.compute(robot, 25, BUMPER, BUMPER).orElseThrow();

        double expectedDifference = BUMPER * Math.sin(Math.toRadians(5));
        assertEquals(expectedDifference, r.leftCorner() - r.rightCorner(), 1e-9);
    }
}
