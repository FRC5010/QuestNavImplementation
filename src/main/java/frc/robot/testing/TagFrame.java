package frc.robot.testing;

import java.util.List;
import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.QuestNavSubsystem;

/**
 * Converts a robot pose into values you can check with a tape measure or laser in front of a tag.
 *
 * <p>Tag frame: the origin is the floor point directly below the tag center. +X points straight
 * out from the tag face. +Y points to your right when you stand in front of the tag and look at
 * it. Only the floor plane is used; the tag height does not matter here.
 */
public final class TagFrame {
    private TagFrame() {}

    /**
     * @param tagId           AprilTag the values are relative to
     * @param centerDistance  robot center distance out from the tag plane (m)
     * @param centerLateral   robot center offset from the tag centerline, + = right (m)
     * @param headingDeg      robot front direction relative to "front points at the tag" (deg, CCW +)
     * @param facingSide      bumper side closest to the tag: Front, Left, Back or Right
     * @param leftCorner      distance from the tag plane to the left corner of that side (m)
     * @param rightCorner     distance from the tag plane to the right corner of that side (m)
     * @param sideLateral     offset of the middle of that side from the centerline, + = right (m)
     */
    public record Reading(
            int tagId,
            double centerDistance,
            double centerLateral,
            double headingDeg,
            String facingSide,
            double leftCorner,
            double rightCorner,
            double sideLateral) {}

    private record Side(String name, Translation2d cornerA, Translation2d cornerB) {}

    /** Tag pose on the floor plane, or empty if the tag is not in the field layout. */
    public static Optional<Pose2d> tagPose(int tagId) {
        return QuestNavSubsystem.FIELD_LAYOUT.getTagPose(tagId).map(p -> p.toPose2d());
    }

    /** A field pose at a point in the tag frame, with the given field heading. */
    public static Pose2d fieldPoint(Pose2d tag, double distance, double lateral, Rotation2d heading) {
        Translation2d point = tag.transformBy(new Transform2d(distance, lateral, Rotation2d.kZero)).getTranslation();
        return new Pose2d(point, heading);
    }

    public static Optional<Reading> compute(Pose2d robot, int tagId, double bumperLength, double bumperWidth) {
        Optional<Pose2d> tag = tagPose(tagId);
        if (tag.isEmpty()) {
            return Optional.empty();
        }
        Pose2d inTag = robot.relativeTo(tag.get());
        double headingDeg = inTag.getRotation().plus(Rotation2d.k180deg).getDegrees();

        double hl = bumperLength / 2;
        double hw = bumperWidth / 2;
        List<Side> sides = List.of(
                new Side("Front", new Translation2d(hl, hw), new Translation2d(hl, -hw)),
                new Side("Left", new Translation2d(hl, hw), new Translation2d(-hl, hw)),
                new Side("Back", new Translation2d(-hl, hw), new Translation2d(-hl, -hw)),
                new Side("Right", new Translation2d(hl, -hw), new Translation2d(-hl, -hw)));

        Side facing = null;
        Translation2d facingA = null;
        Translation2d facingB = null;
        double closest = Double.POSITIVE_INFINITY;
        for (Side side : sides) {
            Translation2d a = toTagFrame(inTag, side.cornerA());
            Translation2d b = toTagFrame(inTag, side.cornerB());
            double middle = (a.getX() + b.getX()) / 2;
            if (middle < closest) {
                closest = middle;
                facing = side;
                facingA = a;
                facingB = b;
            }
        }
        // As you face the tag, +Y is to your right, so the corner with the larger Y is the right one.
        Translation2d left = facingA.getY() < facingB.getY() ? facingA : facingB;
        Translation2d right = left == facingA ? facingB : facingA;

        return Optional.of(new Reading(
                tagId,
                inTag.getX(),
                inTag.getY(),
                headingDeg,
                facing.name(),
                left.getX(),
                right.getX(),
                (left.getY() + right.getY()) / 2));
    }

    private static Translation2d toTagFrame(Pose2d robotInTag, Translation2d robotPoint) {
        return robotInTag.transformBy(new Transform2d(robotPoint, Rotation2d.kZero)).getTranslation();
    }
}
