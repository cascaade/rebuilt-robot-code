package frc.robot.util;

import edu.wpi.first.math.geometry.*;
import frc.robot.Constants;

import static frc.robot.Constants.FieldConstants.FIELD_LENGTH;
import static frc.robot.Constants.FieldConstants.FIELD_WIDTH;

/**
 * A class that provides methods to flip the alliance localization of {@link Pose2d} objects
 */
public class FieldUtil {
    /**
     * Rotate a {@link Pose2d} around the center of the field
     * @param pose The pose to rotate
     * @return The rotated pose
     */
    public static Pose2d rotate(Pose2d pose) {
        return pose.rotateAround(new Translation2d(FIELD_LENGTH / 2, FIELD_WIDTH / 2), Rotation2d.k180deg);
    }

    public static Pose2d rotateIfRed(Pose2d pose) {
        return Constants.isRed() ? rotate(pose) : pose;
    }

    /**
     * Reflect a {@link Pose2d} across the center of the field
     * @param pose The pose to reflect
     * @return The reflected pose
     */
    public static Pose2d mirror(Pose2d pose) {
        return new Pose2d(new Translation2d(pose.getX() * 2 - (FIELD_LENGTH / 2), pose.getY()), pose.getRotation());
    }

    public static Pose2d mirrorIfRed(Pose2d pose) {
        return Constants.isRed() ? mirror(pose) : pose;
    }
}
