package frc.robot.util;

import edu.wpi.first.math.geometry.*;

import static frc.robot.Constants.FieldConstants.FIELD_LENGTH;
import static frc.robot.Constants.FieldConstants.FIELD_WIDTH;

public class FieldUtil {
    public static Pose2d rotate(Pose2d pose) {
        return pose.rotateAround(new Translation2d(FIELD_LENGTH / 2, FIELD_WIDTH / 2), Rotation2d.k180deg);
    }

    public static Pose2d mirror(Pose2d pose) {
        return new Pose2d(new Translation2d(pose.getX() * 2 - (FIELD_LENGTH / 2), pose.getY()), pose.getRotation());
    }
}
