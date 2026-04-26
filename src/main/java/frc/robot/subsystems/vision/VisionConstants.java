package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class VisionConstants {
    public static record CameraConfig(
        String name,
        Transform3d robotToCamera,
        double standardDeviationMultiplier
    ) {}

    public static final CameraConfig[] camConfigs = {
        new CameraConfig(
            "shooter_left",
            new Transform3d(
                new Translation3d(
                    Units.inchesToMeters(-11.493),
                    Units.inchesToMeters(-7.75),
                    Units.inchesToMeters(7.92)
                ),
                new Rotation3d(
                    0,
                    Units.degreesToRadians(-15 - 7.5),
                    Units.degreesToRadians(180)
                )
            ),
            1
        ),
        new CameraConfig(
            "shooter_right",
            new Transform3d(
                new Translation3d(
                    Units.inchesToMeters(-11.5),
                    Units.inchesToMeters(6.9),
                    Units.inchesToMeters(7.75)
                ),
                new Rotation3d(
                    Units.degreesToRadians(0),
                    Units.degreesToRadians(-15),
                    Units.degreesToRadians(180)
                )
            ),
            1
        ),
        new CameraConfig(
            "side_left",
            new Transform3d(
                new Translation3d(
                    Units.inchesToMeters(-7.5),
                    Units.inchesToMeters(-12.3446),
                    Units.inchesToMeters(7.808)
                ),
                new Rotation3d(
                    Units.degreesToRadians(0),
                    Units.degreesToRadians(-15),
                    Units.degreesToRadians(-90)
                )
            ),
            1.0
        ),
        new CameraConfig(
            "side_right",
            new Transform3d(
                new Translation3d(
                    Units.inchesToMeters(-7.5),
                    Units.inchesToMeters(12.3446),
                    Units.inchesToMeters(7.808)
                ),
                new Rotation3d(
                    Units.degreesToRadians(0),
                    Units.degreesToRadians(-15),
                    Units.degreesToRadians(90)
                )
            ),
            1.0
        )
    };

    // AprilTag layout
    public static AprilTagFieldLayout aprilTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    public static final double maxAmbiguity = 0.3;
    public static final double maxZError = 0.75;

    public static final double linearStdDevBaseline = 0.02; // Meters
    public static final double angularStdDevBaseline = 0.06; // Radians

    // Multipliers to apply for MegaTag 2 observations
    public static final double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
    public static final double angularStdDevMegatag2Factor = Double.POSITIVE_INFINITY; // No rotation data available
}