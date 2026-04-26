package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;

public class Constants {
    public static enum RobotMode {
		/** Running on a real robot. */
		REAL,
	
		/** Running a physics simulator. */
		SIM,
	
		/** Replaying from a log file. */
		REPLAY,
	}

    public static final RobotMode simMode = RobotMode.SIM;
	public static final RobotMode currentMode = RobotBase.isReal() ? RobotMode.REAL : simMode;

    public static boolean isRed() {
        return DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red;
    }

    // https://firstfrc.blob.core.windows.net/frc2026/FieldAssets/2026-field-dimension-dwgs.pdf
    /**
     * Get robot's initial pose based on field type and alliance
     * @return
     */
    public static Pose2d getInitialPose() {
        if (Constants.isRed()) {
            if (Constants.kFieldType.equals(FieldType.WELDED)) {
                return new Pose2d(
                    Units.inchesToMeters(651.22),
                    Units.inchesToMeters(317.69),
                    Rotation2d.k180deg
                );
            } else {
                return new Pose2d(
                    Units.inchesToMeters(650.12),
                    Units.inchesToMeters(316.64),
                    Rotation2d.k180deg
                );
            }
        } else {
            return new Pose2d();
        }
    }

    public static enum FieldType {
		ANDYMARK("andymark"), WELDED("welded");

        private final String jsonFolder;

        FieldType(String folder) {
            this.jsonFolder = folder;
        }

        public String getJsonFolder() {
            return jsonFolder;
        }
	}

    public static final FieldType kFieldType = FieldType.ANDYMARK;

    // public static final SendableChooser<FieldType> kFieldType = new SendableChooser<>();
    // static {
    //     kFieldType.setDefaultOption("AndyMark Field", FieldType.ANDYMARK);
    //     kFieldType.addOption("Welded Field", FieldType.WELDED);
    // }

    public static final double odometryFrequency = 50.0; // hz (50 is default of 20ms)

    public static class FieldConstants {
        public static Pose2d getHubCenter() {
            double x, y;

            if (kFieldType.equals(FieldType.WELDED)) {
                x = Units.inchesToMeters(182.11);
                y = Units.inchesToMeters(158.84);
            } else {
                x = Units.inchesToMeters(181.56);
                y = Units.inchesToMeters(158.32);
            }

            return getInitialPose().transformBy(
                new Transform2d(x, y, Rotation2d.kZero)
            );
        }
    }

    public static class OperatorConstants {
        public static final int kDriverControllerPort = 0;
        public static final int kAuxControllerPort = 1;
    }
}
