package frc.robot.subsystems;

import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

public class Constants {
    public static enum RobotMode {
		/** Running on a real robot. */
		REAL,
	
		/** Running a physics simulator. */
		SIM,
	
		/** Replaying from a log file. */
		REPLAY;
	}

    public static final RobotMode simMode = RobotMode.SIM;
	public static final RobotMode currentMode = RobotBase.isReal() ? RobotMode.REAL : simMode;

    public static boolean isRed() {
        return DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red;
    }

    public static enum FieldType {
		ANDYMARK, WELDED
	}

    public static final SendableChooser<FieldType> kFieldType = new SendableChooser<>();

    static {
        kFieldType.setDefaultOption("AndyMark Field", FieldType.ANDYMARK);
        kFieldType.addOption("Welded Field", FieldType.WELDED);
    }

    public static final double odometryFrequency = 50.0; // hz (50 is default of 20ms)

    public static class SwerveConstants {
        public static final int[] turnCANIDs = { 1, 2, 3, 4 };
        public static final int[] driveCANIDs = { 5, 6, 7, 8 };
        public static final int[] canCoderCANIDs = { 9, 10, 11, 12 };
        public static final int pigeonCANID = 13;

        public static final double kWheelDistanceMetersX = Units.inchesToMeters(20); // forward/back
        public static final double kWheelDistanceMetersY = Units.inchesToMeters(20); // left/right

        public static final double kSlowedMult = 0.12;
        
        public static final double kMaxWheelSpeed = 20; // m/2
        public static final double kMagVelLimit = 5; // m/s -- trusting andy that 5 is physical limit, maybe requires testing
        public static final double kRotVelLimit = 2 * (2 * Math.PI); // rad/s

        public static final double toXDelaySeconds = 1;

        // https://firstfrc.blob.core.windows.net/frc2026/FieldAssets/2026-field-dimension-dwgs.pdf
        /**
         * Get robot's initial pose based on field type and alliance
         * @return
         */
        public static Pose2d getInitialPose() {
            if (isRed()) {
                if (kFieldType.getSelected().equals(FieldType.WELDED)) {
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
    }

    public static class SwerveModuleConstants {
        public static final double kSwerveWheelDiameter = Units.inchesToMeters(4);
        
        // drive config
        public static final SparkMaxConfig driveConfig = new SparkMaxConfig();

        public static final double driveMotorReduction = 6.12; // l3 mk4i gear set
        
        public static final int driveMotorCurrentLimit = 50;

        public static final double driveEncoderPositionFactor = 2 * Math.PI / driveMotorReduction; // Rotor Rotations -> Wheel Radians
        public static final double driveEncoderVelocityFactor = (2 * Math.PI) / 60.0 / driveMotorReduction; // Rotor RPM -> Wheel Rad/Sec
        
        public static final double driveP = 0.008;
        public static final double driveI = 0;
        public static final double driveD = 0.1;
        public static final double driveKs = 0.0;
        public static final double driveKv = 0.12;

        public static final double driveSimP = 0.05;
        public static final double driveSimI = 0;
        public static final double driveSimD = 0.0;
        public static final double driveSimKs = 0.0;
        public static final double driveSimKv = 0.0789;
        
        // turn config
        public static final SparkMaxConfig turnConfig = new SparkMaxConfig();

        public static final double turnMotorReduction = 150/7;

        public static final int turnMotorCurrentLimit = 20;

        public static final double turnEncoderPositionFactor = 2 * Math.PI / turnMotorReduction;
        public static final double turnEncoderVelocityFactor = (2 * Math.PI) / 60.0 / turnMotorReduction;
        
        public static final double turnP = 1.0;
        public static final double turnI = 0;
        public static final double turnD = 0;

        public static final double turnSimP = 8.0;
        public static final double turnSimI = 0;
        public static final double turnSimD = 0.0;

        public static final double turnPIDMinInput = 0;
        public static final double turnPIDMaxInput = 2 * Math.PI;
        public static final double[] doubleZeroRotations = {
            0.296, // fl
            -0.242, // fr
            0.041, // bl
            0.372 // br
        };
        public static final Rotation2d[] zeroRotations = {
            new Rotation2d(Units.rotationsToRadians(doubleZeroRotations[0])),
            new Rotation2d(Units.rotationsToRadians(doubleZeroRotations[1])),
            new Rotation2d(Units.rotationsToRadians(doubleZeroRotations[2])),
            new Rotation2d(Units.rotationsToRadians(doubleZeroRotations[3]))
        };

        static {
            Preferences.initDouble("driveP", driveP);
            Preferences.initDouble("driveI", driveI);
            Preferences.initDouble("driveD", driveD);
            Preferences.initDouble("driveKv", driveKv);
            Preferences.initDouble("driveKs", driveKs);
            Preferences.initDouble("turnP", turnP);
            Preferences.initDouble("turnI", turnI);
            Preferences.initDouble("turnD", turnD);

            turnConfig
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(turnMotorCurrentLimit)
                .voltageCompensation(12)
                .closedLoopRampRate(0.01)
                .inverted(true);
            turnConfig.encoder
                .positionConversionFactor(turnEncoderPositionFactor)
                .velocityConversionFactor(turnEncoderVelocityFactor)
                .uvwAverageDepth(2);
            turnConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(turnP, turnI, turnD)
                .positionWrappingEnabled(true)
                .positionWrappingInputRange(turnPIDMinInput, turnPIDMaxInput)
                .outputRange(-1,1);
            turnConfig.signals
                .absoluteEncoderPositionAlwaysOn(true)
                .absoluteEncoderPositionPeriodMs((int) (1000.0 / odometryFrequency))
                .absoluteEncoderVelocityAlwaysOn(true)
                .absoluteEncoderVelocityPeriodMs(20)
                .appliedOutputPeriodMs(20)
                .busVoltagePeriodMs(20)
                .outputCurrentPeriodMs(20);

            driveConfig
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(driveMotorCurrentLimit)
                .voltageCompensation(12.0)
                .closedLoopRampRate(0.01);
            driveConfig.encoder
                .positionConversionFactor(driveEncoderPositionFactor)
                .velocityConversionFactor(driveEncoderVelocityFactor)
                .uvwMeasurementPeriod(10)
                .uvwAverageDepth(2);
            driveConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(driveP, driveI, driveD)
                .outputRange(-1, 1);
                // .feedForward
                //     .kV(driveKv);
            driveConfig.signals
                .primaryEncoderPositionAlwaysOn(true)
                .primaryEncoderPositionPeriodMs((int) (1000.0 / odometryFrequency))
                .primaryEncoderVelocityAlwaysOn(true)
                .primaryEncoderVelocityPeriodMs(20)
                .appliedOutputPeriodMs(20)
                .busVoltagePeriodMs(20)
                .outputCurrentPeriodMs(20);
        }
    }

    public static class IntakeConstants {
        public static final int kWristCANID = 14;

        public static final SparkMaxConfig wristConfig = new SparkMaxConfig();
        public static final SparkBaseConfig intakeConfig = new SparkMaxConfig();

        public static final double intakeVoltage = 6;

        public static final double wristMotorReduction = 1;

        public static final double wristEncoderPositionFactor = 2 * Math.PI / wristMotorReduction;
        public static final double wristEncoderVelocityFactor = (2 * Math.PI) / wristMotorReduction / 60;

        public static final double wristP = 0;
        public static final double wristI = 0;
        public static final double wristD = 0;
        public static final double wristG = 0;
        public static final double wristS = 0;
        public static final double wristV = 0;

        public static final double wristPIDMinInput = 0;
        public static final double wristPIDMaxInput = 0;

        static {
            wristConfig
                .idleMode(IdleMode.kBrake)
                .voltageCompensation(12)
                .closedLoopRampRate(0.01)
                .inverted(false);
            wristConfig.encoder
                .positionConversionFactor(wristEncoderPositionFactor)
                .velocityConversionFactor(wristEncoderVelocityFactor)
                .uvwAverageDepth(2);
            wristConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(wristP, wristI, wristD)
                .positionWrappingEnabled(true)
                .positionWrappingInputRange(wristPIDMinInput, wristPIDMaxInput)
                .outputRange(-1,1);
            wristConfig.signals
                .absoluteEncoderPositionAlwaysOn(true)
                .absoluteEncoderPositionPeriodMs((int) (1000.0 / odometryFrequency))
                .absoluteEncoderVelocityAlwaysOn(true)
                .absoluteEncoderVelocityPeriodMs(20)
                .appliedOutputPeriodMs(20)
                .busVoltagePeriodMs(20)
                .outputCurrentPeriodMs(20);
            
        }
    }

    public static class OperatorConstants {
        public static final int kDriverControllerPort = 0;
        public static final int kAuxControllerPort = 1;
    }
}
