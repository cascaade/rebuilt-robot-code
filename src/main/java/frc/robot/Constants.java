package frc.robot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.util.LoggedTunableControlConstants;

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

            return SwerveConstants.getInitialPose().transformBy(
                new Transform2d(x, y, Rotation2d.kZero)
            );
        }
    }

    public static class SwerveConstants {
        public static final int[] turnCANIDs = { 1, 2, 3, 4 };
        public static final int[] driveCANIDs = { 5, 6, 7, 8 };
        public static final int[] canCoderCANIDs = { 9, 10, 11, 12 };
        public static final int pigeonCANID = 13;

        public static final double kWheelDistanceMetersX = Units.inchesToMeters(25 - 5.25); // forward/back
        public static final double kWheelDistanceMetersY = Units.inchesToMeters(29 - 5.25); // left/right

        public static final double kSlowedMult = 0.12;
        
        public static final double kMaxWheelSpeed = 20; // m/s?
        public static final double kMagVelLimit = 4.5; // m/s
        public static final double kRotVelLimit = 2 * (2 * Math.PI); // rad/s

        public static final double crossbuckDelaySeconds = 1;

        // https://firstfrc.blob.core.windows.net/frc2026/FieldAssets/2026-field-dimension-dwgs.pdf
        /**
         * Get robot's initial pose based on field type and alliance
         * @return
         */
        public static Pose2d getInitialPose() {
            if (isRed()) {
                if (kFieldType.equals(FieldType.WELDED)) {
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

        public static final double driveMotorReduction = 6.75; // l2 mk4i gear set
        
        public static final int driveMotorCurrentLimit = 30;

        public static final double driveEncoderPositionFactor = 2 * Math.PI / driveMotorReduction; // Rotor Rotations -> Wheel Radians
        public static final double driveEncoderVelocityFactor = (2 * Math.PI) / 60.0 / driveMotorReduction; // Rotor RPM -> Wheel Rad/Sec
        
        public static final double driveP = 0.0001;
        public static final double driveI = 0;
        public static final double driveD = 0;
        public static final double driveKs = 0.01;
        public static final double driveKv = 0.11;

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
        
        public static final double turnP = 0.6;
        public static final double turnI = 0;
        public static final double turnD = 0;

        public static final double turnSimP = 8.0;
        public static final double turnSimI = 0;
        public static final double turnSimD = 0.0;

        public static final double turnPIDMinInput = 0;
        public static final double turnPIDMaxInput = 2 * Math.PI;
        public static final double[] doubleZeroRotations = {
            0.048, // fl
            -0.464, // fr
            0.051, // bl
            0.353 // br
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

    public static class VisionConstants {
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
                "photonvision_climber",
                new Transform3d(),
                1.0
            ),
            new CameraConfig(
                "photonvision_shoot_climb",
                new Transform3d(),
                1.0
            )
        };

        // AprilTag layout
        public static AprilTagFieldLayout aprilTagLayout =
            AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

        public static double maxAmbiguity = 0.3;
        public static double maxZError = 0.75;

        public static double linearStdDevBaseline = 0.02; // Meters
        public static double angularStdDevBaseline = 0.06; // Radians

        // Multipliers to apply for MegaTag 2 observations
        public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
        public static double angularStdDevMegatag2Factor =
            Double.POSITIVE_INFINITY; // No rotation data available
    }

    public static class ClimbConstants {
        public static enum ClimbPose {
            RETRACTED(2 * Math.PI),
            EXTENDED(0);

            // radians
            private final double setpoint;

            ClimbPose(double setpoint) {
                this.setpoint = setpoint;
            }

            public double getSetpoint() {
                return setpoint;
            }
        }

        public static final int climberMotorCANID = 32;

        public static final double climbMotorReduction = 3.0 / 1.0;
        public static final double climbEncoderPositionFactor = 2 * Math.PI / climbMotorReduction;
        public static final double climbEncoderVelocityFactor = (2 * Math.PI) / 60.0 / climbMotorReduction;

        public static final double kP = 5;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kS = 0;
        public static final double kV = 0;

        public static final SparkMaxConfig climberConfig = new SparkMaxConfig();

        static {
            climberConfig
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(60)
                .voltageCompensation(12)
                .inverted(false);
            climberConfig.encoder
                .positionConversionFactor(climbEncoderPositionFactor)
                .positionConversionFactor(climbEncoderVelocityFactor)
                .uvwAverageDepth(2);
            climberConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(kP, kI, kD)
                .outputRange(-1,1);
            climberConfig.closedLoop.feedForward
                .kS(kS).kV(kV);
            climberConfig.signals
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

    public static class ShooterConstants {
        public static final int shooterLMotorCANID = 16;
        public static final int shooterMMotorCANID = 17;
        public static final int shooterRMotorCANID = 18;
        public static final int feederMotorCANID = 23;
        public static final int indexMotorCANID = 24;

        // TODO test check theoretical limits
        public static final double shooterMaxSpeed = 340; // rad/sec
        public static final double feederMaxSpeed = 5000; // rad/sec
        public static final double indexMaxSpeed = 600; // rad/sec
        public static final double idleMult = 0.8;
        public static final double feederMotorMult = 1;
        
        public static final double shooterRunSpeed = 340; // rad/sec
        public static final double feederRunSpeed = 5000; // rad/sec
        public static final double indexRunSpeed = 5000; // rad/sec

        public static final LoggedTunableControlConstants flywheelConstants =
            new LoggedTunableControlConstants("Shooter/Flywheel")
                .setP(0.34)
                .setD(0)
                .setS(0.18)
                .setV(0.111);

        public static final LoggedTunableControlConstants feederConstants =
            new LoggedTunableControlConstants("Shooter/Feeder")
                .setP(0.000001)
                .setD(0)
                .setS(0.129)
                .setV(0.00203);

        public static final LoggedTunableControlConstants indexConstants =
            new LoggedTunableControlConstants("Shooter/Index")
                .setP(0.00001)
                .setD(0)
                .setS(0.18)
                .setV(0.00209);

        public static final SparkMaxConfig feederConfig = new SparkMaxConfig();
        public static final SparkMaxConfig indexConfig = new SparkMaxConfig();
        public static final TalonFXConfiguration talonFlywheelConfigs = new TalonFXConfiguration();

        public static final double feederMotorReduction = 3.0 / 1.0;
        public static final double feederEncoderPositionFactor = 2 * Math.PI / feederMotorReduction;
        public static final double feederEncoderVelocityFactor = (2 * Math.PI) / 60.0 / feederMotorReduction;

        public static final double indexMotorReduction = 3.0 / 1.0;
        public static final double indexEncoderPositionFactor = 2 * Math.PI / indexMotorReduction;
        public static final double indexEncoderVelocityFactor = (2 * Math.PI) / 60.0 / indexMotorReduction;

        static {
            feederConfig
                .idleMode(IdleMode.kCoast)
                .smartCurrentLimit(30)
                .voltageCompensation(12)
                .closedLoopRampRate(0.01)
                .inverted(false);
            feederConfig.encoder
                .positionConversionFactor(feederEncoderPositionFactor)
                .velocityConversionFactor(feederEncoderVelocityFactor)
                .uvwAverageDepth(2);
            feederConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(feederConstants.kP(), 0, feederConstants.kD())
                .positionWrappingEnabled(true)
                .positionWrappingInputRange(0, 2 * Math.PI)
                .outputRange(-1,1);
            feederConfig.closedLoop.feedForward
                .kS(feederConstants.kS()).kV(feederConstants.kV());
            feederConfig.signals
                .absoluteEncoderPositionAlwaysOn(true)
                .absoluteEncoderPositionPeriodMs((int) (1000.0 / odometryFrequency))
                .absoluteEncoderVelocityAlwaysOn(true)
                .absoluteEncoderVelocityPeriodMs(20)
                .appliedOutputPeriodMs(20)
                .busVoltagePeriodMs(20)
                .outputCurrentPeriodMs(20);

            indexConfig
                .idleMode(IdleMode.kCoast)
                .smartCurrentLimit(30)
                .voltageCompensation(12)
                .closedLoopRampRate(0.01)
                .inverted(false);
            indexConfig.encoder
                .positionConversionFactor(indexEncoderPositionFactor)
                .velocityConversionFactor(indexEncoderVelocityFactor)
                .uvwAverageDepth(2);
            indexConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(indexConstants.kP(), 0, indexConstants.kD())
                .positionWrappingEnabled(true)
                .positionWrappingInputRange(0, 2 * Math.PI)
                .outputRange(-1,1);
            indexConfig.closedLoop.feedForward
                .kS(indexConstants.kS()).kV(indexConstants.kV());
            indexConfig.signals
                .absoluteEncoderPositionAlwaysOn(true)
                .absoluteEncoderPositionPeriodMs((int) (1000.0 / odometryFrequency))
                .absoluteEncoderVelocityAlwaysOn(true)
                .absoluteEncoderVelocityPeriodMs(20)
                .appliedOutputPeriodMs(20)
                .busVoltagePeriodMs(20)
                .outputCurrentPeriodMs(20);

            talonFlywheelConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
            talonFlywheelConfigs.Audio.AllowMusicDurDisable = true;
            var slot0 = talonFlywheelConfigs.Slot0;
            slot0.kP = flywheelConstants.kP();
            slot0.kD = flywheelConstants.kD();
            slot0.kS = flywheelConstants.kS();
            slot0.kV = flywheelConstants.kV();
            talonFlywheelConfigs.CurrentLimits.StatorCurrentLimit = 100;
            talonFlywheelConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
            talonFlywheelConfigs.CurrentLimits.SupplyCurrentLimit = 60;
            talonFlywheelConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
        }
    }
    
    public static class IntakeConstants {
        public static final int kWristCANID = 28;
        public static final int kRollerCANID = 29;

        public static final SparkMaxConfig wristSparkConfig = new SparkMaxConfig();
        public static final SparkMaxConfig rollerSparkConfig = new SparkMaxConfig();

        public static final double intakeMaxSpeed = 5000; // rad/sec

        public static final double autoHomeCurrentThreshold = 30;
        public static final int wristCurrentLimit = 40;
        public static final int rollerCurrentLimit = 30;

        public static final double wristMotorReduction = 32.0 / 1.0;
        public static final double rollerMotorReduction = 2.0 / 1.0;

        public static final double wristEncoderPositionFactor = 2 * Math.PI / wristMotorReduction;
        public static final double wristEncoderVelocityFactor = (2 * Math.PI) / wristMotorReduction / 60;
        public static final double rollerEncoderPositionFactor = 2 * Math.PI / rollerMotorReduction;
        public static final double rollerEncoderVelocityFactor = (2 * Math.PI) / rollerMotorReduction / 60;

        public static final double wristP = 0.2;
        public static final double wristD = 0;
        public static final double wristCos = 0;
        public static final double wristS = 0;

        public static final double rollerP = 0.00025;
        public static final double rollerD = 0;
        public static final double rollerS = 0.23;
        public static final double rollerV = 0.0395;

        public static enum IntakeWristPose {
            STOWED(0.1),
            DEPLOYED(1.95);

            private final double setpoint;

            IntakeWristPose(double setpoint) {
                this.setpoint = setpoint;
            }

            public double getSetpoint() {
                return setpoint;
            }
        }

        static {
            wristSparkConfig
                .idleMode(IdleMode.kBrake)
                .voltageCompensation(12)
                .smartCurrentLimit(wristCurrentLimit)
                .inverted(false);
            wristSparkConfig.encoder
                .positionConversionFactor(wristEncoderPositionFactor)
                .velocityConversionFactor(wristEncoderVelocityFactor)
                .uvwAverageDepth(2);
            wristSparkConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(wristP, 0.0, wristD)
                .positionWrappingEnabled(true)
                .positionWrappingInputRange(0, 2 * Math.PI)
                .outputRange(-1,1);
            wristSparkConfig.closedLoop.feedForward
                .kCos(wristCos).kS(wristS);
            wristSparkConfig.signals
                .absoluteEncoderPositionAlwaysOn(true)
                .absoluteEncoderPositionPeriodMs((int) (1000.0 / odometryFrequency))
                .absoluteEncoderVelocityAlwaysOn(true)
                .absoluteEncoderVelocityPeriodMs(20)
                .appliedOutputPeriodMs(20)
                .busVoltagePeriodMs(20)
                .outputCurrentPeriodMs(20);

            rollerSparkConfig
                .idleMode(IdleMode.kCoast)
                .voltageCompensation(12)
                .smartCurrentLimit(rollerCurrentLimit)
                .closedLoopRampRate(0.02)
                .inverted(true);
            rollerSparkConfig.encoder
                .positionConversionFactor(rollerEncoderPositionFactor)
                .velocityConversionFactor(rollerEncoderVelocityFactor)
                .uvwAverageDepth(2);
            rollerSparkConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(rollerP, 0.0, rollerD)
                .outputRange(-1,1);
            rollerSparkConfig.closedLoop.feedForward
                .kV(rollerV).kS(rollerS);
            rollerSparkConfig.signals
                .absoluteEncoderPositionAlwaysOn(true)
                .absoluteEncoderPositionPeriodMs((int) (1000.0 / odometryFrequency))
                .absoluteEncoderVelocityAlwaysOn(true)
                .absoluteEncoderVelocityPeriodMs(20)
                .appliedOutputPeriodMs(20)
                .busVoltagePeriodMs(20)
                .outputCurrentPeriodMs(20);
        }
    }
}
