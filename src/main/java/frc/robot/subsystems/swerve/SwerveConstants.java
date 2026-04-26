package frc.robot.subsystems.swerve;

import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Preferences;
import frc.robot.Constants;

public class SwerveConstants {
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

    public static final double crossbuckDelaySeconds = .3;

    public static final double kSwerveWheelDiameter = Units.inchesToMeters(4);

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

    public static final double turnMotorReduction = 150 / 7;

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
            .outputRange(-1, 1);
        turnConfig.signals
            .absoluteEncoderPositionAlwaysOn(true)
            .absoluteEncoderPositionPeriodMs((int) (1000.0 / Constants.odometryFrequency))
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
            .primaryEncoderPositionPeriodMs((int) (1000.0 / Constants.odometryFrequency))
            .primaryEncoderVelocityAlwaysOn(true)
            .primaryEncoderVelocityPeriodMs(20)
            .appliedOutputPeriodMs(20)
            .busVoltagePeriodMs(20)
            .outputCurrentPeriodMs(20);
    }
}