package frc.robot.subsystems.swerve;

import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;
import frc.robot.Constants;
import frc.robot.util.TunableControlConstants;

import static edu.wpi.first.units.Units.*;

public class SwerveConstants {
    public static final int[] turnCANIDs = { 1, 2, 3, 4 };
    public static final int[] driveCANIDs = { 5, 6, 7, 8 };
    public static final int[] canCoderCANIDs = { 9, 10, 11, 12 };
    public static final int pigeonCANID = 13;

    public static final Angle BODY_ROTATION_ALIGN_TOLERANCE = Degrees.of(5);

    public static final Distance kWheelDistanceX = Inches.of(25 - 5.25); // forward/back
    public static final Distance kWheelDistanceY = Inches.of(29 - 5.25); // left/right

    public static final double kSlowedMult = 0.12;

    public static final LinearVelocity kMaxWheelSpeed = MetersPerSecond.of(4.6);
    public static final LinearVelocity kMagVelLimit = MetersPerSecond.of(4.5);
    public static final AngularVelocity kRotVelLimit = RotationsPerSecond.of(2);

    public static final Time crossDelay = Seconds.of(.3);

    public static final Distance kSwerveWheelRadius = Inches.of(2);

    public static final SparkMaxConfig driveConfig = new SparkMaxConfig();
    public static final TunableControlConstants driveControlConstants =
        new TunableControlConstants("Swerve/Drive")
            .withP(0.0001)
            .withS(0.01)
            .withV(0.11)
            .withSimP(0.05)
            .withSimV(0.0789);

    public static final double driveMotorReduction = 6.75; // l2 mk4i gear set

    public static final Current driveMotorCurrentLimit = Amps.of(30);

    public static final double driveEncoderPositionFactor = 2 * Math.PI * kSwerveWheelRadius.in(Meters) / driveMotorReduction; // Rotor Rotations -> Wheel Radians
    public static final double driveEncoderVelocityFactor = (2 * Math.PI * kSwerveWheelRadius.in(Meters)) / 60.0 / driveMotorReduction; // Rotor RPM -> Wheel Rad/Sec

    // turn config
    public static final SparkMaxConfig turnConfig = new SparkMaxConfig();
    public static final TunableControlConstants turnControlConstants =
        new TunableControlConstants("Swerve/Drive")
            .withP(0.6)
            .withSimP(8);

    public static final double turnMotorReduction = 150 / 7;

    public static final Current turnMotorCurrentLimit = Amps.of(20);

    public static final double turnEncoderPositionFactor = 2 * Math.PI / turnMotorReduction;
    public static final double turnEncoderVelocityFactor = (2 * Math.PI) / 60.0 / turnMotorReduction;

    public static final double turnPIDMinInput = 0;
    public static final double turnPIDMaxInput = 2 * Math.PI;
    public static final Rotation2d[] zeroRotations = {
        new Rotation2d(Units.rotationsToRadians(0.048)), // fl
        new Rotation2d(Units.rotationsToRadians(-0.464)), // fr
        new Rotation2d(Units.rotationsToRadians(0.051)), // bl
        new Rotation2d(Units.rotationsToRadians(0.353)) // br
    };

    static {
        driveConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit((int) driveMotorCurrentLimit.in(Amps))
            .voltageCompensation(12.0)
            .closedLoopRampRate(0.01);
        driveConfig.encoder
            .positionConversionFactor(driveEncoderPositionFactor)
            .velocityConversionFactor(driveEncoderVelocityFactor)
            .uvwMeasurementPeriod(10)
            .uvwAverageDepth(2);
        driveConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .outputRange(-1, 1);
        driveConfig.signals
            .primaryEncoderPositionAlwaysOn(true)
            .primaryEncoderPositionPeriodMs((int) (1000.0 / Constants.odometryFrequency))
            .primaryEncoderVelocityAlwaysOn(true)
            .primaryEncoderVelocityPeriodMs(20)
            .appliedOutputPeriodMs(20)
            .busVoltagePeriodMs(20)
            .outputCurrentPeriodMs(20);

        driveControlConstants.applyTo(driveConfig);

        turnConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit((int) turnMotorCurrentLimit.in(Amps))
            .voltageCompensation(12.0)
            .inverted(true);
        turnConfig.encoder
            .positionConversionFactor(turnEncoderPositionFactor)
            .velocityConversionFactor(turnEncoderVelocityFactor)
            .uvwAverageDepth(2);
        turnConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
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

        turnControlConstants.applyTo(turnConfig);
    }
}