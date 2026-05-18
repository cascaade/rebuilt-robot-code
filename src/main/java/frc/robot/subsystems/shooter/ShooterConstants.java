package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.Constants;
import frc.robot.util.TunableControlConstants;

import static edu.wpi.first.units.Units.RadiansPerSecond;

public class ShooterConstants {
    public static final int shooterLMotorCANID = 16;
    public static final int shooterMMotorCANID = 17;
    public static final int shooterRMotorCANID = 18;
    public static final int feederMotorCANID = 23;
    public static final int indexMotorCANID = 24;

    // TODO test check theoretical limits
    public static final AngularVelocity shooterMaxSpeed = RadiansPerSecond.of(340);
    public static final AngularVelocity feederMaxSpeed = RadiansPerSecond.of(5000);
    public static final AngularVelocity indexMaxSpeed = RadiansPerSecond.of(600);
    public static final double idleMult = 0.8;
    public static final double feederMotorMult = 1;

    public static final AngularVelocity shooterRunSpeed = RadiansPerSecond.of(340);
    public static final AngularVelocity feederRunSpeed = RadiansPerSecond.of(5000);
    public static final AngularVelocity indexRunSpeed = RadiansPerSecond.of(7250);

    public static final TunableControlConstants flywheelControlConstants =
        new TunableControlConstants("Shooter/Flywheel")
            .withP(0.34)
            .withS(0.18)
            .withV(0.111);

    public static final TunableControlConstants feederControlConstants =
        new TunableControlConstants("Shooter/Feeder")
            .withP(0.000001)
            .withS(0.129)
            .withV(0.00203);

    public static final TunableControlConstants indexControlConstants =
        new TunableControlConstants("Shooter/Index")
            .withP(0.00001)
            .withS(0.18)
            .withV(0.00209);

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
            .positionWrappingEnabled(true)
            .positionWrappingInputRange(0, 2 * Math.PI)
            .outputRange(-1,1);
        feederConfig.signals
            .absoluteEncoderPositionAlwaysOn(true)
            .absoluteEncoderPositionPeriodMs((int) (1000.0 / Constants.odometryFrequency))
            .absoluteEncoderVelocityAlwaysOn(true)
            .absoluteEncoderVelocityPeriodMs(20)
            .appliedOutputPeriodMs(20)
            .busVoltagePeriodMs(20)
            .outputCurrentPeriodMs(20);

        feederControlConstants.applyTo(feederConfig);

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
            .positionWrappingEnabled(true)
            .positionWrappingInputRange(0, 2 * Math.PI)
            .outputRange(-1,1);
        indexConfig.signals
            .absoluteEncoderPositionAlwaysOn(true)
            .absoluteEncoderPositionPeriodMs((int) (1000.0 / Constants.odometryFrequency))
            .absoluteEncoderVelocityAlwaysOn(true)
            .absoluteEncoderVelocityPeriodMs(20)
            .appliedOutputPeriodMs(20)
            .busVoltagePeriodMs(20)
            .outputCurrentPeriodMs(20);

        indexControlConstants.applyTo(indexConfig);

        talonFlywheelConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        talonFlywheelConfigs.Audio.AllowMusicDurDisable = true;
        talonFlywheelConfigs.CurrentLimits.StatorCurrentLimit = 100;
        talonFlywheelConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
        talonFlywheelConfigs.CurrentLimits.SupplyCurrentLimit = 60;
        talonFlywheelConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
    }
}