package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.Constants;
import frc.robot.util.LoggedTunableControlConstants;

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
            .absoluteEncoderPositionPeriodMs((int) (1000.0 / Constants.odometryFrequency))
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
            .absoluteEncoderPositionPeriodMs((int) (1000.0 / Constants.odometryFrequency))
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