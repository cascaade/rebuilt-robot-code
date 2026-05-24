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

    public static final TalonFXConfiguration talonFlywheelConfigs = new TalonFXConfiguration();

    static {
        talonFlywheelConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        talonFlywheelConfigs.Audio.AllowMusicDurDisable = true;
        talonFlywheelConfigs.CurrentLimits.StatorCurrentLimit = 100;
        talonFlywheelConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
        talonFlywheelConfigs.CurrentLimits.SupplyCurrentLimit = 60;
        talonFlywheelConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;

        flywheelControlConstants.applyTo(talonFlywheelConfigs);
    }
}