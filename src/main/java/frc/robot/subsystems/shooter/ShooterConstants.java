package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
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
    public static final double idleMult = 0.8;
    public static final double feederMotorMult = 1;

    public static final AngularVelocity shooterRunSpeed = RadiansPerSecond.of(340);
    public static final AngularVelocity feederRunSpeed = RadiansPerSecond.of(5000);
    public static final AngularVelocity indexRunSpeed = RadiansPerSecond.of(7250);

    public static final AngularVelocity SHOOTER_SPEED_TOLERANCE = RadiansPerSecond.of(5);

    public static final int FLYWHEEL_STATOR_CURRENT_LIMIT = 60;
    public static final int FLYWHEEL_SUPPLY_CURRENT_LIMIT = 40;

    public static final TunableControlConstants FLYWHEEL_CONTROL_CONSTANTS =
        new TunableControlConstants("Shooter/Flywheel")
            .withP(0.34)
            .withS(0.18)
            .withV(0.111);

    public static final TalonFXConfiguration FLYWHEEL_SPARK_CONFIG = new TalonFXConfiguration();

    static {
        FLYWHEEL_SPARK_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        FLYWHEEL_SPARK_CONFIG.Audio.AllowMusicDurDisable = true;
        FLYWHEEL_SPARK_CONFIG.CurrentLimits.StatorCurrentLimit = FLYWHEEL_STATOR_CURRENT_LIMIT;
        FLYWHEEL_SPARK_CONFIG.CurrentLimits.StatorCurrentLimitEnable = true;
        FLYWHEEL_SPARK_CONFIG.CurrentLimits.SupplyCurrentLimit = FLYWHEEL_SUPPLY_CURRENT_LIMIT;
        FLYWHEEL_SPARK_CONFIG.CurrentLimits.SupplyCurrentLimitEnable = true;

        FLYWHEEL_CONTROL_CONSTANTS.applyTo(FLYWHEEL_SPARK_CONFIG);
    }
}