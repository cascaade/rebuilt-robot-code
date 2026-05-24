package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.shooter.ShooterConstants.flywheelControlConstants;
import static frc.robot.subsystems.shooter.ShooterConstants.talonFlywheelConfigs;

public class ShooterIOTalon implements ShooterIO {
    public final TalonFX motor;
    public final int CANID;

    private final VelocityVoltage velocityRequest = new VelocityVoltage(0.0);
    private final SlewRateLimiter rateLimiter = new SlewRateLimiter(100);

    public final AngularVelocity shooterEpsilon = RadiansPerSecond.of(2);
    public final MutAngularVelocity setpoint = RadiansPerSecond.mutable(0);
    public final MutAngularVelocity currentVelocity = RadiansPerSecond.mutable(0);

    public ShooterIOTalon(int CANID) {
        motor = new TalonFX(CANID);
        this.CANID = CANID;

        syncControlConstants();
    }

    @Override
    public void syncControlConstants() {
        flywheelControlConstants.applyIfChanged(talonFlywheelConfigs, motor);
    }

    // @Override
    // public void addToOrchestra(Orchestra orchestra, int track) {
    //     orchestra.addInstrument(motor, track);
    // }

    @Override
    public void setClosedLoop(AngularVelocity velocity) {
        setpoint.mut_replace(rateLimiter.calculate(velocity.in(RadiansPerSecond)), RadiansPerSecond);
        Logger.recordOutput("Shooter/Flywheel/" + CANID + "/Setpoint", setpoint);

        motor.setControl(
            velocityRequest.withVelocity(setpoint).withSlot(0)
        );
    }

    @Override 
    public void setOpenLoop(Voltage voltage) {
        rateLimiter.calculate(currentVelocity.in(RadiansPerSecond));
        motor.setVoltage(voltage.in(Volts));
    }

    public void updateInputs(ShooterIOInputs inputs){
        currentVelocity.mut_replace(motor.getVelocity().getValue());
        inputs.velocity.mut_replace(currentVelocity);

        Logger.recordOutput("Shooter/Flywheel/" + CANID + "/AtSetpoint", currentVelocity.minus(setpoint).lt(shooterEpsilon));

        inputs.appliedVolts.mut_replace(motor.getMotorVoltage().getValue());
        inputs.supplyCurrentAmps.mut_replace(motor.getSupplyCurrent().getValue());
    }
}
