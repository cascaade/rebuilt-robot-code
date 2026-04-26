package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.LoggedTunableControlConstants;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

public class ShooterIOTalonFlywheel implements ShooterIO {
    public final TalonFX motor;
    public final int CANID;

    private final VelocityVoltage velocityRequest = new VelocityVoltage(0.0);
    private final SlewRateLimiter rateLimiter = new SlewRateLimiter(100);

    public final LoggedTunableControlConstants controlConstants = ShooterConstants.flywheelConstants;

    public final AngularVelocity shooterEpsilon = RadiansPerSecond.of(2);
    public final MutAngularVelocity setpoint = RadiansPerSecond.mutable(0);
    public final MutAngularVelocity currentVelocity = RadiansPerSecond.mutable(0);

    public ShooterIOTalonFlywheel(int CANID) {
        motor = new TalonFX(CANID);
        this.CANID = CANID;

        motor.getConfigurator().apply(ShooterConstants.talonFlywheelConfigs);

        controlConstants.addCallback(() -> {
            Slot0Configs slot0 = new Slot0Configs();
            slot0.kP = controlConstants.kP();
            slot0.kD = controlConstants.kD();
            slot0.kS = controlConstants.kS();
            slot0.kV = controlConstants.kV();

            motor.getConfigurator().apply(slot0);

            System.out.println("Control Constants Updated!" + " " + CANID);
        });
    }

    // @Override
    // public void addToOrchestra(Orchestra orchestra, int track) {
    //     orchestra.addInstrument(motor, track);
    // }

    @Override
    public void setVelocityClosedLoop(AngularVelocity velocity) {
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

    @Override 
    public void stop(){
        rateLimiter.calculate(currentVelocity.in(RadiansPerSecond)); // stop rate limiter from jumping around due to gaps in the data it receives
        motor.setVoltage(0);
    }

    public void updateInputs(ShooterIOInputs inputs){
        currentVelocity.mut_replace(motor.getVelocity().getValue());
        inputs.velocity.mut_replace(currentVelocity);

        Logger.recordOutput("Shooter/Flywheel/" + CANID + "/AtSetpoint", currentVelocity.minus(setpoint).lt(shooterEpsilon));

        inputs.appliedVolts.mut_replace(motor.getMotorVoltage().getValue());
        inputs.supplyCurrentAmps.mut_replace(motor.getSupplyCurrent().getValue());
    }
}
