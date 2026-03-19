package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.ShooterConstants;
import frc.robot.util.LoggedTunableControlConstants;

public class ShooterIOTalonFlywheel implements ShooterIO {
    public final TalonFX motor;
    public final int CANID;

    private final VelocityVoltage velocityRequest = new VelocityVoltage(0.0);
    private final SlewRateLimiter rateLimiter = new SlewRateLimiter(100);

    public final LoggedTunableControlConstants controlConstants = ShooterConstants.flywheelConstants;

    public final double shooterEpsilon = 2;
    public double setpointRad = 0;
    public double currentVelocityRadPerSec = 0;

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
    public void setVelocityClosedLoop(double velocityRadPerSec) {
        setpointRad = rateLimiter.calculate(velocityRadPerSec);;
        Logger.recordOutput("Shooter/Flywheel/" + CANID + "/Setpoint", setpointRad);

        motor.setControl(
            velocityRequest.withVelocity(Units.radiansToRotations(setpointRad)).withSlot(0)
        );
    }

    @Override 
    public void setOpenLoop(double voltage) {
        rateLimiter.calculate(currentVelocityRadPerSec);
        motor.setVoltage(voltage); 
    }

    @Override 
    public void stop(){
        rateLimiter.calculate(currentVelocityRadPerSec); // stop rate limiter from jumping around due to gaps in the data it receives
        motor.setVoltage(0);
    }

    public void updateInputs(ShooterIOInputs inputs){
        currentVelocityRadPerSec = motor.getVelocity().getValueAsDouble() * 2 * Math.PI;
        inputs.velocityRadPerSec = currentVelocityRadPerSec;

        Logger.recordOutput("Shooter/Flywheel/" + CANID + "/AtSetpoint", Math.abs(currentVelocityRadPerSec - setpointRad) <  shooterEpsilon);

        inputs.appliedVolts = motor.getMotorVoltage().getValueAsDouble();
        inputs.supplyCurrentAmps = motor.getSupplyCurrent().getValueAsDouble();
    }
}
