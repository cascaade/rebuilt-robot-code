package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.util.Units;
import frc.robot.Constants.ShooterConstants;
import frc.robot.util.LoggedTunableControlConstants;

import org.littletonrobotics.junction.Logger;

public class ShooterIOTalonFlywheel implements ShooterIO {
    public final TalonFX motor;
    public final int CANID;

    private final MotionMagicVelocityVoltage velocityRequest = new MotionMagicVelocityVoltage(0.0);

    public final LoggedTunableControlConstants controlConstants = ShooterConstants.flywheelConstants;

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

    @Override
    public void addToOrchestra(Orchestra orchestra, int track) {
        orchestra.addInstrument(motor, track);
    }

    @Override
    public void setVelocityClosedLoop(double velocityRadPerSec) {
        Logger.recordOutput("Shooter/Flywheel/" + CANID + "/Setpoint", velocityRadPerSec);

        motor.setControl(
            velocityRequest.withVelocity(Units.radiansToRotations(velocityRadPerSec)).withSlot(0)
        );
    }

    @Override 
    public void setOpenLoop(double voltage) {
        motor.setVoltage(voltage); 
    }

    @Override 
    public void stop(){
        motor.setVoltage(0);
    }

    public void updateInputs(ShooterIOInputs inputs){
        inputs.velocityRadPerSec = motor.getVelocity().getValueAsDouble() * 2 * Math.PI;

        inputs.appliedVolts = motor.getMotorVoltage().getValueAsDouble();
        inputs.supplyCurrentAmps = motor.getSupplyCurrent().getValueAsDouble();
    }
}
