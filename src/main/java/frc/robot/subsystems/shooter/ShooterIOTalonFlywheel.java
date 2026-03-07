package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.ShooterConstants;

public class ShooterIOTalonFlywheel implements ShooterIO {
    public final TalonFX motor;

    private final MotionMagicVelocityVoltage velocityRequest = new MotionMagicVelocityVoltage(0.0);

    public ShooterIOTalonFlywheel(int CANID) {
        motor = new TalonFX(CANID); 
        
        MotorOutputConfigs outputConfigs = new MotorOutputConfigs(); 
        outputConfigs.NeutralMode = NeutralModeValue.Coast; // IMPORTANT
        motor.getConfigurator().apply(outputConfigs);

        TalonFXConfiguration talonConfigs = new TalonFXConfiguration();

        var slot0 = talonConfigs.Slot0;
        slot0.kP = ShooterConstants.kP;
        slot0.kI = ShooterConstants.kD;
        slot0.kS = ShooterConstants.kS;
        slot0.kV = ShooterConstants.kV;

        var motionMagicConfigs = talonConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = ShooterConstants.mmCruise;
        motionMagicConfigs.MotionMagicAcceleration = ShooterConstants.mmAcceleration;
        motionMagicConfigs.MotionMagicJerk = ShooterConstants.mmJerk;

        motor.getConfigurator().apply(talonConfigs);
    }

    @Override 
    public void setVelocityClosedLoop(double velocityRadPerSec) {
        motor.setControl(
            velocityRequest.withVelocity(Units.radiansToRotations(velocityRadPerSec))
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
