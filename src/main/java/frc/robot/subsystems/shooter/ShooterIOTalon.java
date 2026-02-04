package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants.ShooterConstants;

public class ShooterIOTalon implements ShooterIO {

    public final TalonFX motor; 

    private final VelocityVoltage velocityRequest = new VelocityVoltage(0.0); 

    public ShooterIOTalon(int CANID) {
        motor = new TalonFX(CANID); 
        
        MotorOutputConfigs outputConfigs = new MotorOutputConfigs(); 
        outputConfigs.NeutralMode = NeutralModeValue.Coast; 
        motor.getConfigurator().apply(outputConfigs); 

        Slot0Configs slot0 = new Slot0Configs(); 
        slot0.kP = ShooterConstants.motorKP; 
        slot0.kI = ShooterConstants.motorKI; 
        slot0.kD = ShooterConstants.motorKD; 
        slot0.kV = ShooterConstants.motorKV; 

        motor.getConfigurator().apply(slot0); 
    }

    @Override 
    public void setVelocityClosedLoop(double velocityRadPerSec) {
        double velocityRPS = velocityRadPerSec / (2.0 * Math.PI); 
        motor.setControl(velocityRequest.withVelocity(velocityRPS)); 
    }

    @Override 
    public void setOpenLoop(double voltage) {
        motor.setVoltage(voltage); 
    }

    @Override 
    public void stop(){
        motor.stopMotor(); 
    }

    public void updateInputs(ShooterIOInputs inputs){
        inputs.driveVelocityRadPerSec = motor.getVelocity().getValueAsDouble() * 2 * Math.PI; 

        inputs.driveAppliedVolts = motor.getMotorVoltage().getValueAsDouble(); 

        inputs.driveSupplyCurrentAmps = motor.getSupplyCurrent().getValueAsDouble(); 
    }
}
