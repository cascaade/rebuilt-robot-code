package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

public class ShooterIOKraken implements ShooterIO {

    public final TalonFX shooterMotor; 

    public ShooterIOKraken(int CANID) {
        this.shooterMotor = new TalonFX(CANID); 
        

    }

    @Override 
    public void setDrive(double output) {
        shooterMotor.setControl(new DutyCycleOut(0.0).withOutput(output)); 
    }

    public void stop(){
        setDrive(0.0); 
    }

    public void updateInputs(ShooterIOInputs inputs){
        inputs.driveVelocityRadPerSec = shooterMotor.getVelocity().getValueAsDouble(); 

        inputs.driveAppliedVolts = shooterMotor.getBusVoltage().getValueAsDouble() * shooterMotor.getMotorOutputPercent().getValueAsDouble(); 

        inputs.driveCurrentAmps = shooterMotor.getStatorCurrent().getValueAsDouble(); 
    }
}
