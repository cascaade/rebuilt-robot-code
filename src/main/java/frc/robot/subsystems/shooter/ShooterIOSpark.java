package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

public class ShooterIOSpark implements ShooterIO {
    
    public final SparkMax motor; 

    public ShooterIOSpark(int CANID) {
        this.motor = new SparkMax(CANID, MotorType.kBrushless); 
    }

    @Override 
    public void setVelocityClosedLoop(double output) {
        //motor.setControl(new DutyCycleOut(0.0).withOutput(output)); 
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
        /*inputs.driveVelocityRadPerSec = motor.getVelocity().getValueAsDouble() * 2 * Math.PI; 

        inputs.driveAppliedVolts = motor.getSupplyVoltage().getValueAsDouble(); 

        inputs.driveCurrentAmps = motor.getStatorCurrent().getValueAsDouble(); */
    }
}
