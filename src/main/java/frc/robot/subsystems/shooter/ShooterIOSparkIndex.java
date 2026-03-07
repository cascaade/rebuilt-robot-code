package frc.robot.subsystems.shooter;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants.ShooterConstants;

public class ShooterIOSparkIndex implements ShooterIO {

    public final SparkMax indexMotor;
    public final RelativeEncoder indexEncoder;
    public final SparkClosedLoopController indexController;

    public ShooterIOSparkIndex(int CANID) {
        indexMotor = new SparkMax(CANID, MotorType.kBrushless);
        indexMotor.setCANTimeout(0);

        indexController = indexMotor.getClosedLoopController();
        indexEncoder = indexMotor.getEncoder();

        SparkMaxConfig motorConfig = ShooterConstants.indexConfig;
        indexMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override 
    public void setVelocityClosedLoop(double velocityRadPerSec) {
        // feedforward should already be accounted for
        indexController.setSetpoint(
            velocityRadPerSec,
            ControlType.kVelocity
        );
    }

    @Override 
    public void setOpenLoop(double voltage) {
        indexMotor.setVoltage(voltage);
    }

    @Override 
    public void stop(){
        indexMotor.stopMotor();
    }
    
    public void updateInputs(ShooterIOInputs inputs){
        inputs.velocityRadPerSec = indexEncoder.getVelocity();

        inputs.appliedVolts = indexMotor.getBusVoltage();
        inputs.supplyCurrentAmps = indexMotor.getOutputCurrent();
    }
}
