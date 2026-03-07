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

public class ShooterIOSparkFeeder implements ShooterIO {

    public final SparkMax feederMotor;
    public final RelativeEncoder feederEncoder;
    public final SparkClosedLoopController feederController;

    public ShooterIOSparkFeeder(int CANID) {
        feederMotor = new SparkMax(CANID, MotorType.kBrushless);
        feederMotor.setCANTimeout(0);

        feederController = feederMotor.getClosedLoopController();
        feederEncoder = feederMotor.getEncoder();

        SparkMaxConfig motorConfig = ShooterConstants.feederConfig;
        feederMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override 
    public void setVelocityClosedLoop(double velocityRadPerSec) {
        // feedforward should already be accounted for
        feederController.setSetpoint(
            velocityRadPerSec,
            ControlType.kVelocity
        );
    }

    @Override 
    public void setOpenLoop(double voltage) {
        feederMotor.setVoltage(voltage);
    }

    @Override 
    public void stop(){
        feederMotor.stopMotor();
    }
    
    public void updateInputs(ShooterIOInputs inputs){
        inputs.velocityRadPerSec = feederEncoder.getVelocity();

        inputs.appliedVolts = feederMotor.getBusVoltage();
        inputs.supplyCurrentAmps = feederMotor.getOutputCurrent();
    }
}
