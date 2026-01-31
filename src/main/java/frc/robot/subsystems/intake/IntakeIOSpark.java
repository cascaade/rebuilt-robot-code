package frc.robot.subsystems.intake;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import frc.robot.subsystems.Constants.IntakeConstants;

public class IntakeIOSpark implements IntakeIO {
    public final SparkMax intakeMotor;
    public final SparkClosedLoopController intakeController;
    public final SparkAbsoluteEncoder intakeEncoder;

    public IntakeIOSpark() {
        intakeMotor = new SparkMax(IntakeConstants.kWristCANID, MotorType.kBrushless);
        intakeController = intakeMotor.getClosedLoopController();
        intakeEncoder = intakeMotor.getAbsoluteEncoder();

        intakeMotor.configure(IntakeConstants.intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.positionMeters = intakeEncoder.getPosition();
        inputs.velocityMetersPerSec = intakeEncoder.getVelocity();

        inputs.appliedVolts = intakeMotor.getBusVoltage();
        inputs.currentAmps = intakeMotor.getOutputCurrent();
    }

    @Override
    public void setClosedLoop(double voltage) {
        intakeController.setSetpoint(voltage, ControlType.kVoltage);
    }

    @Override
    public void setOpenLoop(double voltage) {
        intakeMotor.setVoltage(voltage);
    }
    
}
