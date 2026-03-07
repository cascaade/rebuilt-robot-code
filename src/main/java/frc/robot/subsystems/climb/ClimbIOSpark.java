package frc.robot.subsystems.climb;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import frc.robot.Constants.ClimbConstants;

public class ClimbIOSpark implements ClimbIO {
    private final SparkMax climbMotor;
    private final RelativeEncoder climbEncoder;
    private final SparkClosedLoopController climbController;

    public ClimbIOSpark() {
        climbMotor = new SparkMax(ClimbConstants.climberMotorCANID, MotorType.kBrushless);

        climbMotor.setCANTimeout(0);

        climbEncoder = climbMotor.getEncoder();
        climbController = climbMotor.getClosedLoopController();

        climbMotor.configure(ClimbConstants.climberConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void updateInputs(ClimbIOInputs inputs) {
        inputs.positionRadians = climbEncoder.getPosition();
        inputs.velocityRotPerSec = climbEncoder.getVelocity();

        inputs.appliedVolts = climbMotor.getAppliedOutput();
        inputs.currentAmps = climbMotor.getOutputCurrent();
    }

    @Override
    public void setClosedLoop(double radians) {
        climbController.setSetpoint(radians, ControlType.kPosition);
    }

    @Override
    public void setOpenLoop(double voltage) {
        climbMotor.setVoltage(voltage);
    }
}
