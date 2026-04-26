package frc.robot.subsystems.climb;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;

import static edu.wpi.first.units.Units.Radians;

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

        inputs.appliedVolts = climbMotor.getBusVoltage() * climbMotor.getAppliedOutput();
        inputs.currentAmps = climbMotor.getOutputCurrent();
    }

    @Override
    public void setClosedLoop(Angle targetAngle) {
        climbController.setSetpoint(targetAngle.in(Radians), ControlType.kPosition);
    }

    @Override
    public void setOpenLoop(Voltage voltage) {
        climbMotor.setVoltage(voltage);
    }
}
