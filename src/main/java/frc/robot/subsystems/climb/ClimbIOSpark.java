package frc.robot.subsystems.climb;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.climb.ClimbConstants.CLIMBER_CONFIG;
import static frc.robot.subsystems.climb.ClimbConstants.CLIMBER_CONTROL_CONSTANTS;

public class ClimbIOSpark implements ClimbIO {
    private final SparkMax climbMotor;
    private final RelativeEncoder climbEncoder;
    private final SparkClosedLoopController climbController;

    public ClimbIOSpark() {
        climbMotor = new SparkMax(ClimbConstants.CLIMBER_CAN_ID, MotorType.kBrushless);

        climbMotor.setCANTimeout(0);

        climbEncoder = climbMotor.getEncoder();
        climbController = climbMotor.getClosedLoopController();

        syncControlConstants();
    }

    @Override
    public void updateInputs(ClimbIOInputs inputs) {
        inputs.position.mut_replace(climbEncoder.getPosition(), Radians);
        inputs.velocity.mut_replace(climbEncoder.getVelocity(), RadiansPerSecond);

        inputs.appliedVolts.mut_replace(climbMotor.getBusVoltage() * climbMotor.getAppliedOutput(), Volts);
        inputs.currentAmps.mut_replace(climbMotor.getOutputCurrent(), Amps);
    }

    @Override
    public void syncControlConstants() {
        CLIMBER_CONTROL_CONSTANTS.applyIfChanged(CLIMBER_CONFIG, climbMotor);
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
