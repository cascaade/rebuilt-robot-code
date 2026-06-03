package frc.robot.subsystems.intake.wrist;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.intake.IntakeConstants.WristConstants.*;

public class WristIOSpark implements WristIO {
    private final SparkMax wristMotor;
    private final SparkClosedLoopController wristController;
    private final RelativeEncoder wristEncoder;

    public WristIOSpark(int wristCanId) {
        wristMotor = new SparkMax(wristCanId, MotorType.kBrushless);
        wristController = wristMotor.getClosedLoopController();
        wristEncoder = wristMotor.getEncoder();

        syncControlConstants();
    }

    @Override
    public void syncControlConstants() {
        WRIST_CONTROL_CONSTANTS.applyIfChanged(WRIST_SPARK_CONFIG, wristMotor);
    }

    @Override
    public void resetPosition(Angle angle) {
        wristEncoder.setPosition(angle.in(Radians));
    }

    @Override
    public void updateInputs(WristIOInputs inputs) {
        inputs.position = new Rotation2d(wristEncoder.getPosition());
        inputs.velocity.mut_replace(wristEncoder.getVelocity(), RadiansPerSecond);

        inputs.appliedVolts.mut_replace(wristMotor.getBusVoltage() * wristMotor.getAppliedOutput(), Volts);
        inputs.currentAmps.mut_replace(wristMotor.getOutputCurrent(), Amps);
    }

    @Override
    public void setClosedLoop(Rotation2d angle) {
//        wristController.setSetpoint(angle, ControlType.kPosition);
        Logger.recordOutput("Intake/Wrist/HardwareSetpointAngle", angle);
    }

    @Override
    public void setOpenLoop(Voltage voltage) {
        wristMotor.setVoltage(voltage);
    }
}
