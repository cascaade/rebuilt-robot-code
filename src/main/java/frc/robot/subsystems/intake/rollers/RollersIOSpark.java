package frc.robot.subsystems.intake.rollers;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Timer;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.intake.IntakeConstants.RollerConstants.*;

public class RollersIOSpark implements RollersIO {
    private final SparkMax intakeMotor;
    private final SparkClosedLoopController intakeController;
    private final RelativeEncoder intakeEncoder;

    public RollersIOSpark() {
        intakeMotor = new SparkMax(ROLLERS_CAN_ID, MotorType.kBrushless);
        intakeController = intakeMotor.getClosedLoopController();
        intakeEncoder = intakeMotor.getEncoder();

        syncControlConstants();
    }

    @Override
    public void syncControlConstants() {
        ROLLERS_CONTROL_CONSTANTS.applyIfChanged(ROLLERS_SPARK_CONFIG, intakeMotor);
    }

    @Override
    public void updateInputs(RollersIOInputs inputs) {
        inputs.position.mut_replace(intakeEncoder.getPosition(), Radians);
        inputs.velocity.mut_replace(intakeEncoder.getVelocity(), RadiansPerSecond);

        inputs.appliedVolts.mut_replace(intakeMotor.getBusVoltage() * intakeMotor.getAppliedOutput(), Volts);
        inputs.currentAmps.mut_replace(intakeMotor.getOutputCurrent(), Amps);
    }

    @Override
    public void setClosedLoop(AngularVelocity velocity) {
        intakeController.setSetpoint(velocity.in(RadiansPerSecond), ControlType.kVelocity, ClosedLoopSlot.kSlot0);

        Logger.recordOutput("Intake/Roller/Setpoint", velocity);
        Logger.recordOutput("Intake/Roller/SetpointUpdate", Timer.getFPGATimestamp());
    }

    @Override
    public void setOpenLoop(Voltage voltage) {
        intakeMotor.setVoltage(voltage);
        Logger.recordOutput("Intake/Roller/Voltage", voltage);
        Logger.recordOutput("Intake/Roller/VoltageUpdate", Timer.getFPGATimestamp());
    }
}
