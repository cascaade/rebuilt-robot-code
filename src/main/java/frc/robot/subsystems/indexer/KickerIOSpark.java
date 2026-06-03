package frc.robot.subsystems.indexer;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.indexer.IndexerConstants.KickerConstants.*;

public class KickerIOSpark implements KickerIO {
    private final SparkMax kickerMotor;
    private final SparkClosedLoopController kickerController;
    private final RelativeEncoder kickerEncoder;

    public KickerIOSpark(int indexMotorCANID) {
        kickerMotor = new SparkMax(indexMotorCANID, MotorType.kBrushless);
        kickerController = kickerMotor.getClosedLoopController();
        kickerEncoder = kickerMotor.getEncoder();

        syncControlConstants();
    }

    @Override
    public void syncControlConstants() {
        KICKER_CONTROL_CONSTANTS.applyIfChanged(KICKER_SPARK_CONFIG, kickerMotor);
    }

    @Override
    public void updateInputs(KickerIOInputs inputs) {
        inputs.velocity.mut_replace(kickerEncoder.getVelocity(), RadiansPerSecond);

        inputs.appliedVolts.mut_replace(kickerMotor.getBusVoltage() * kickerMotor.getAppliedOutput(), Volts);
        inputs.currentAmps.mut_replace(kickerMotor.getOutputCurrent(), Amps);
    }

    @Override
    public void setClosedLoop(AngularVelocity velocity) {
        kickerController.setSetpoint(velocity.in(RadiansPerSecond), ControlType.kVelocity, ClosedLoopSlot.kSlot0);
    }

    @Override
    public void setOpenLoop(Voltage voltage) {
        kickerMotor.setVoltage(voltage);
    }
}
