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
import static frc.robot.subsystems.indexer.IndexerConstants.*;

public class IndexerIOSpark implements IndexerIO {
    private final SparkMax indexerMotor;
    private final SparkClosedLoopController indexerController;
    private final RelativeEncoder indexerEncoder;

    public IndexerIOSpark() {
        indexerMotor = new SparkMax(INDEXER_CAN_ID, MotorType.kBrushless);
        indexerController = indexerMotor.getClosedLoopController();
        indexerEncoder = indexerMotor.getEncoder();

        syncControlConstants();
    }

    @Override
    public void syncControlConstants() {
        INDEXER_CONTROL_CONSTANTS.applyIfChanged(INDEXER_SPARK_CONFIG, indexerMotor);
    }

    @Override
    public void updateInputs(IndexerIOInputs inputs) {
        inputs.velocity.mut_replace(indexerEncoder.getVelocity(), RadiansPerSecond);

        inputs.appliedVolts.mut_replace(indexerMotor.getBusVoltage() * indexerMotor.getAppliedOutput(), Volts);
        inputs.currentAmps.mut_replace(indexerMotor.getOutputCurrent(), Amps);
    }

    @Override
    public void setClosedLoop(AngularVelocity velocity) {
        indexerController.setSetpoint(velocity.in(RadiansPerSecond), ControlType.kVelocity, ClosedLoopSlot.kSlot0);
    }

    @Override
    public void setOpenLoop(Voltage voltage) {
        indexerMotor.setVoltage(voltage);
    }
}
