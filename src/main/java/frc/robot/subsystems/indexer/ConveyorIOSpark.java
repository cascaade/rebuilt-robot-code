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
import static frc.robot.subsystems.indexer.IndexerConstants.ConveyorConstants.*;

public class ConveyorIOSpark implements ConveyorIO {
    private final SparkMax conveyorMotor;
    private final SparkClosedLoopController conveyorController;
    private final RelativeEncoder conveyorEncoder;

    public ConveyorIOSpark() {
        conveyorMotor = new SparkMax(CONVEYOR_CAN_ID, MotorType.kBrushless);
        conveyorController = conveyorMotor.getClosedLoopController();
        conveyorEncoder = conveyorMotor.getEncoder();

        syncControlConstants();
    }

    @Override
    public void syncControlConstants() {
        CONVEYOR_CONTROL_CONSTANTS.applyIfChanged(CONVEYOR_SPARK_CONFIG, conveyorMotor);
    }

    @Override
    public void updateInputs(ConveyorIOInputs inputs) {
        inputs.velocity.mut_replace(conveyorEncoder.getVelocity(), RadiansPerSecond);

        inputs.appliedVolts.mut_replace(conveyorMotor.getBusVoltage() * conveyorMotor.getAppliedOutput(), Volts);
        inputs.currentAmps.mut_replace(conveyorMotor.getOutputCurrent(), Amps);
    }

    @Override
    public void setClosedLoop(AngularVelocity velocity) {
        conveyorController.setSetpoint(velocity.in(RadiansPerSecond), ControlType.kVelocity, ClosedLoopSlot.kSlot0);
    }

    @Override
    public void setOpenLoop(Voltage voltage) {
        conveyorMotor.setVoltage(voltage);
    }
}
