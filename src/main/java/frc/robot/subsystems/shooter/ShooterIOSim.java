package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.shooter.ShooterConstants.FLYWHEEL_CONTROL_CONSTANTS;
import static frc.robot.subsystems.shooter.ShooterConstants.flywheelMOI;

public class ShooterIOSim implements ShooterIO {
    private static final DCMotor gearbox = DCMotor.getKrakenX60(1);

    private final DCMotorSim flywheelSim;

    private final PIDController controller;
    private final SimpleMotorFeedforward feedforward;

    private boolean closedLoop = false;

    private final MutVoltage appliedVolts = Volts.mutable(0);

    public ShooterIOSim() {
        flywheelSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                gearbox,
                flywheelMOI,
                1
            ),
            gearbox
        );

        controller = new PIDController(0, 0, 0);
        feedforward = new SimpleMotorFeedforward(0, 0, 0);

        syncControlConstants();
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        if (closedLoop) {
            appliedVolts.mut_replace(
                feedforward.calculate(inputs.velocity.in(RadiansPerSecond)) + controller.calculate(
                    flywheelSim.getAngularVelocityRadPerSec()
                ),
                Volts
            );
        } else {
            controller.reset();
        }

        flywheelSim.setInputVoltage(
            MathUtil.clamp(
                appliedVolts.in(Volts),
                -12.0,
                12.0
            )
        );

        flywheelSim.update(0.02);

        inputs.velocity.mut_replace(
            flywheelSim.getAngularVelocity()
        );

        inputs.appliedVolts.mut_replace(appliedVolts);

        inputs.supplyCurrentAmps.mut_replace(
            Math.abs(flywheelSim.getCurrentDrawAmps()),
            Amps
        );
    }

    @Override
    public void setClosedLoop(AngularVelocity velocity) {
        closedLoop = true;

        Logger.recordOutput("Shooter/RecordedSetpoint", velocity);

        controller.setSetpoint(
            velocity.in(RadiansPerSecond)
        );
    }

    @Override
    public void setOpenLoop(Voltage voltage) {
        closedLoop = false;
        appliedVolts.mut_replace(voltage);
    }

    @Override
    public void syncControlConstants() {
        FLYWHEEL_CONTROL_CONSTANTS.applySimTo(controller, feedforward);
    }
}
