package frc.robot.subsystems.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.swerve.SwerveConstants.driveControlConstants;
import static frc.robot.subsystems.swerve.SwerveConstants.turnControlConstants;

// sim based on the advantagekit 2026 swerve example
public class SDSModuleIOSim implements SDSModuleIO {
    private final DCMotorSim driveSim;
    private final DCMotorSim turnSim;

    public static final DCMotor driveGearbox = DCMotor.getNEO(1);
    public static final DCMotor turnGearbox = DCMotor.getNEO(1);

    private boolean driveClosedLoop = false;
    private boolean turnClosedLoop = false;

    private PIDController driveController;
    private PIDController turnController;

    // to be updated by sim
    private final MutVoltage driveFFVolts = Volts.mutable(0);
    private final MutVoltage driveAppliedVolts = Volts.mutable(0);
    private final MutVoltage turnAppliedVolts = Volts.mutable(0);

    public SDSModuleIOSim() {
        driveSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(driveGearbox, 0.025, SwerveConstants.driveMotorReduction),
            driveGearbox
        );
        turnSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(turnGearbox, 0.004, SwerveConstants.turnMotorReduction),
            turnGearbox
        );

        driveController = new PIDController(0, 0, 0);
        turnController = new PIDController(0, 0, 0);
        turnController.enableContinuousInput(-Math.PI, Math.PI);

        syncControlConstants();
    }

    @Override
    public void updateInputs(SDSModuleIOInputs inputs) {
        if (driveClosedLoop) {
            driveAppliedVolts.mut_replace(driveFFVolts.in(Volts) + driveController.calculate(driveSim.getAngularVelocityRadPerSec()), Volts);
        } else {
            driveController.reset();
        }

        if (turnClosedLoop) {
            turnAppliedVolts.mut_replace(turnController.calculate(turnSim.getAngularPositionRad()), Volts);
        } else {
            turnController.reset();
        }

        driveSim.setInputVoltage(MathUtil.clamp(driveAppliedVolts.in(Volts), -12.0, 12.0));
        turnSim.setInputVoltage(MathUtil.clamp(turnAppliedVolts.in(Volts), -12.0, 12.0));
        driveSim.update(0.02);
        turnSim.update(0.02);

        inputs.driveConnected = true;
        inputs.driveDistance.mut_replace(
            SwerveConstants.kSwerveWheelRadius.times(driveSim.getAngularPosition().in(Radians))
        );
        inputs.driveLinearVelocity.mut_replace(
            SwerveConstants.kSwerveWheelRadius.in(Meters) * driveSim.getAngularVelocity().in(RadiansPerSecond),
            MetersPerSecond
        );
        inputs.driveAppliedVolts.mut_replace(driveAppliedVolts);
        inputs.driveCurrentAmps.mut_replace(Math.abs(driveSim.getCurrentDrawAmps()), Amps);

        inputs.turnConnected = true;
        inputs.turnPosition = new Rotation2d(turnSim.getAngularPosition());
        inputs.turnVelocity.mut_replace(turnSim.getAngularVelocity());
        inputs.turnAppliedVolts.mut_replace(turnAppliedVolts);
        inputs.turnCurrentAmps.mut_replace(Math.abs(turnSim.getCurrentDrawAmps()), Amps);
    }

    @Override
    public void syncControlConstants() {
        // todo: fix issue with applySimIfChanged bc it only applies to one module, and by then it is not considered changing.
        driveControlConstants.applySimTo(driveController);
        turnControlConstants.applySimTo(turnController);
    }

    @Override
    public void setDriveVelocity(AngularVelocity velocity) {
        driveClosedLoop = true;
        driveController.setSetpoint(velocity.in(RadiansPerSecond));
        driveFFVolts.mut_replace(0, Volts);
    }

    @Override
    public void setDriveOpenLoop(Voltage output) {
        driveClosedLoop = false;
        driveAppliedVolts.mut_replace(output);
    }

    @Override
    public void setTurnOpenLoop(Voltage output) {
        turnClosedLoop = false;
        turnAppliedVolts.mut_replace(output);
    }

    @Override
    public void setTurnPosition(Rotation2d rotation) {
        turnClosedLoop = true;
        turnController.setSetpoint(rotation.getRadians());
    }
}
