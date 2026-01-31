package frc.robot.subsystems.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.SwerveModuleConstants;

// sim based on the advantagekit 2026 swerve example
public class SDSModuleIOSim implements SDSModuleIO {
    private final DCMotorSim driveSim;
    private final DCMotorSim turnSim;

    public static final DCMotor driveGearbox = DCMotor.getNEO(1);
    public static final DCMotor turnGearbox = DCMotor.getNEO(1);

    private boolean driveClosedLoop = false;
    private boolean turnClosedLoop = false;

    private PIDController driveController = new PIDController(
        SwerveModuleConstants.driveSimP,
        SwerveModuleConstants.driveSimI,
        SwerveModuleConstants.driveSimD
    );
    private PIDController turnController = new PIDController(
        SwerveModuleConstants.turnSimP,
        SwerveModuleConstants.turnSimI,
        SwerveModuleConstants.turnSimD
    );

    // to be updated by sim
    private double driveFFVolts = 0.0;
    private double driveAppliedVolts = 0.0;
    private double turnAppliedVolts = 0.0;

    public SDSModuleIOSim() {
        driveSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(driveGearbox, 0.025, SwerveModuleConstants.driveMotorReduction),
            driveGearbox
        );
        turnSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(turnGearbox, 0.004, SwerveModuleConstants.turnMotorReduction),
            turnGearbox
        );

        turnController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void updateInputs(SDSModuleIOInputs inputs) {
        if (driveClosedLoop) {
            driveAppliedVolts = driveFFVolts + driveController.calculate(driveSim.getAngularVelocityRadPerSec());
        } else {
            driveController.reset();
        }

        if (turnClosedLoop) {
            turnAppliedVolts = turnController.calculate(turnSim.getAngularPositionRad());
        } else {
            turnController.reset();
        }

        driveSim.setInputVoltage(MathUtil.clamp(driveAppliedVolts, -12.0, 12.0));
        turnSim.setInputVoltage(MathUtil.clamp(turnAppliedVolts, -12.0, 12.0));
        driveSim.update(0.02);
        turnSim.update(0.02);

        inputs.driveConnected = true;
        inputs.drivePositionRad = driveSim.getAngularPositionRad();
        inputs.driveVelocityRadPerSec = driveSim.getAngularVelocityRadPerSec();
        inputs.driveAppliedVolts = driveAppliedVolts;
        inputs.driveCurrentAmps = Math.abs(driveSim.getCurrentDrawAmps());

        inputs.turnConnected = true;
        inputs.turnPosition = new Rotation2d(turnSim.getAngularPositionRad());
        inputs.turnVelocityRadPerSec = turnSim.getAngularVelocityRadPerSec();
        inputs.turnAppliedVolts = turnAppliedVolts;
        inputs.turnCurrentAmps = Math.abs(turnSim.getCurrentDrawAmps());
    }

    @Override
    public void setDriveOpenLoop(double output) {
        driveClosedLoop = false;
        driveAppliedVolts = output;
    }

    @Override
    public void setTurnOpenLoop(double output) {
        turnClosedLoop = false;
        turnAppliedVolts = output;
    }

    @Override
    public void setDriveVelocityRadPerSec(double velocityRadPerSec) {
        driveClosedLoop = true;
        driveFFVolts = SwerveModuleConstants.driveSimKs * Math.signum(velocityRadPerSec) + SwerveModuleConstants.driveSimKv * velocityRadPerSec;
        driveController.setSetpoint(velocityRadPerSec);
    }

    @Override
    public void setTurnPosition(Rotation2d rotation) {
        turnClosedLoop = true;
        turnController.setSetpoint(rotation.getRadians());
    }
}
