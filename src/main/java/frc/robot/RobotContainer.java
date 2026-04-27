// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.autos.AutoBrain;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.intake.RollerIO;
import frc.robot.subsystems.intake.RollerIOSpark;
import frc.robot.subsystems.intake.WristIO;
import frc.robot.subsystems.intake.WristIOSpark;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOSparkFeeder;
import frc.robot.subsystems.shooter.ShooterIOSparkIndex;
import frc.robot.subsystems.shooter.ShooterIOTalonFlywheel;
import frc.robot.subsystems.swerve.GyroIO;
import frc.robot.subsystems.swerve.GyroIOPigeon;
import frc.robot.subsystems.swerve.SDSModuleIO;
import frc.robot.subsystems.swerve.SDSModuleIOSim;
import frc.robot.subsystems.swerve.SDSModuleIOSpark;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhoton;

public class RobotContainer {
    private final CommandXboxController driverController;
    private final CommandXboxController auxController;

    private final SwerveSubsystem swerveSubsystem;
    private final VisionSubsystem visionSubsystem;
    private final ShooterSubsystem shooterSubsystem;
    private final IntakeSubsystem intakeSubsystem;

    private final AutoBrain autoBrain;

    public RobotContainer() {

        Preferences.removeAll();

        driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);
        auxController = new CommandXboxController(OperatorConstants.kAuxControllerPort);

        switch (Constants.currentMode) {
            case REAL:
                swerveSubsystem = new SwerveSubsystem(
                    new GyroIOPigeon(),
                    new SDSModuleIOSpark(0),
                    new SDSModuleIOSpark(1),
                    new SDSModuleIOSpark(2),
                    new SDSModuleIOSpark(3)
                );
                visionSubsystem = new VisionSubsystem(
                    swerveSubsystem::addVisionMeasurement,
                    new VisionIOPhoton(VisionConstants.camConfigs[0]),
                    new VisionIOPhoton(VisionConstants.camConfigs[1]),
                    new VisionIOPhoton(VisionConstants.camConfigs[2]),
                    new VisionIOPhoton(VisionConstants.camConfigs[3])
                );
                shooterSubsystem = new ShooterSubsystem(
                    new ShooterIOTalonFlywheel(ShooterConstants.shooterLMotorCANID),
                    new ShooterIOTalonFlywheel(ShooterConstants.shooterMMotorCANID),
                    new ShooterIOTalonFlywheel(ShooterConstants.shooterRMotorCANID),
                    new ShooterIOSparkFeeder(ShooterConstants.feederMotorCANID),
                    new ShooterIOSparkIndex(ShooterConstants.indexMotorCANID),
                    swerveSubsystem::getPose
                );
                intakeSubsystem = new IntakeSubsystem(
                    new RollerIOSpark(),
                    new WristIOSpark()
                );
                break;
            case SIM:
                swerveSubsystem = new SwerveSubsystem(
                    new GyroIO() {},
                    new SDSModuleIOSim(),
                    new SDSModuleIOSim(),
                    new SDSModuleIOSim(),
                    new SDSModuleIOSim()
                );
                visionSubsystem = new VisionSubsystem(
                    swerveSubsystem::addVisionMeasurement,
                    new VisionIO() {},
                    new VisionIO() {},
                    new VisionIO() {},
                    new VisionIO() {}
                );
                shooterSubsystem = new ShooterSubsystem(
                    new ShooterIO() {},
                    new ShooterIO() {},
                    new ShooterIO() {},
                    new ShooterIO() {},
                    new ShooterIO() {},
                    Pose2d::new
                );
                intakeSubsystem = new IntakeSubsystem(
                    new RollerIO() {},
                    new WristIO() {}
                );
                break;
            default:
                swerveSubsystem = new SwerveSubsystem(
                    new GyroIO() {},
                    new SDSModuleIO() {},
                    new SDSModuleIO() {},
                    new SDSModuleIO() {},
                    new SDSModuleIO() {}
                );
                visionSubsystem = new VisionSubsystem(
                    swerveSubsystem::addVisionMeasurement,
                    new VisionIO() {},
                    new VisionIO() {},
                    new VisionIO() {},
                    new VisionIO() {}
                );
                shooterSubsystem = new ShooterSubsystem(
                    new ShooterIO() {},
                    new ShooterIO() {},
                    new ShooterIO() {},
                    new ShooterIO() {},
                    new ShooterIO() {},
                    Pose2d::new
                );
                intakeSubsystem = new IntakeSubsystem(
                    new RollerIO() {},
                    new WristIO() {}
                );
                break;
        }
        
        configureBindings();

        autoBrain = new AutoBrain(swerveSubsystem, shooterSubsystem, intakeSubsystem);
    }

    private void configureBindings() {
        // swerve default joystick inputs
        swerveSubsystem.setDefaultCommand(swerveSubsystem.runDriveInputs(
            driverController::getLeftX,          // vx
            driverController::getLeftY,          // vy
            driverController::getRightX,         // omega
            driverController::getLeftTriggerAxis // raw slow input
        ));

        // reset robot orientation (doesn't work with vision or FMS)
        driverController.y().onTrue(swerveSubsystem.runZeroGyro());

        // hub aim on/off
        driverController.leftBumper().onTrue(swerveSubsystem.runToggleAimHub(true));
        driverController.leftBumper().onFalse(swerveSubsystem.runToggleAimHub(false));

        // swerve adjustments using POV
        // auxController.povUp().onTrue(swerve.runXSetTime(-0.15));
        // auxController.povDown().onTrue(swerve.runXSetTime(0.15));
        // auxController.povLeft().onTrue(swerve.runOmegaSetTime(0.05));
        // auxController.povRight().onTrue(swerve.runOmegaSetTime(-0.05));

        // shooter flywheel on/off

        // index feed on/off
        driverController.rightTrigger(.5).onTrue(shooterSubsystem.toggleRunIndex(true));
        driverController.rightTrigger(.5).onFalse(shooterSubsystem.toggleRunIndex(false));

        // agitator feed/unjam
        driverController.povDown().onTrue(shooterSubsystem.runToggleReverseFeeder(true));
        driverController.povDown().onFalse(shooterSubsystem.runToggleReverseFeeder(false));

        // intake wrist up/down
        auxController.a().onTrue(intakeSubsystem.toggleWristPosFlag(true));
        auxController.a().onFalse(intakeSubsystem.toggleWristPosFlag(false));
        auxController.x().onTrue(intakeSubsystem.toggleWristNegFlag(true));
        auxController.x().onFalse(intakeSubsystem.toggleWristNegFlag(false));

        // intake wrist pulses
        auxController.leftTrigger().onTrue(intakeSubsystem.addPulse());

        // intake rollers suck/repel
        auxController.b().onTrue(intakeSubsystem.toggleRollerDirection(true));
        auxController.b().onFalse(intakeSubsystem.toggleRollerDirection(false));

        // intake rollers on/off
        auxController.rightBumper().onTrue(intakeSubsystem.toggleRollerFlag(false));
        auxController.rightBumper().onFalse(intakeSubsystem.toggleRollerFlag(true));

        // shooter toggles on/off
        auxController.leftBumper().onTrue(shooterSubsystem.toggleRunShooter());

        // shooter goes to max speed
        auxController.y().onTrue(shooterSubsystem.switchMaxShooterFlag(true));
        auxController.y().onFalse(shooterSubsystem.switchMaxShooterFlag(false));
    }

    public void testPeriodic() {
        swerveSubsystem.periodic();
    }

    public Command getAutonomousCommand() {
        return autoBrain.fetchAuto().cmd();
    }
}