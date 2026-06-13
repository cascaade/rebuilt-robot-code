// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.autos.AutoBrain;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.WantedSuperState;
import frc.robot.subsystems.indexer.*;
import frc.robot.subsystems.intake.*;
import frc.robot.subsystems.intake.IntakeConstants.RollerConstants;
import frc.robot.subsystems.intake.IntakeConstants.WristConstants;
import frc.robot.subsystems.intake.rollers.RollersIO;
import frc.robot.subsystems.intake.rollers.RollersIOSpark;
import frc.robot.subsystems.intake.wrist.WristIO;
import frc.robot.subsystems.intake.wrist.WristIOSpark;
import frc.robot.subsystems.led.LEDControllerIOSim;
import frc.robot.subsystems.led.LEDSubsystem;
import frc.robot.subsystems.shooter.*;
import frc.robot.subsystems.swerve.*;
import frc.robot.subsystems.vision.*;

public class RobotContainer {
    private final CommandXboxController controller;

    private final SwerveFSM swerveSubsystem;
    private final ShooterFSM shooterSubsystem;
    private final IndexerSubsystem indexerSubsystem;
    private final IntakeFSM intakeSubsystem;
    private final LEDSubsystem ledSubsystem;
    private final VisionSubsystem visionSubsystem;

    private final AutoBrain autoBrain;

    private final Superstructure superstructure;

    // todo: research on why this can't be anywhere but here (tried in constructor, first thing, no work)
    private final RobotState robotState = RobotState.getInstance();

    public RobotContainer() {
        Preferences.removeAll();

        controller = new CommandXboxController(OperatorConstants.OPERATOR_CONTROLLER_PORT);

        switch (Constants.currentMode) {
            case REAL:
                swerveSubsystem = new SwerveFSM(
                    controller,
                    new GyroIOPigeon(),
                    new SDSModuleIOSpark(0),
                    new SDSModuleIOSpark(1),
                    new SDSModuleIOSpark(2),
                    new SDSModuleIOSpark(3)
                );
                shooterSubsystem = new ShooterFSM(
                    new ShooterIOTalon(ShooterConstants.shooterLMotorCANID),
                    new ShooterIOTalon(ShooterConstants.shooterMMotorCANID),
                    new ShooterIOTalon(ShooterConstants.shooterRMotorCANID),
                    swerveSubsystem::getPose
                );
                indexerSubsystem = new IndexerSubsystem(
                    new ConveyorIOSpark(ShooterConstants.feederMotorCANID),
                    new KickerIOSpark(ShooterConstants.indexMotorCANID)
                );
                intakeSubsystem = new IntakeFSM(
                    new WristIOSpark(WristConstants.WRIST_CAN_ID),
                    new RollersIOSpark(RollerConstants.ROLLERS_CAN_ID)
                );
                visionSubsystem = new VisionSubsystem(
                    swerveSubsystem::addVisionMeasurement,
                    new VisionIOPhoton(VisionConstants.camConfigs[0]),
                    new VisionIOPhoton(VisionConstants.camConfigs[1]),
                    new VisionIOPhoton(VisionConstants.camConfigs[2]),
                    new VisionIOPhoton(VisionConstants.camConfigs[3])
                );
                break;
            case SIM:
                swerveSubsystem = new SwerveFSM(
                    controller,
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
                shooterSubsystem = new ShooterFSM(
                    new ShooterIOSim(),
                    new ShooterIOSim(),
                    new ShooterIOSim(),
                    swerveSubsystem::getPose
                );
                indexerSubsystem = new IndexerSubsystem(
                    new ConveyorIO() {},
                    new KickerIO() {}
                );
                intakeSubsystem = new IntakeFSM(
                    new WristIO() {},
                    new RollersIO() {}
                );
                break;
            default:
                swerveSubsystem = new SwerveFSM(
                    controller,
                    new GyroIO() {
                    },
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
                shooterSubsystem = new ShooterFSM(
                    new ShooterIO() {},
                    new ShooterIO() {},
                    new ShooterIO() {},
                    swerveSubsystem::getPose
                );
                indexerSubsystem = new IndexerSubsystem(
                    new ConveyorIO() {},
                    new KickerIO() {}
                );
                intakeSubsystem = new IntakeFSM(
                    new WristIO() {},
                    new RollersIO() {}
                );
                break;
        }

        ledSubsystem = new LEDSubsystem(
            new LEDControllerIOSim()
        );

        superstructure = new Superstructure(
            swerveSubsystem,
            intakeSubsystem,
            indexerSubsystem,
            shooterSubsystem,
            ledSubsystem
        );
        
        configureBindings();

        autoBrain = new AutoBrain(superstructure, swerveSubsystem);
    }

    private void configureBindings() {
        // swerve default joystick inputs
//        swerveSubsystem.setDefaultCommand(swerveSubsystem.runDriveInputs(
//            driverController::getLeftX,          // vx
//            driverController::getLeftY,          // vy
//            driverController::getRightX,         // omega
//            driverController::getLeftTriggerAxis // raw slow input
//        ));

        // reset robot orientation (doesn't work with vision or FMS)
//        driverController.y().onTrue(swerveSubsystem.runZeroGyro());

        // hub aim on/off
//        driverController.leftBumper().onTrue(swerveSubsystem.runToggleAimHub(true));
//        driverController.leftBumper().onFalse(swerveSubsystem.runToggleAimHub(false));

        // swerve adjustments using POV
        // auxController.povUp().onTrue(swerve.runXSetTime(-0.15));
        // auxController.povDown().onTrue(swerve.runXSetTime(0.15));
        // auxController.povLeft().onTrue(swerve.runOmegaSetTime(0.05));
        // auxController.povRight().onTrue(swerve.runOmegaSetTime(-0.05));

        // shooter flywheel on/off

        // index feed on/off
//        driverController.rightTrigger(.5).onTrue(shooterSubsystem.toggleRunIndex(true));
//        driverController.rightTrigger(.5).onFalse(shooterSubsystem.toggleRunIndex(false));
//
//        // agitator feed/unjam
//        driverController.povDown().onTrue(shooterSubsystem.runToggleReverseFeeder(true));
//        driverController.povDown().onFalse(shooterSubsystem.runToggleReverseFeeder(false));
//
//        // intake wrist up/down
//        auxController.a().onTrue(intakeSubsystem.toggleWristPosFlag(true));
//        auxController.a().onFalse(intakeSubsystem.toggleWristPosFlag(false));
//        auxController.x().onTrue(intakeSubsystem.toggleWristNegFlag(true));
//        auxController.x().onFalse(intakeSubsystem.toggleWristNegFlag(false));
//
//        // intake wrist pulses
//        auxController.leftTrigger().onTrue(intakeSubsystem.addPulse());
//
//        // intake rollers suck/repel
//        auxController.b().onTrue(intakeSubsystem.toggleRollerDirection(true));
//        auxController.b().onFalse(intakeSubsystem.toggleRollerDirection(false));
//
//        // intake rollers on/off
//        auxController.rightBumper().onTrue(intakeSubsystem.toggleRollerFlag(false));
//        auxController.rightBumper().onFalse(intakeSubsystem.toggleRollerFlag(true));
//
//        // shooter toggles on/off
//        auxController.leftBumper().onTrue(shooterSubsystem.toggleRunShooter());
//
//        // shooter goes to max speed
//        auxController.y().onTrue(shooterSubsystem.switchMaxShooterFlag(true));
//        auxController.y().onFalse(shooterSubsystem.switchMaxShooterFlag(false));

        controller.leftBumper().whileTrue(superstructure.intakeCommand());
        controller.rightTrigger().whileTrue(superstructure.shootCommand());
    }

    public void testPeriodic() {
        swerveSubsystem.periodic();
    }

    public Command getAutonomousCommand() {
        return Commands.runOnce(() -> superstructure.setWantedSuperState(WantedSuperState.DRIVE))
            .andThen(new WaitUntilCommand(superstructure::hasHomeCompleted))
            .andThen(autoBrain.fetchAuto().cmd())
            .andThen(superstructure.disableCommand());
    }
}