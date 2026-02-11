// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.AutoAlignToHub;
import frc.robot.subsystems.Constants;
import frc.robot.subsystems.Constants.OperatorConstants;
import frc.robot.subsystems.Constants.VisionConstants;
import frc.robot.subsystems.swerve.GyroIO;
import frc.robot.subsystems.swerve.GyroIOPigeon;
import frc.robot.subsystems.swerve.SDSModuleIO;
import frc.robot.subsystems.swerve.SDSModuleIOSim;
import frc.robot.subsystems.swerve.SDSModuleIOSpark;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIOPhoton;
import frc.robot.subsystems.vision.Vision.VisionConsumer;
import frc.robot.subsystems.vision.VisionIO;

public class RobotContainer {
    private final CommandXboxController driverController;
    private final CommandXboxController auxController;

    private final SwerveDrive swerve;
    private final Vision vision;

    private final Command autoAlignCommand;

    public RobotContainer() {
        Preferences.removeAll();

        driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);
        auxController = new CommandXboxController(OperatorConstants.kAuxControllerPort);

        switch(Constants.currentMode) {
            case REAL:
                swerve = new SwerveDrive(
                    new GyroIOPigeon(),
                    new SDSModuleIOSpark(0),
                    new SDSModuleIOSpark(1),
                    new SDSModuleIOSpark(2),
                    new SDSModuleIOSpark(3)
                );
                vision = new Vision(
                    swerve::addVisionMeasurement,
                    new VisionIOPhoton(VisionConstants.cam0config.name(), VisionConstants.cam0config.robotToCamera()),
                    new VisionIOPhoton(VisionConstants.cam1config.name(), VisionConstants.cam1config.robotToCamera()),
                    new VisionIOPhoton(VisionConstants.cam2config.name(), VisionConstants.cam2config.robotToCamera()),
                    new VisionIOPhoton(VisionConstants.cam3config.name(), VisionConstants.cam3config.robotToCamera())
                );
                break;
            case SIM:
                swerve = new SwerveDrive(
                    new GyroIO() {},
                    new SDSModuleIOSim(),
                    new SDSModuleIOSim(),
                    new SDSModuleIOSim(),
                    new SDSModuleIOSim()
                );
                vision = new Vision(
                    swerve::addVisionMeasurement,
                    new VisionIO() {},
                    new VisionIO() {},
                    new VisionIO() {},
                    new VisionIO() {}
                );
                break;
            default:
                swerve = new SwerveDrive(
                    new GyroIO() {},
                    new SDSModuleIO() {},
                    new SDSModuleIO() {},
                    new SDSModuleIO() {},
                    new SDSModuleIO() {}
                );
                vision = new Vision(
                    swerve::addVisionMeasurement,
                    new VisionIO() {},
                    new VisionIO() {},
                    new VisionIO() {},
                    new VisionIO() {}
                );
                break;
        }

        autoAlignCommand = new AutoAlignToHub(swerve);

        configureBindings();
    }

    private void configureBindings() {
        swerve.setDefaultCommand(swerve.runDriveInputs(
            driverController::getLeftX, // vx
            driverController::getLeftY, // vy
            driverController::getRightX, // omega
            driverController::getRightTriggerAxis // raw slow input
        ));

        // driverController.a().whileTrue(swerve.goofyFunction());
        // driverController.a().onFalse(swerve.runStopDrive());

        driverController.a().whileTrue(autoAlignCommand);

        driverController.y().onTrue(swerve.runZeroGyro());
        driverController.x().onTrue(swerve.runToggleToXPosition());
        driverController.b().onTrue(swerve.runReconfigure());
    }

    public void testPeriodic() {
        swerve.periodic();
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
