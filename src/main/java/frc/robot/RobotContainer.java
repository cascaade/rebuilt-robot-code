// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.climb.ClimbIO;
import frc.robot.subsystems.climb.ClimbIOSpark;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.RollerIO;
import frc.robot.subsystems.intake.RollerIOSpark;
import frc.robot.subsystems.intake.WristIO;
import frc.robot.subsystems.intake.WristIOSpark;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOSparkFeeder;
import frc.robot.subsystems.shooter.ShooterIOSparkIndex;
import frc.robot.subsystems.shooter.ShooterIOTalonFlywheel;
import frc.robot.subsystems.swerve.GyroIO;
import frc.robot.subsystems.swerve.GyroIOPigeon;
import frc.robot.subsystems.swerve.SDSModuleIO;
import frc.robot.subsystems.swerve.SDSModuleIOSim;
import frc.robot.subsystems.swerve.SDSModuleIOSpark;
import frc.robot.subsystems.swerve.SwerveDrive;

public class RobotContainer {
    private final CommandXboxController driverController;
    private final CommandXboxController auxController;

    private final SwerveDrive swerve;
    private final Shooter shooter;
    private final Intake intake;
    private final Climb climb;

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
                shooter = new Shooter(
                    new ShooterIOTalonFlywheel(ShooterConstants.shooterLMotorCANID),
                    new ShooterIOTalonFlywheel(ShooterConstants.shooterMMotorCANID),
                    new ShooterIOTalonFlywheel(ShooterConstants.shooterRMotorCANID),
                    new ShooterIOSparkFeeder(ShooterConstants.feederMotorCANID),
                    new ShooterIOSparkIndex(ShooterConstants.indexMotorCANID)
                );
                intake = new Intake(
                    new RollerIOSpark(),
                    new WristIOSpark()
                );
                climb = new Climb(
                    new ClimbIOSpark()
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
                shooter = new Shooter(
                    new ShooterIO() {},
                    new ShooterIO() {},
                    new ShooterIO() {},
                    new ShooterIO() {},
                    new ShooterIO() {}
                );
                intake = new Intake(
                    new RollerIO() {},
                    new WristIO() {}
                );
                climb = new Climb(
                    new ClimbIO() {}
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
                shooter = new Shooter(
                    new ShooterIO() {},
                    new ShooterIO() {},
                    new ShooterIO() {},
                    new ShooterIO() {},
                    new ShooterIO() {}
                );
                intake = new Intake(
                    new RollerIO() {},
                    new WristIO() {}
                );
                climb = new Climb(
                    new ClimbIO() {}
                );
                break;
        }

        configureBindings();
    }

    private void configureBindings() {
        swerve.setDefaultCommand(swerve.runDriveInputs(
            driverController::getLeftX, // vx
            driverController::getLeftY, // vy
            driverController::getRightX, // omega
            driverController::getRightTriggerAxis // raw slow input
        ));

        driverController.y().onTrue(swerve.runZeroGyro());
        driverController.x().onTrue(swerve.runToggleToXPosition());
        driverController.b().onTrue(swerve.runReconfigure());

        shooter.setDefaultCommand(shooter.runStopShooter());
        driverController.rightBumper().whileTrue(shooter.runAllFromNetworkSpeed());

        driverController.x().onTrue(intake.toggleWristPose());
        driverController.leftBumper().whileTrue(intake.runRollers());
        driverController.leftBumper().whileFalse(intake.runStopRollers());
    }

    public void testPeriodic() {
        swerve.periodic();
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
