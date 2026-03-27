// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

import choreo.auto.AutoFactory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.autos.AutoBrain;
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
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhoton;

public class RobotContainer {
    private final CommandXboxController driverController;
    private final CommandXboxController auxController;

    private final SwerveDrive swerve;
    private final Vision vision;
    private final Shooter shooter;
    private final Intake intake;
    private final Climb climb;

    private final LoggedNetworkBoolean runAutoBoolean = new LoggedNetworkBoolean("Auto/RunAuto", true);

    private final AutoBrain autoBrain;

    public RobotContainer() {
        Preferences.removeAll();

        driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);
        auxController = new CommandXboxController(OperatorConstants.kAuxControllerPort);

        switch (Constants.currentMode) {
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
                    new VisionIOPhoton(VisionConstants.camConfigs[0]),
                    new VisionIOPhoton(VisionConstants.camConfigs[1])
                );
                shooter = new Shooter(
                    new ShooterIOTalonFlywheel(ShooterConstants.shooterLMotorCANID),
                    new ShooterIOTalonFlywheel(ShooterConstants.shooterMMotorCANID),
                    new ShooterIOTalonFlywheel(ShooterConstants.shooterRMotorCANID),
                    new ShooterIOSparkFeeder(ShooterConstants.feederMotorCANID),
                    new ShooterIOSparkIndex(ShooterConstants.indexMotorCANID),
                    swerve::getPose
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
                vision = new Vision(
                    swerve::addVisionMeasurement,
                    new VisionIO() {},
                    new VisionIO() {},
                    new VisionIO() {},
                    new VisionIO() {}
                );
                shooter = new Shooter(
                    new ShooterIO() {},
                    new ShooterIO() {},
                    new ShooterIO() {},
                    new ShooterIO() {},
                    new ShooterIO() {},
                    () -> new Pose2d()
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
                vision = new Vision(
                    swerve::addVisionMeasurement,
                    new VisionIO() {},
                    new VisionIO() {},
                    new VisionIO() {},
                    new VisionIO() {}
                );
                shooter = new Shooter(
                    new ShooterIO() {},
                    new ShooterIO() {},
                    new ShooterIO() {},
                    new ShooterIO() {},
                    new ShooterIO() {},
                    () -> new Pose2d()
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

        autoBrain = new AutoBrain(swerve, shooter, intake);

        configureBindings();
    }

    private void configureBindings() {
        swerve.setDefaultCommand(swerve.runDriveInputs(
            driverController::getLeftX,          // vx
            driverController::getLeftY,          // vy
            driverController::getRightX,         // omega
            driverController::getLeftTriggerAxis // raw slow input
        ));
        driverController.leftBumper().onTrue(swerve.runToggleAimHub());
        driverController.leftBumper().onFalse(swerve.runToggleAimHub());
        driverController.y().onTrue(swerve.runZeroGyro());
//        driverController.x().onTrue(swerve.runToggleToXPosition());
//        driverController.b().onTrue(swerve.runReconfigure());
        auxController.povUp().onTrue(swerve.runXSetTime(-0.15));
        auxController.povDown().onTrue(swerve.runXSetTime(0.15));
        auxController.povLeft().onTrue(swerve.runOmegaSetTime(0.05));
        auxController.povRight().onTrue(swerve.runOmegaSetTime(-0.05));

        // shooter.setDefaultCommand(shooter.runShooter);
        auxController.y().onTrue(shooter.toggleRunShooter());
        driverController.rightTrigger(.5).onTrue(shooter.toggleRunIndex());
        driverController.rightTrigger(.5).onFalse(shooter.toggleRunIndex());
//      auxController.x().onTrue(shooter.incrementShooterDistanceAdjust(true));
//      auxController.y().onTrue(shooter.incrementShooterDistanceAdjust(false));

        // intake.setDefaultCommand(intake.runStopRollers());
//      auxController.rightBumper().toggleOnFalse(intake.runRollers());
//      auxController.leftTrigger().onTrue(intake.toggleWristPose());
//      auxController.x().onTrue(intake.moveWristWithVoltage(-1));
//      auxController.a().onTrue(intake.moveWristWithVoltage(1));
        auxController.x().onTrue(intake.toggleWristPosFlag(true));
        auxController.x().onFalse(intake.toggleWristPosFlag(false));

        auxController.a().onTrue(intake.toggleWristNegFlag(true));
        auxController.a().onFalse(intake.toggleWristNegFlag(false));
//      auxController.a().onTrue(intake.incrementWristSetpointAdjust(false));
//      auxController.leftBumper().onTrue(intake.resetPosition());
        auxController.b().onTrue(intake.toggleRollerDirection());
        auxController.rightBumper().onTrue(intake.toggleRollerFlag());
    }

    public void testPeriodic() {
        swerve.periodic();
    }

    public Command getAutonomousCommand() {
        return Commands.sequence(
            intake.toggleRollerFlag(),
            intake.toggleWristPosFlag(true),
            new WaitCommand(1),
            intake.toggleWristPosFlag(false),
            autoBrain.buildAuto().cmd()
        );
    }
}