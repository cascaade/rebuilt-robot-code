// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.autos.AutoBrain;
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
                    new VisionIOPhoton(VisionConstants.camConfigs[1]),
                    new VisionIOPhoton(VisionConstants.camConfigs[2]),
                    new VisionIOPhoton(VisionConstants.camConfigs[3])
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
                    Pose2d::new
                );
                intake = new Intake(
                    new RollerIO() {},
                    new WristIO() {}
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
                    Pose2d::new
                );
                intake = new Intake(
                    new RollerIO() {},
                    new WristIO() {}
                );
                break;
        }
        
        configureBindings();

        autoBrain = new AutoBrain(swerve, shooter, intake);
    }

    private void configureBindings() {
        // swerve default joystick inputs
        swerve.setDefaultCommand(swerve.runDriveInputs(
            driverController::getLeftX,          // vx
            driverController::getLeftY,          // vy
            driverController::getRightX,         // omega
            driverController::getLeftTriggerAxis // raw slow input
        ));

        // reset robot orientation (doesn't work with vision or FMS)
        driverController.y().onTrue(swerve.runZeroGyro());

        // hub aim on/off
        driverController.leftBumper().onTrue(swerve.runToggleAimHub(true));
        driverController.leftBumper().onFalse(swerve.runToggleAimHub(false));

        // swerve adjustments using POV
        // auxController.povUp().onTrue(swerve.runXSetTime(-0.15));
        // auxController.povDown().onTrue(swerve.runXSetTime(0.15));
        // auxController.povLeft().onTrue(swerve.runOmegaSetTime(0.05));
        // auxController.povRight().onTrue(swerve.runOmegaSetTime(-0.05));

        // shooter flywheel on/off

        // index feed on/off
        driverController.rightTrigger(.5).onTrue(shooter.toggleRunIndex(true));
        driverController.rightTrigger(.5).onFalse(shooter.toggleRunIndex(false));

        // agitator feed/unjam
        driverController.povDown().onTrue(shooter.runToggleReverseFeeder(true));
        driverController.povDown().onFalse(shooter.runToggleReverseFeeder(false));

        // intake wrist up/down
        auxController.a().onTrue(intake.toggleWristPosFlag(true));
        auxController.a().onFalse(intake.toggleWristPosFlag(false));
        auxController.x().onTrue(intake.toggleWristNegFlag(true));
        auxController.x().onFalse(intake.toggleWristNegFlag(false));

        // intake wrist pulses
        auxController.leftTrigger().onTrue(intake.addPulse());

        // intake rollers suck/repel
        auxController.b().onTrue(intake.toggleRollerDirection(true));
        auxController.b().onFalse(intake.toggleRollerDirection(false));

        // intake rollers on/off
        auxController.rightBumper().onTrue(intake.toggleRollerFlag(false));
        auxController.rightBumper().onFalse(intake.toggleRollerFlag(true));

        // shooter toggles on/off
        auxController.leftBumper().onTrue(shooter.toggleRunShooter());

        // shooter goes to max speed
        auxController.y().onTrue(shooter.switchMaxShooterFlag(true));
        auxController.y().onFalse(shooter.switchMaxShooterFlag(false));
    }

    public void testPeriodic() {
        swerve.periodic();
    }

    public Command getAutonomousCommand() {
        return autoBrain.fetchAuto().cmd();
    }
}