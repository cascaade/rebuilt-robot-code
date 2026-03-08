// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
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

    private final AutoFactory autoFactory;
    private final AutoChooser autoChooser;

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
                break;
            case SIM:
                swerve = new SwerveDrive(
                    new GyroIO() {},
                    new SDSModuleIOSim(),
                    new SDSModuleIOSim(),
                    new SDSModuleIOSim(),
                    new SDSModuleIOSim()
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
                break;
        }

        // -------------------------------------------------------
        // [NEW] Instantiate intake and shooter subsystems
        // -------------------------------------------------------
        // intake  = new IntakeSubsystem();
        // shooter = new ShooterSubsystem();

        // -------------------------------------------------------
        // [NEW] Build the AutoFactory
        //   - swerve::getPose         → supplies current robot Pose2d
        //   - swerve::resetOdometry   → resets odometry to trajectory start
        //   - swerve::followTrajectory → your SwerveSample follower method
        //   - true                    → enable alliance (red/blue) flipping
        //   - swerve                  → drive subsystem requirement
        //
        // ACTION REQUIRED: Verify these method names match your SwerveDrive class.
        //   getPose()           should return Pose2d
        //   resetOdometry()     should accept Pose2d
        //   followTrajectory()  should accept SwerveSample
        // -------------------------------------------------------
        autoFactory = new AutoFactory(
            swerve::getPose,
            swerve::resetOdometry,
            swerve::followTrajectory,
            true,
            swerve
        );

        // -------------------------------------------------------
        // [NEW] Build AutoChooser and register autonomous routines
        // Additional autos can be added here with more addRoutine() calls
        // -------------------------------------------------------
        autoChooser = new AutoChooser();
        // autoChooser.addRoutine(
        //     "Red 1 No Climb Auto",
        //     () -> new Red1_NoClimb_Auto(autoFactory, intake, shooter).buildRoutine()
        // );

        // autoChooser.addRoutine(
        //     "Blue 1 Climb Auto",
        //     () -> new Blue1_Climb_Auto(autoFactory, intake, shooter, climber).buildRoutine()
        // );

        SmartDashboard.putData("Auto Chooser", autoChooser);

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
    }

    public void testPeriodic() {
        swerve.periodic();
    }

    // -------------------------------------------------------
    // getAutonomousCommand() is preserved for compatibility
    // but the auto is now also scheduled via RobotModeTriggers
    // above, so this is only needed if Robot.java calls it directly.
    // -------------------------------------------------------
    public Command getAutonomousCommand() {
        return autoChooser.selectedCommandScheduler();
    }
}
