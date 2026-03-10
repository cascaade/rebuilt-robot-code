// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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

    private SendableChooser<String> autoChoose = new SendableChooser<>();
    private SendableChooser<String> EP1 = new SendableChooser<>();
    private SendableChooser<String> EP2 = new SendableChooser<>();
    private SendableChooser<String> EP3 = new SendableChooser<>();
    private SendableChooser<String> EP4 = new SendableChooser<>();
    private SendableChooser<String> EP5 = new SendableChooser<>();
    private String lastSelected = "";
    String[] EPs1a3 = {"2", "3", "4", "5", "6", "7", "8", "N/A"};
    String[] EPs2 = {"2", "3", "4a", "4b", "5", "6a", "6b", "7", "8", "N/A"};

    public RobotContainer() {
        autoChoose.setDefaultOption("Auto 1", "Auto1");
        autoChoose.addOption("Auto 2", "Auto2");
        autoChoose.addOption("Auto 3", "Auto3");

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

        SmartDashboard.putData("Auto Chooser", autoChoose);

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
        // String auto = autoChoose.getSelected();
        // if (!lastSelected.equals(auto)) {
        //     lastSelected = autoChoose.getSelected();
        //     EP1 = new SendableChooser<>();
        //     EP2 = new SendableChooser<>();
        //     EP3 = new SendableChooser<>();
        //     EP4 = new SendableChooser<>();
        //     EP5 = new SendableChooser<>();

        //     String[] actualEPs = auto.equals("Auto2") ? EPs2 : EPs1a3;
            
        //     for (String EP : actualEPs) {
        //         EP1.addOption(EP,   EP);
        //         EP2.addOption(EP, EP);
        //         EP3.addOption(EP, EP);
        //         EP4.addOption(EP, EP);
        //         EP5.addOption(EP, EP);
        //     }
            
        //     SmartDashboard.putData("Endpoint 1", EP1);
        //     SmartDashboard.putData("Endpoint 2", EP2);
        //     SmartDashboard.putData("Endpoint 3", EP3);
        //     SmartDashboard.putData("Endpoint 4", EP4);
        //     SmartDashboard.putData("Endpoint 5", EP5);
        // }

        swerve.periodic();
    }

    public void initAutoChooser() {
        String auto = autoChoose.getSelected();
        if (!lastSelected.equals(auto)) {
            lastSelected = autoChoose.getSelected();
            EP1 = new SendableChooser<>();
            EP2 = new SendableChooser<>();
            EP3 = new SendableChooser<>();
            EP4 = new SendableChooser<>();
            EP5 = new SendableChooser<>();

            String[] actualEPs = auto.equals("Auto2") ? EPs2 : EPs1a3;
            
            for (String EP : actualEPs) {
                EP1.addOption(EP,   EP);
                EP2.addOption(EP, EP);
                EP3.addOption(EP, EP);
                EP4.addOption(EP, EP);
                EP5.addOption(EP, EP);
            }
            
            SmartDashboard.putData("Endpoint 1", EP1);
            SmartDashboard.putData("Endpoint 2", EP2);
            SmartDashboard.putData("Endpoint 3", EP3);
            SmartDashboard.putData("Endpoint 4", EP4);
            SmartDashboard.putData("Endpoint 5", EP5);
        }
    }

    // -------------------------------------------------------
    // getAutonomousCommand() is preserved for compatibility
    // but the auto is now also scheduled via RobotModeTriggers
    // above, so this is only needed if Robot.java calls it directly.
    // -------------------------------------------------------
    public Command getAutonomousCommand() {
        return Commands.none();
    }
}
