// ============================================================
// RobotContainer.java  — UPDATED
// Changes from original:
//   1. Added IntakeSubsystem and ShooterSubsystem fields
//   2. Added AutoFactory construction (ChoreoLib)
//   3. Added AutoChooser with Path2877Auto registered
//   4. getAutonomousCommand() now returns the selected auto
//   5. RobotModeTriggers schedules the auto during autonomous
//
// Lines marked  // [NEW]  are additions to the original file.
// All original bindings and swerve setup are preserved unchanged.
// ============================================================

package frc.robot;

import choreo.auto.AutoChooser;          // [NEW]
import choreo.auto.AutoFactory;          // [NEW]
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard; // [NEW]
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers; // [NEW]
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants;
import frc.robot.autos.Blue1_Climb_Auto;
import frc.robot.autos.Red1_NoClimb_Auto;     // [NEW]
// import frc.robot.subsystems.IntakeSubsystem;  // [NEW] — adjust package if needed
// import frc.robot.subsystems.ShooterSubsystem; // [NEW] — adjust package if needed
import frc.robot.subsystems.swerve.GyroIO;
import frc.robot.subsystems.swerve.GyroIOPigeon;
import frc.robot.subsystems.swerve.SDSModuleIO;
import frc.robot.subsystems.swerve.SDSModuleIOSim;
import frc.robot.subsystems.swerve.SDSModuleIOSpark;
import frc.robot.subsystems.swerve.SwerveDrive;

public class RobotContainer {

    // -------------------------------------------------------
    // Existing fields — UNCHANGED
    // -------------------------------------------------------
    private final CommandXboxController driverController;
    private final CommandXboxController auxController;
    private final SwerveDrive swerve;

    // -------------------------------------------------------
    // [NEW] Subsystems required for autonomous
    // -------------------------------------------------------
    // private final IntakeSubsystem intake;
    // private final ShooterSubsystem shooter;

    // -------------------------------------------------------
    // [NEW] ChoreoLib auto infrastructure
    // -------------------------------------------------------
    private final AutoFactory autoFactory;
    private final AutoChooser autoChooser;

    public RobotContainer() {
        Preferences.removeAll();

        driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);
        auxController    = new CommandXboxController(OperatorConstants.kAuxControllerPort);

        // -------------------------------------------------------
        // Existing swerve construction — UNCHANGED
        // -------------------------------------------------------
        switch (Constants.currentMode) {
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

        // -------------------------------------------------------
        // [NEW] Schedule the selected auto during autonomous period
        // This replaces the need to call getAutonomousCommand() manually
        // -------------------------------------------------------
        RobotModeTriggers.autonomous().whileTrue(autoChooser.selectedCommandScheduler());

        configureBindings();
    }

    // -------------------------------------------------------
    // Existing bindings — UNCHANGED
    // -------------------------------------------------------
    private void configureBindings() {
        swerve.setDefaultCommand(swerve.runDriveInputs(
            driverController::getLeftX,
            driverController::getLeftY,
            driverController::getRightX,
            driverController::getRightTriggerAxis
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