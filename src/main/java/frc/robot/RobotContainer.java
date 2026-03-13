// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.VisionConstants;
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
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
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

    private final AutoFactory autoFactory;

    private SendableChooser<String> autoChoose = new SendableChooser<>();
    private SendableChooser<String> EP1 = new SendableChooser<>();
    private SendableChooser<String> EP2 = new SendableChooser<>();
    private SendableChooser<String> EP3 = new SendableChooser<>();
    private SendableChooser<String> EP4 = new SendableChooser<>();
    private SendableChooser<String> EP5 = new SendableChooser<>();
    private String lastSelected = "";
    String[] EPs1 = {"2", "3", "4", "5", "6", "7", "8", "Preload", "N/A"};
    String[] EPs3 = {"2", "3", "4", "5", "6", "7", "8", "N/A"};
    String[] EPs2 = {"2", "3", "4a", "4b", "5", "6a", "6b", "7", "8", "Preload", "N/A"};

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
            driverController::getLeftTriggerAxis // raw slow input
        ));
//        auxController.x().onTrue(swerve.runToggleAimHub());
        driverController.y().onTrue(swerve.runZeroGyro());
//        driverController.x().onTrue(swerve.runToggleToXPosition());
//        driverController.b().onTrue(swerve.runReconfigure());
        auxController.povUp().onTrue(swerve.runXSetTime(-0.15));
        auxController.povDown().onTrue(swerve.runXSetTime(0.15));
        auxController.povLeft().onTrue(swerve.runOmegaSetTime(0.05));
        auxController.povRight().onTrue(swerve.runOmegaSetTime(-0.05));

        driverController.rightBumper().onTrue(shooter.toggleRunShooter());
        driverController.rightTrigger(.5).onTrue(shooter.toggleRunIndex());
        driverController.rightTrigger(.5).onFalse(shooter.toggleRunIndex());
//        auxController.x().onTrue(shooter.incrementShooterDistanceAdjust(true));
//        auxController.y().onTrue(shooter.incrementShooterDistanceAdjust(false));

        // intake.setDefaultCommand(intake.runStopRollers());
        // auxController.rightBumper().toggleOnFalse(intake.runRollers());
        auxController.rightBumper().onTrue(intake.toggleRollerFlag());
        auxController.leftTrigger().onTrue(intake.toggleWristPose());
        auxController.a().onTrue(intake.incrementWristSetpointAdjust(
            true));
        auxController.b().onTrue(intake.incrementWristSetpointAdjust(false));
        auxController.leftBumper().onTrue(intake.resetPosition());
    }

    public void testPeriodic() {
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

            String[] actualEPs = auto.equals("Auto2") ? EPs2 : auto.equals("Auto1") ? EPs1 : EPs3;

            for (String EP : actualEPs) {
                EP1.addOption(EP, EP);
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

    public AutoRoutine buildAuto() {

        String autoName = autoChoose.getSelected();
        String[] points = {EP1.getSelected(), EP2.getSelected(), EP3.getSelected(), EP4.getSelected(), EP5.getSelected()};
        AutoTrajectory[] pathsTemp = new AutoTrajectory[5];
        String[] pathNamesTemp = new String[5];
        AutoRoutine auto = autoFactory.newRoutine("auto");

        pathNamesTemp[0] = autoName+"__1"+"_"+points[0];
        pathsTemp[0] = auto.trajectory(pathNamesTemp[0]);

        // Define everything in pathNameTemp and pathsTemp
        for (int i=1; i<points.length; i++) {
            if (points[i] != null || !points[i].equals("N/A")) {
                pathNamesTemp[i] = autoName+"__"+points[i-1]+"_"+points[i];
                pathsTemp[i] = auto.trajectory(pathNamesTemp[i]);
            } else {
                break;
            }
        }

        // Checks how many non-nulls there are to make new arrays without nulls
        int count = 0;
        for (String name : pathNamesTemp) {
            if (name != null) {
                count++;
            }
        }

        // Makes new arrays with no null objects
        AutoTrajectory[] paths = new AutoTrajectory[count];
        String[] pathNames = new String[5];
        for (int i=0; i<count; i++) {
            paths[i] = pathsTemp[i];
            pathNames[i] = pathNames[i];
        }


        // Checks if the path requires shooting or intaking and puts it accordingly
        for (int i=1; i<paths.length; i++) {
            if (pathNames[i].equals("Auto1__2_5") || pathNames[i].equals("Auto2__2_5") || pathNames[i].equals("Auto2__5_6a") || pathNames[i].equals("Auto2__2_3") || pathNames[i].equals("Auto3__5_6") || pathNames[i].equals("Auto1__2_3") || pathNames[i].equals("Auto2__2_4a") || pathNames[i].equals("Auto2__5_2") || pathNames[i].equals("Auto3__5_2") || pathNames[i].equals("Auto1__2_4")) {
                // paths[i].active().whileTrue(intake.runRollers()); Will work once code is merged
            }
        }

        // Checks if path is a preloading path
        if (points[0].equals("Preload")) {
            AutoTrajectory path = auto.trajectory(autoName+"__1_Preload");
            auto.active().onTrue(
            Commands.sequence(
                    path.resetOdometry(),
                    path.cmd()
                )
            );
            // path.done().onTrue(shooter.runShooterIdle().withTimeout(10)); Will work once code is merged
            return auto;
        }
        auto.active().onTrue(
            Commands.sequence(
                paths[0].resetOdometry(),
                paths[0].cmd()
            )
        );
        // chain paths together, inserting shoot when needed
        for (int i = 1; i < paths.length; i++) {
            if (shouldShootAfter(pathNames[i-1])) {
                paths[i-1].done().onTrue(
                    Commands.sequence(
                        // shooter.runShooterIdle().withTimeout(2.0), Will work once code is merged
                        paths[i].cmd()
                    )
                );
            } else {
                paths[i-1].done().onTrue(paths[i].cmd());
            }
    }
        return auto;
    }

    private boolean shouldShootAfter(String name) {
        return (name.equals("Auto3__6_8") || name.equals("Auto2__5_6a") || name.equals("Auto2__2_4a")|| name.equals("Auto2__1_5") || name.equals("Auto2__3_4") || name.equals("Auto2__5_6b") || name.equals("Auto1__2_4") || name.equals("Auto1__5_6"));
    }

    // -------------------------------------------------------
    // getAutonomousCommand() is preserved for compatibility
    // but the auto is now also scheduled via RobotModeTriggers
    // above, so this is only needed if Robot.java calls it directly.
    // -------------------------------------------------------
    public Command getAutonomousCommand() {
        return buildAuto().cmd();
    }
}
