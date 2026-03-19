
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
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

    private final AutoFactory autoFactory;
    private final StringPublisher autoPathPublisher;
    private final StringSubscriber autoPathSubscriber;
    private final SendableChooser<String> autoMode = new SendableChooser<>();

    public RobotContainer() {
        autoMode.setDefaultOption("Auto 1", "Auto1");
        autoMode.addOption("Auto 2", "Auto2");
        autoMode.addOption("Auto 3", "Auto3");
        SmartDashboard.putData("Auto", autoMode);

        var topic = NetworkTableInstance.getDefault()
            .getStringTopic("/SmartDashboard/Auto Path");
        autoPathPublisher = topic.publish();
        autoPathPublisher.set("1,2,5,6"); // default shown in textbox
        autoPathSubscriber = topic.subscribe("1,2,5,6");

        Preferences.removeAll();

        driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);
        auxController = new CommandXboxController(OperatorConstants.kAuxControllerPort);

        // var topic = NetworkTableInstance.getDefault().getStringTopic("/SmartDashboard/Auto Path");

        // StringPublisher autoPathPublisher = topic.publish();
        // StringSubscriber autoPathSubscriber = topic.subscribe("Auto1__2_5");;

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

        autoFactory = new AutoFactory(
            swerve::getPose,
            swerve::resetOdometry,
            swerve::followTrajectory,
            true,
            swerve
            );

        configureBindings();
    }

    private void configureBindings() {
        swerve.setDefaultCommand(swerve.runDriveInputs(
            driverController::getLeftX, // vx
            driverController::getLeftY, // vy
            driverController::getRightX, // omega
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

        auxController.y().onTrue(shooter.toggleRunShooter());
        driverController.rightTrigger(.5).onTrue(shooter.toggleRunIndex());
        driverController.rightTrigger(.5).onFalse(shooter.toggleRunIndex());
    //    auxController.x().onTrue(shooter.incrementShooterDistanceAdjust(true));
    //    auxController.y().onTrue(shooter.incrementShooterDistanceAdjust(false));

        // intake.setDefaultCommand(intake.runStopRollers());
//         auxController.rightBumper().toggleOnFalse(intake.runRollers());
        //        auxController.leftTrigger().onTrue(intake.toggleWristPose());
//        auxController.x().onTrue(intake.moveWristWithVoltage(-1));
//        auxController.a().onTrue(intake.moveWristWithVoltage(1));
        auxController.x().onTrue(intake.toggleWristPosFlag(true));
        auxController.x().onFalse(intake.toggleWristPosFlag(false));

        auxController.a().onTrue(intake.toggleWristNegFlag(true));
        auxController.a().onFalse(intake.toggleWristNegFlag(false));
//        auxController.a().onTrue(intake.incrementWristSetpointAdjust(false));
//        auxController.leftBumper().onTrue(intake.resetPosition());
        auxController.b().onTrue(intake.toggleRollerDirection());
        auxController.rightBumper().onTrue(intake.toggleRollerFlag());
    }

    public AutoRoutine buildAuto() {
        String autoNum = autoMode.getSelected();
        String pathName = autoPathSubscriber.get();
        
        String[] points = pathName.split(",");
        
        AutoRoutine auto = autoFactory.newRoutine("auto");

        if (pathName.isEmpty()) {
            AutoTrajectory path = auto.trajectory(autoNum+"__1_Preload");
            auto.active().onTrue(Commands.sequence(
                path.resetOdometry(),
                path.cmd(),
                shooter.toggleRunShooter(),
                new WaitCommand(5),
                shooter.toggleRunIndex(),
                new WaitCommand(5),
                shooter.toggleRunIndex(),
                shooter.toggleRunShooter()
            ));
            return auto;
        }

        if (points.length < 2) {
            System.out.println("Not enough points");
            return auto;
        }

        String[] pathNames = new String[points.length-1];
        AutoTrajectory[] paths = new AutoTrajectory[points.length-1];

        for (int i=0; i<points.length-1; i++) {
            pathNames[i] = autoNum+"__"+points[i]+"_"+points[i+1];
            paths[i] = auto.trajectory(pathNames[i]);
        }

        // Debug only
        for(String s : pathNames) {
            System.out.println(s);
        }

        auto.active().onTrue(Commands.sequence(
            paths[0].resetOdometry(),
            paths[0].cmd()
        ));

        for (int i = 0; i < paths.length - 1; i++) {
            String EP = points[i+1];
            String pathN = pathNames[i];
            if (shouldShootAfter(EP)) {
                paths[i].done().onTrue(Commands.sequence(
                    shooter.toggleRunShooter(),
                    new WaitCommand(5),
                    shooter.toggleRunIndex(),
                    new WaitCommand(5),
                    shooter.toggleRunIndex(),
                    shooter.toggleRunShooter(),
                    paths[i+1].cmd()
                ));
            } 
            else if (shouldIntakeDuring(pathN)) {
                paths[i].active().onTrue(intake.toggleRollerFlag());
                paths[i].done().onTrue(Commands.sequence(
                    intake.toggleRollerFlag(),
                    paths[i+1].cmd()
                ));
            }
            else {
                paths[i].done().onTrue(paths[i+1].cmd());
            }
        }

        String lastEP = points[points.length-1];
        AutoTrajectory lastPath = paths[paths.length-1];
        String lastPathName = pathNames[pathNames.length-1];

        if (shouldShootAfter(lastEP)) {
            paths[paths.length - 1].done().onTrue(Commands.sequence(
                shooter.toggleRunShooter(),
                new WaitCommand(5),
                shooter.toggleRunIndex(),
                new WaitCommand(5),
                shooter.toggleRunIndex(),
                shooter.toggleRunShooter()
            ));
        }
        else if (shouldIntakeDuring(lastPathName)) {
            lastPath.active().onTrue(intake.toggleRollerFlag());
            lastPath.done().onTrue(intake.toggleRollerFlag());
        }

        return auto;
        
        // pathNames = new String[pathName.length()-1];
        // AutoTrajectory[] paths = new AutoTrajectory[pathName.length()-1];
        // for (int i=0; i<pathName.length()-1; i++) {
        //     AutoTrajectory path = auto.trajectory(autoNum+"__"+pathName.substring(i, i+1)+"_"+pathName.substring(i+1, i+2));
        //     pathNames[i] = autoNum+"__"+pathName.substring(i, i+1)+"_"+pathName.substring(i+1, i+2);
        //     paths[i] = path;
        // }
        
        // for(String s : pathNames) {
        //     System.out.println(s);
        // }
        
        // auto.active().onTrue(Commands.sequence(
        //     paths[0].resetOdometry(),
        //     paths[0].cmd()
        // ));
        // for (int i=1; i<pathNames.length-1; i++) {
        //     String EP = pathNames[i].substring(pathNames[i].length()-2);
        //     if (EP.equals("4a") || EP.equals("4b") || EP.equals("_4") || EP.equals("_6") 
        //     || EP.equals("6a") || EP.equals("6b")) {
        //         paths[i].done().onTrue(
        //         Commands.sequence(
        //             shooter.toggleRunShooter(),
        //             new WaitCommand(5),
        //             shooter.toggleRunIndex(),
        //             new WaitCommand(5),
        //             shooter.toggleRunIndex(),
        //             shooter.toggleRunShooter(),
        //             paths[i+1].cmd()
        //         )
        //     );
        //     }
        //     else {
        //         paths[i].done().onTrue(paths[i+1].cmd());
        //     }
        // }
        
        // return auto;
    }

    private boolean shouldShootAfter(String EP) {
        return EP.equals("4a") || EP.equals("4b") || EP.equals("4") || EP.equals("6") 
            || EP.equals("6a") || EP.equals("6b");
    }

    private boolean shouldIntakeDuring(String path) {
        return path.equals("Auto2__2_5") || path.equals("Auto2__5_6a") || path.equals("Auto2__2_3") ||
        path.equals("Auto3__5_6") || path.equals("Auto1__2_3") || path.equals("Auto2__2_4a") ||
        path.equals("Auto2__5_2") || path.equals("Auto1__2_5") || path.equals("Auto3__5_2") ||
        path.equals("Auto1__2_4");
    }

    public void testPeriodic() {
        swerve.periodic();
    }

    public Command getAutonomousCommand() {
            return buildAuto().cmd();
    }
}
