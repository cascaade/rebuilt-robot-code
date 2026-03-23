package frc.robot.autos;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.swerve.SwerveDrive;

public class AutoBrain {
    private final AutoFactory autoFactory;
    private final StringPublisher autoPathPublisher;
    private final StringSubscriber autoPathSubscriber;
    private final SendableChooser<String> autoMode = new SendableChooser<>();

    private final SwerveDrive swerveSubsystem;
    private final Shooter shooterSubsystem;
    private final Intake intakeSubsystem;

    public AutoBrain(SwerveDrive swerveSubsystem, Shooter shooterSubsystem, Intake intakeSubsystem) {
        this.swerveSubsystem = swerveSubsystem;
        this.shooterSubsystem = shooterSubsystem;
        this.intakeSubsystem = intakeSubsystem;

        autoMode.setDefaultOption("Auto 1", "Auto1");
        autoMode.addOption("Auto 2", "Auto2");
        autoMode.addOption("Auto 3", "Auto3");
        SmartDashboard.putData("Auto", autoMode);

        var topic = NetworkTableInstance.getDefault()
            .getStringTopic("/SmartDashboard/Auto Path");
        autoPathPublisher = topic.publish();
        autoPathPublisher.set("1,2,5,6"); // default shown in textbox
        autoPathSubscriber = topic.subscribe("1,2,5,6");

        autoFactory = new AutoFactory(
            swerveSubsystem::getPose,
            swerveSubsystem::resetOdometry,
            swerveSubsystem::followTrajectory,
            true,
            swerveSubsystem
        );
    }

    public AutoRoutine buildAuto() {
        String autoNum = autoMode.getSelected();
        String pathName = autoPathSubscriber.get().trim();

        String[] points = pathName.split(",");

        AutoRoutine auto = autoFactory.newRoutine("auto");

        if (pathName.isEmpty()) {
            // CHANGE SHOOTER LOGIC
            AutoTrajectory path = auto.trajectory(autoNum+"__1_Preload");
            auto.active().onTrue(Commands.sequence(
                path.resetOdometry(),
                path.cmd(),
                swerveSubsystem.setImmediateCrossbuckOverride(true),
                swerveSubsystem.runToggleAimHub(),
                new WaitCommand(1),
                shooterSubsystem.toggleRunIndex(),
                new WaitCommand(2),
                swerveSubsystem.setImmediateCrossbuckOverride(false),
                shooterSubsystem.toggleRunIndex(),
                shooterSubsystem.toggleRunShooter()
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
            shooterSubsystem.toggleRunShooter(),
            paths[0].cmd()
        ));

        for (int i = 0; i < paths.length - 1; i++) {
            String EP = points[i+1];
            String pathN = pathNames[i];
            if (shouldShootAfter(EP)) {
                paths[i].done().onTrue(Commands.sequence(
                    swerveSubsystem.setImmediateCrossbuckOverride(true),
                    swerveSubsystem.runToggleAimHub(),
                    new WaitCommand(1),
                    shooterSubsystem.toggleRunIndex(),
                    new WaitCommand(2),
                    swerveSubsystem.setImmediateCrossbuckOverride(false),
                    shooterSubsystem.toggleRunIndex(),
                    shooterSubsystem.toggleRunShooter(),
                    paths[i+1].cmd()
                ));
            }
            else if (shouldIntakeDuring(pathN)) {
                paths[i].active().onTrue(intakeSubsystem.toggleRollerFlag());
                paths[i].done().onTrue(Commands.sequence(
                    intakeSubsystem.toggleRollerFlag(),
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
            // NEW SHOOTER LOGIC
            paths[paths.length - 1].done().onTrue(Commands.sequence(
                swerveSubsystem.setImmediateCrossbuckOverride(true),
                swerveSubsystem.runToggleAimHub(),
                new WaitCommand(1),
                shooterSubsystem.toggleRunIndex(),
                new WaitCommand(2),
                swerveSubsystem.setImmediateCrossbuckOverride(false),
                shooterSubsystem.toggleRunIndex(),
                shooterSubsystem.toggleRunShooter()
            ));
        }
        else if (shouldIntakeDuring(lastPathName)) {
            lastPath.active().onTrue(intakeSubsystem.toggleRollerFlag());
            lastPath.done().onTrue(intakeSubsystem.toggleRollerFlag());
        }

        return auto;
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
}
