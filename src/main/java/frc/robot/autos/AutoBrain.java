package frc.robot.autos;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import choreo.trajectory.TrajectorySample;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.WantedSuperState;
import frc.robot.subsystems.swerve.SwerveFSM;
import frc.robot.util.OperatorDashboard;

import java.util.*;

public class AutoBrain {
    private final AutoFactory autoFactory;

    private final Superstructure superstructure;
    private final SwerveFSM swerveSubsystem;

    private AutoRoutine cachedAuto;
    private String autoThatIsCached;

    public AutoBrain(Superstructure superstructure, SwerveFSM swerveSubsystem) {
        this.superstructure = superstructure;
        this.swerveSubsystem = swerveSubsystem;

        SmartDashboard.putData("Rebuild Auto", new InstantCommand(() -> {
            /* note from testing:
             * this thing will not work without creating a new thread or scheduling
             * a command because of ConcurrentModificationExceptions
             */
            new Thread(() -> {
                this.cachedAuto = null;
                buildAuto();
                System.out.println("Tried to build auto!");
                updateBuiltBoolean();
            }).start();
        }).ignoringDisable(true));

        updateBuiltBoolean();

        autoFactory = new AutoFactory(
            swerveSubsystem::getPose,
            swerveSubsystem::resetOdometry,
            superstructure::requestFollowTrajectory,
            true,
            swerveSubsystem
        );
    }

    private List<AutoTrajectory> buildPreload(String startPoint) {
        AutoRoutine auto = autoFactory.newRoutine("preload");

        AutoTrajectory path = auto.trajectory(startPoint + "__1_Preload");
        auto.active().onTrue(Commands.sequence(
            path.resetOdometry(),
            path.cmd().withName("preloadPathSequence"),
            superstructure.shootCommand().withTimeout(3)
        ));

        cachedAuto = auto;
        autoThatIsCached = "";

        var list = new ArrayList<AutoTrajectory>();
        list.add(path);

        return list;
    }

    private List<AutoTrajectory> buildWaypoints(String startPoint, String requestedPath, String[] points) {
        AutoRoutine auto = autoFactory.newRoutine("auto:" + requestedPath);

        String[] pathNames = new String[points.length - 1];
        AutoTrajectory[] paths = new AutoTrajectory[points.length - 1];

        for (int i = 0; i < points.length - 1; i++) {
            pathNames[i] = startPoint + "__" + points[i] + "_" + points[i + 1];
            paths[i] = auto.trajectory(pathNames[i]);
        }

        // Debug only
        for (String s : pathNames) {
            System.out.println(s);
        }

        auto.active().onTrue(Commands.sequence(
            paths[0].resetOdometry(),
            paths[0].cmd()
        ));

        for (int i = 0; i < paths.length - 1; i++) {
            String EP = points[i + 1];
            String pathN = pathNames[i];

            if (shouldShootAfter(EP)) {
                paths[i].done().onTrue(Commands.sequence(
                    superstructure.shootCommand().withTimeout(6),
                    paths[i + 1].cmd()
                ));
            }
            else if (shouldIntakeDuring(pathN)) {
                paths[i].active().onTrue(
                    Commands.runOnce(
                        () -> superstructure.setWantedSuperState(
                            WantedSuperState.INTAKE
                        )
                    )
                );

                paths[i].done().onTrue(
                    Commands.runOnce(
                        () -> superstructure.setWantedSuperState(
                            WantedSuperState.DRIVE
                        )
                    )
                );

                paths[i].done().onTrue(paths[i + 1].cmd());
            }
            else if (shouldWaitfor5After(EP)) {
                paths[i].done().onTrue(
                    paths[i+ 1].cmd()
                );
            }
            else {
                paths[i].done().onTrue(
                    paths[i + 1].cmd()
                );
            }
        }

        String lastEP = points[points.length - 1];
        AutoTrajectory lastPath = paths[paths.length - 1];

        if (shouldShootAfter(lastEP)) {
            paths[paths.length - 1].done().onTrue(Commands.sequence(
                superstructure.shootCommand()
            ));
        }

        var list = new ArrayList<AutoTrajectory>();
        list.addAll(Arrays.stream(paths).toList());

        cachedAuto = auto;
        autoThatIsCached = requestedPath;

        return list;
    }

    private void buildAuto() {
        String startPoint = OperatorDashboard.getSelectedAutoStartLocation();
        String requestedPath = OperatorDashboard.getRequestedAutoPath();

        String[] points = requestedPath.split(",");
        List<AutoTrajectory> trajectories;

        if (requestedPath.isEmpty()) {
            trajectories = buildPreload(startPoint);
        } else if (points.length < 2) {
            System.out.println("Not enough points");
            cachedAuto = null;
            autoThatIsCached = "";
            OperatorDashboard.getField().getObject("traj").setPoses(new ArrayList<>());
            return;
        } else {
            trajectories = buildWaypoints(startPoint, requestedPath, points);
        }

        List<Pose2d> allPoses = new ArrayList<>();

        for (AutoTrajectory traj : trajectories) {
            allPoses.addAll(
                traj.getRawTrajectory()
                    .samples()
                    .stream()
                    .map(TrajectorySample::getPose)
                    .toList()
            );
        }

        OperatorDashboard.getField().getObject("traj").setPoses(allPoses);
    }

    public AutoRoutine fetchAuto() {
        if (cachedAuto == null) {
            System.out.println("No cached auto detected. Building auto again...");
            buildAuto();
            System.out.println("Finished building auto.");
        }
        updateBuiltBoolean();
        return cachedAuto;
    }

    /**
     * Returns true if the shooter flywheel should begin spinning at the START
     * of the given path segment, so it is up to speed by the time the robot
     * reaches the shoot waypoint at the end of that segment.
     *
     * Add any path here where pre-spinning the shooter during travel is required.
     */
    private boolean shouldSpinShooterDuring(String path) {
        return path.equals("Auto3__10_6");
    }

    private boolean shouldShootAfter(String EP) {
        return EP.equals("4a") || EP.equals("4b") || EP.equals("4") || EP.equals("6")
            || EP.equals("6a") || EP.equals("6b");
    }

    private boolean shouldWaitfor5After(String EP) {
        return EP.equals("67");
    }

    private boolean shouldIntakeDuring(String path) {
        return path.equals("Auto2__2_5") || path.equals("Auto2__5_6a") || path.equals("Auto2__2_3") ||
            path.equals("Auto3__5_6") || path.equals("Auto1__2_3") || path.equals("Auto2__2_4a") ||
            path.equals("Auto2__5_2") || path.equals("Auto1__2_5") || path.equals("Auto3__5_2") ||
            path.equals("Auto1__2_4") || path.equals("Auto1__4_11") || path.equals("Auto1__2_9") || 
            path.equals("Auto3__5_10") || path.equals("Auto1__1_3") || path.equals("Auto1__5_2") || 
            path.equals("Auto1__4_3") || path.equals("Auto1__4_13");
    }

    private void updateBuiltBoolean() {
        SmartDashboard.putBoolean("AutoHasBeenBuiltAndCached", cachedAuto != null && autoThatIsCached.equals(OperatorDashboard.getRequestedAutoPath()));
    }
}