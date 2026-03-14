// ============================================================
// Blue1_Climb_Auto.java
// Autonomous routine for FRC robot using ChoreoLib AutoRoutine
// Trajectory: Path_1768_2 (13 waypoints, ~9.25 seconds total)
//
// Waypoint Index Reference (0-based, from trajectory.waypoints array):
//   Index 0  (t=0.000s)  - Start: deploy intake, keep deployed all auto
//   Index 2  (t=1.322s)  - Intake rollers ON  (PDF waypoint 3)
//   Index 4  (t=5.004s)  - Intake rollers OFF (PDF waypoint 5)
//   Index 8  (t=6.855s)  - StopPoint: shoot for 7 seconds (PDF waypoint 9)
//   Index 10 (t=8.399s)  - StopPoint: climber extends (PDF waypoint 11/12)
//   Index 11 (t=8.997s)  - StopPoint: climber climbs (PDF waypoint 12/13)
//   Index 12 (t=9.250s)  - Trajectory end (final StopPoint)
//
// StopPoints confirmed in .traj constraints:
//   "first", "last", index 9, index 10, index 11, index 12
// ============================================================

package frc.robot.autos;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
//import frc.robot.subsystems.IntakeSubsystem;
//import frc.robot.subsystems.ShooterSubsystem;
//import frc.robot.subsystems.ClimberSubsystem;

public class Blue1_Climb_Auto {

    // -------------------------------------------------------
    // Dependencies — injected via constructor
    // -------------------------------------------------------
    private final AutoFactory autoFactory;
    //private final IntakeSubsystem intake;
   // private final ShooterSubsystem shooter;
   // private final ClimberSubsystem climber;

    // -------------------------------------------------------
    // Timestamps from Path_1768_2.traj — trajectory.waypoints array:
    // [0.0, 0.65316, 1.32185, 3.13244, 5.00358, 5.56949,
    //  6.04223, 6.42896, 6.85522, 7.45843, 8.39903, 8.99746, 9.2503]
    // -------------------------------------------------------

    // Index 0 (t=0.000s): Start — deploy intake, stays deployed all auto
    private static final double T_WP0_START            = 0.0;

    // Index 2 (t=1.322s): Intake rollers ON
    // This is the start of the slow straight segment (MaxVel=0.75 m/s,
    // MaxAngVel=0, KeepInLane active between index 2 and 4)
    private static final double T_WP2_INTAKE_START     = 1.32185;

    // Index 4 (t=5.004s): Intake rollers OFF
    // End of the slow straight collection segment
    private static final double T_WP4_INTAKE_END       = 5.00358;

    // Index 8 (t=6.855s): STOP AND SHOOT FOR 7 SECONDS
    // StopPoint constraint confirmed at index 9 in .traj constraints
    // (Choreo constraint indices are 0-based, "from":9 = waypoint index 9
    // in the params array = the 9th waypoint = t=7.458s)
    // NOTE: Check against your PDF — if shooter fires at PDF waypoint 9
    // (0-based index 8), use t=6.85522. If PDF waypoint 9 = 1-based index 9
    // (0-based index 8), t=6.85522 is correct.
    private static final double T_WP8_SHOOT            = 6.85522;
    private static final double SHOOT_DURATION         = 7.0; // seconds

    // Index 10 (t=8.399s): CLIMBER EXTENDS (pre-position above Rung 1)
    // StopPoint confirmed at index 10 in .traj constraints
    private static final double T_WP10_CLIMBER_EXTEND  = 8.39903;

    // Index 11 (t=8.997s): CLIMBER CLIMBS (robot lifts off ground)
    // StopPoint confirmed at index 11 in .traj constraints
    private static final double T_WP11_CLIMBER_CLIMB   = 8.99746;

    // -------------------------------------------------------
    // Constructor
    // -------------------------------------------------------
    public Blue1_Climb_Auto(
           AutoFactory autoFactory
        //     IntakeSubsystem intake,
        //     ShooterSubsystem shooter,
        //    ClimberSubsystem climber
           ) {
        this.autoFactory = autoFactory;
      //  this.intake = intake;
       // this.shooter = shooter;
      //  this.climber = climber;
    }

    // -------------------------------------------------------
    // buildRoutine()
    // Register in RobotContainer via:
    //   autoChooser.addRoutine("Blue 1 Climb Auto",
    //       () -> new Blue1_Climb_Auto(autoFactory, intake, shooter, climber)
    //                 .buildRoutine());
    // -------------------------------------------------------
    public AutoRoutine buildRoutine() {

        AutoRoutine routine = autoFactory.newRoutine("Blue1_Climb_Auto");

        // Trajectory file must be deployed to:
        // src/main/deploy/choreo/Blue1_Climb_Auto.traj
        AutoTrajectory path = routine.trajectory("Blue1_Climb_Auto");

        // ===================================================
        // ENTRY POINT (index 0, t=0.0s)
        // Reset odometry to trajectory start pose, deploy intake
        // mechanism (stays open for entire auto), begin path.
        // ===================================================
        routine.active().onTrue(
            Commands.sequence(
                path.resetOdometry(),
               // intake.deploy(),            // deploy once — stays open all auto
                path.cmd()
            )
        );

        // ===================================================
        // INDEX 2 → 4 (t=1.322s → 5.004s) — INTAKE WINDOW
        // Run intake rollers during the slow straight segment.
        // MaxVelocity=0.75 m/s, MaxAngVel=0, KeepInLane are all
        // active — robot moves slowly in a straight line while
        // collecting game pieces between these waypoints.
        // ===================================================
        path.atTime(T_WP2_INTAKE_START).onTrue(null
           // intake.intake()             // start intake rollers
        );

        path.atTime(T_WP4_INTAKE_END).onTrue(null
            //intake.stop()               // stop rollers, mechanism stays deployed
        );

        // ===================================================
        // INDEX 8 (t=6.855s) — STOP AND SHOOT FOR 7 SECONDS
        // StopPoint constraint guarantees robot is stationary here.
        // Shoot, hold for 7 seconds, then stop shooter before
        // trajectory resumes toward the climb position.
        // ===================================================
        path.atTime(T_WP8_SHOOT).onTrue(
            Commands.sequence(
               // shooter.shoot(),
                // new WaitCommand(SHOOT_DURATION),
               // shooter.stop()
            )
        );

        // ===================================================
        // INDEX 10 (t=8.399s) — CLIMBER EXTENDS
        // StopPoint constraint guarantees robot is stationary.
        // Climber arms extend upward to reach above Rung 1.
        // Pre-positions the robot before the final climb pull.
        // ===================================================
        path.atTime(T_WP10_CLIMBER_EXTEND).onTrue(null
           // climber.extend()            // raise climber arms above Rung 1
        );

        // ===================================================
        // INDEX 11 (t=8.997s) — CLIMBER CLIMBS
        // StopPoint constraint guarantees robot is stationary.
        // Climber retracts/pulls down — robot lifts off ground.
        // ===================================================
        path.atTime(T_WP11_CLIMBER_CLIMB).onTrue(null
           // climber.climb()             // retract climber, robot lifts off floor
        );

        // Index 12 (t=9.250s) — trajectory ends, robot is hanging.
        // No additional action needed at path.done().

        return routine;
    }
}