// ============================================================
// Red1_NoClimb_Auto.java
// Autonomous routine for FRC robot using ChoreoLib AutoRoutine
// Red 1 path with 2 intake windows and 2 shooting positions
// Trajectory: Red1_NoClimb_Auto (13 waypoints, ~7.22 seconds)
//
// Waypoint Index Reference (0-based, matches Choreo file):
//   WP0  (t=0.000s) - Start: deploy intake
//   WP1  (t=0.705s) - Shoot for 4 seconds (stop & hold)
//   WP3  (t=2.238s) - Begin intake (run through WP6)
//   WP6  (t=3.428s) - End intake window 1
//   WP10 (t=5.916s) - Begin intake (run through WP11)
//   WP11 (t=6.406s) - End intake window 2
//   WP12 (t=7.218s) - End: shoot for 8 seconds (stop & hold)
// ============================================================

package frc.robot.autos;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;

public class Red1_NoClimb_Auto {

    // -------------------------------------------------------
    // Dependencies — injected via constructor
    // -------------------------------------------------------
    private final AutoFactory autoFactory;
    // private final IntakeSubsystem intake;
    // private final ShooterSubsystem shooter;

    // -------------------------------------------------------
    // Trajectory timestamps extracted from Red1_NoClimb_Auto.traj
    // These correspond to the waypoint array in the .traj file
    // -------------------------------------------------------

    // Waypoint 1 (index 0): robot start — deploy intake immediately
    private static final double T_WP0_START        = 0.0;

    // Waypoint 2 (index 1): shooter fires, robot stops for 4 seconds
    private static final double T_WP1_SHOOT        = 0.70516;
    private static final double SHOOT_DURATION_WP1 = 4.0; // seconds

    // Waypoint 4 (index 3): intake window begins
    private static final double T_WP3_INTAKE_START = 2.23795;

    // Waypoint 7 (index 6): intake window ends
    private static final double T_WP6_INTAKE_END   = 3.42836;

    // Waypoint 11 (index 10): second intake window begins
    private static final double T_WP10_INTAKE_START = 5.916;

    // Waypoint 12 (index 11): second intake window ends
    private static final double T_WP11_INTAKE_END   = 6.40557;

    // Waypoint 13 (index 12): final shooter fires, robot stops for 8 seconds
    private static final double T_WP12_SHOOT        = 7.21794;
    private static final double SHOOT_DURATION_WP12 = 8.0; // seconds

    // -------------------------------------------------------
    // Constructor
    // -------------------------------------------------------
    public Red1_NoClimb_Auto(
            AutoFactory autoFactory
            // IntakeSubsystem intake,
            // ShooterSubsystem shooter
            ) {
            
        this.autoFactory = autoFactory;
        // this.intake = intake;
        // this.shooter = shooter;
    }

    // -------------------------------------------------------
    // buildRoutine()
    // Call this from RobotContainer.getAutonomousCommand()
    // Returns the fully configured AutoRoutine
    // -------------------------------------------------------
    public AutoRoutine buildRoutine() {

        // Create a new named routine
        AutoRoutine routine = autoFactory.newRoutine("Red1_NoClimb_Auto");

        // Load the single trajectory — must match the filename in
        // src/main/deploy/choreo/Red1_NoClimb_Auto.traj (no extension needed)
        AutoTrajectory path = routine.trajectory("Red1_NoClimb_Auto");

        // ===================================================
        // ENTRY POINT: When autonomous starts
        // 1. Reset odometry to match the trajectory start pose
        // 2. Deploy the intake (open it up)
        // 3. Begin following the trajectory
        // ===================================================
        routine.active().onTrue(
            Commands.sequence(
                path.resetOdometry(),                  // sync robot pose to WP0
                // intake.deploy(),                       // open/deploy intake mechanism
                path.cmd()                             // start trajectory following
            )
        );

        // ===================================================
        // WAYPOINT 2 (index 1, t ≈ 0.705s)
        // Robot reaches shooter position — stop and shoot for 4 seconds
        //
        // atTime() pauses the trigger until the trajectory clock
        // reaches this timestamp. onTrue() fires once at that moment.
        // The trajectory is NOT interrupted — the robot must be
        // configured with StopPoint constraints at this waypoint
        // (already set in the .traj file: "from":1 StopPoint).
        // ===================================================
        path.atTime(T_WP1_SHOOT).onTrue(
            Commands.sequence(
                // shooter.shoot(),                       // start shooter
                // new WaitCommand(SHOOT_DURATION_WP1),   // hold for 4 seconds
                // shooter.stop()                         // stop shooter after duration
            )
        );

        // ===================================================
        // WAYPOINT 4 → 7 (index 3 → 6, t ≈ 2.238s → 3.428s)
        // Run intake continuously while travelling through this segment
        //
        // atTime(start) fires intake ON when trajectory reaches WP4
        // atTime(end)   fires intake OFF when trajectory reaches WP7
        // ===================================================
        path.atTime(T_WP3_INTAKE_START).onTrue(null
            // intake.intake()                            // start intake rollers
        );

        path.atTime(T_WP6_INTAKE_END).onTrue(null
            // intake.stop()                              // stop intake rollers
        );

        // ===================================================
        // WAYPOINT 11 → 12 (index 10 → 11, t ≈ 5.916s → 6.406s)
        // Second intake window — same pattern as above
        // ===================================================
        path.atTime(T_WP10_INTAKE_START).onTrue(null
            // intake.intake()                            // start intake rollers
        );

        path.atTime(T_WP11_INTAKE_END).onTrue(null
            // intake.stop()                              // stop intake rollers
        );

        // ===================================================
        // WAYPOINT 13 (index 12, t ≈ 7.218s — trajectory end)
        // Robot reaches final scoring position — stop and shoot for 8 seconds
        //
        // done() fires for exactly ONE scheduler cycle after the
        // trajectory finishes. Use onTrue() (not whileTrue()).
        // ===================================================
        path.done().onTrue(
            Commands.sequence(
                // shooter.shoot(),                       // start shooter
                // new WaitCommand(SHOOT_DURATION_WP12),  // hold for 8 seconds
                // shooter.stop()                         // stop shooter after duration
            )
        );

        return routine;
    }
}