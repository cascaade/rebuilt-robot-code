package frc.robot.subsystems.swerve;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;

public class OrientedChassisSpeeds extends ChassisSpeeds {
    public boolean allianceFlipped;
    public boolean fieldCentric;

    /**
     * Construct an OrientedChassisSpeeds object
     */
    public OrientedChassisSpeeds() {
        super();
        this.allianceFlipped = false;
        this.fieldCentric = false;
    }

    public OrientedChassisSpeeds(double vxMetersPerSecond, double vyMetersPerSecond, double omegaRadiansPerSecond) {
        super(vxMetersPerSecond, vyMetersPerSecond, omegaRadiansPerSecond);
        this.allianceFlipped = false;
        this.fieldCentric = false;
    }

    public OrientedChassisSpeeds(LinearVelocity vx, LinearVelocity vy, AngularVelocity omega) {
        super(vx, vy, omega);
        this.allianceFlipped = false;
        this.fieldCentric = false;
    }

    public OrientedChassisSpeeds(
        double vxMetersPerSecond,
        double vyMetersPerSecond,
        double omegaRadiansPerSecond,
        boolean allianceFlipped,
        boolean fieldCentric
    ) {
        super(vxMetersPerSecond, vyMetersPerSecond, omegaRadiansPerSecond);
        this.allianceFlipped = allianceFlipped;
        this.fieldCentric = fieldCentric;
    }

    public OrientedChassisSpeeds(
        LinearVelocity vx,
        LinearVelocity vy,
        AngularVelocity omega,
        boolean allianceFlipped,
        boolean fieldCentric
    ) {
        super(vx, vy, omega);
        this.allianceFlipped = allianceFlipped;
        this.fieldCentric = fieldCentric;
    }

    public OrientedChassisSpeeds(
        ChassisSpeeds speeds,
        boolean allianceFlipped,
        boolean fieldCentric
    ) {
        super(
            speeds.vxMetersPerSecond,
            speeds.vyMetersPerSecond,
            speeds.omegaRadiansPerSecond
        );
        this.allianceFlipped = allianceFlipped;
        this.fieldCentric = fieldCentric;
    }
}