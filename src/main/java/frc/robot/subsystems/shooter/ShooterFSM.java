package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FieldConstants;
import frc.robot.RobotState;
import frc.robot.subsystems.shooter.ShooterIO.ShooterIOInputs;
import frc.robot.util.ShooterLUT;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import java.util.function.Supplier;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.shooter.ShooterConstants.SHOOTER_SPEED_TOLERANCE;
import static frc.robot.subsystems.shooter.ShooterConstants.shooterMaxSpeed;

public class ShooterFSM extends SubsystemBase {
    public enum WantedState {
        IDLE,
        RESPONSIVE,
        PASS
    }

    private enum SystemState {
        IDLING,
        RESPONDING,
        PASSING
    }

    @Setter
    private WantedState wantedState = WantedState.IDLE;
    private WantedState previousWantedState = WantedState.IDLE;
    private SystemState systemState = SystemState.IDLING;

    private final Supplier<Pose2d> robotPoseSupplier;
    private final MutAngularVelocity targetVelocity = RadiansPerSecond.mutable(0);

    private final ShooterIO[] shooterIOs;
    private final ShooterIOInputsAutoLogged[] shooterIOInputs;

    public ShooterFSM(ShooterIO shooterIOa, ShooterIO shooterIOb, ShooterIO shooterIOc, Supplier<Pose2d> robotPoseSupplier) {
        this.shooterIOs = new ShooterIO[] { shooterIOa, shooterIOb, shooterIOc };
        this.robotPoseSupplier = robotPoseSupplier;

        this.shooterIOInputs = new ShooterIOInputsAutoLogged[3];
        for (int i = 0; i < shooterIOInputs.length; i++)
            shooterIOInputs[i] = new ShooterIOInputsAutoLogged();
    }

    private SystemState handleStateTransitions() {
        switch (wantedState) {
            case RESPONSIVE -> {
                return SystemState.RESPONDING;
            }
            case PASS -> {
                return SystemState.PASSING;
            }
            default -> {
                return SystemState.IDLING;
            }
        }
    }

    private void applyStates() {
        switch (systemState) {
            case RESPONDING -> {
                targetVelocity.mut_replace(ShooterLUT.getFlywheelSpeedAtDistance(RobotState.getInstance().getFieldHubDistance()));

                for (ShooterIO shooterIO : shooterIOs)
                    shooterIO.setClosedLoop(targetVelocity);
            }

            case PASSING -> {
                targetVelocity.mut_replace(ShooterLUT.getFlywheelSpeedAtDistance(RobotState.getInstance().getClosestFieldPassDistance()));

                for (ShooterIO shooterIO : shooterIOs)
                    shooterIO.setClosedLoop(targetVelocity);
            }

            default -> {
                targetVelocity.mut_replace(Double.NaN, RadiansPerSecond);

                for (ShooterIO shooterIO : shooterIOs)
                    shooterIO.setOpenLoop(Volts.of(0));
            }
        }
    }

    public void periodic() {
        this.systemState = handleStateTransitions();
        applyStates();
        this.previousWantedState = wantedState;

        Logger.recordOutput("Shooter/WantedState", wantedState);
        Logger.recordOutput("Shooter/SystemState", systemState);

        for (int i = 0; i < shooterIOInputs.length; i++) {
            shooterIOs[i].updateInputs(shooterIOInputs[i]);
            Logger.processInputs("Shooter/Flywheel" + i, shooterIOInputs[i]);
            shooterIOs[i].syncControlConstants();
        }
    }

    @AutoLogOutput(key = "Shooter/IsAtSpeed")
    public boolean isAtSpeed() {
        boolean atSpeed = true;

        if (Double.isNaN(targetVelocity.in(RadiansPerSecond)))
            return false;

        for (ShooterIOInputs inputs : shooterIOInputs)
            atSpeed = atSpeed && inputs.velocity.isNear(targetVelocity, SHOOTER_SPEED_TOLERANCE);

        return atSpeed;
    }
}
