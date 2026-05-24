package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import frc.robot.Constants.FieldConstants;
import frc.robot.util.ShooterLUT;
import lombok.Setter;
import org.littletonrobotics.junction.Logger;

import java.util.function.Supplier;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.shooter.ShooterConstants.shooterMaxSpeed;

public class ShooterFSM {
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
                Pose2d robotPose = robotPoseSupplier.get();
                Pose2d hubPose = FieldConstants.getHubCenter();

                Distance hubDistance = Meters.of(robotPose.getTranslation().getDistance(hubPose.getTranslation()));

                AngularVelocity shooterRunSpeed = ShooterLUT.getFlywheelSpeedAtDistance(hubDistance);

                for (ShooterIO shooterIO : shooterIOs)
                    shooterIO.setClosedLoop(shooterRunSpeed);
            }

            case PASSING -> {
                for (ShooterIO shooterIO : shooterIOs)
                    shooterIO.setClosedLoop(shooterMaxSpeed);
            }

            default -> {
                for (ShooterIO shooterIO : shooterIOs)
                    shooterIO.setOpenLoop(Volts.of(0));
            }
        }
    }

    public void periodic() {
        this.systemState = handleStateTransitions();
        applyStates();
        this.previousWantedState = wantedState;

        for (int i = 0; i < shooterIOInputs.length; i++) {
            shooterIOs[i].updateInputs(shooterIOInputs[i]);
            Logger.processInputs("Shooter/Flywheel" + i, shooterIOInputs[i]);
            shooterIOs[i].syncControlConstants();
        }
    }
}
