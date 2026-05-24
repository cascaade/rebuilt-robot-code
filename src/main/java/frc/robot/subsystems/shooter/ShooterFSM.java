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

    private final ShooterIO shooterIO;
    private final ShooterIOInputsAutoLogged shooterIOInputs = new ShooterIOInputsAutoLogged();

    public ShooterFSM(ShooterIO shooterIO, Supplier<Pose2d> robotPoseSupplier) {
        this.shooterIO = shooterIO;
        this.robotPoseSupplier = robotPoseSupplier;
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
                shooterIO.setClosedLoop(shooterRunSpeed);
            }

            case PASSING ->
                shooterIO.setClosedLoop(shooterMaxSpeed);

            default ->
                shooterIO.setOpenLoop(Volts.of(0));
        }
    }

    public void periodic() {
        this.systemState = handleStateTransitions();
        applyStates();
        this.previousWantedState = wantedState;

        shooterIO.updateInputs(shooterIOInputs);
        Logger.processInputs("Shooter", shooterIOInputs);
        shooterIO.syncControlConstants();
    }
}
