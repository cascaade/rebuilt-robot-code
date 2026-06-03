package frc.robot.subsystems.intake.wrist;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import lombok.Setter;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.intake.IntakeConstants.WristConstants.*;

public class Wrist {
    public enum WantedState {
        IDLE,
        STOW,
        DEPLOY,
        HOME
    }

    private enum SystemState {
        IDLING,
        STOWING,
        DEPLOYING,
        HOMING
    }

    @Setter
    private WantedState wantedState = WantedState.IDLE;
    private WantedState previousWantedState = WantedState.IDLE;
    private SystemState systemState = SystemState.IDLING;

    private double wristHomeTimestamp = Double.NaN;
    private boolean isWristHomed = false;

    private final WristIO wristIO;
    private final WristIOInputsAutoLogged wristIOInputs = new WristIOInputsAutoLogged();

    public Wrist(WristIO wristIO) {
        this.wristIO = wristIO;
    }

    private SystemState handleStateTransitions() {
        if (!isWristHomed && wantedState != WantedState.HOME) {
            wristHomeTimestamp = Double.NaN;
            return SystemState.IDLING;
        }

        switch (wantedState) {
            case STOW -> {
                return SystemState.STOWING;
            }
            case DEPLOY -> {
                return SystemState.DEPLOYING;
            }
            case HOME -> {
                if (previousWantedState != WantedState.HOME) {
                    isWristHomed = false;
                }

                if (!DriverStation.isDisabled()) {
                    if (wristIOInputs.velocity.isNear(RadiansPerSecond.of(0), WRIST_ZERO_VELOCITY_THRESHOLD)) {
                        if (Double.isNaN(wristHomeTimestamp)) {
                            wristHomeTimestamp = Timer.getFPGATimestamp();
                        } else if (Seconds.of(Timer.getFPGATimestamp() - wristHomeTimestamp).gte(WRIST_ZERO_VELOCITY_DURATION)) {
                            wristHomeTimestamp = Double.NaN;
                            isWristHomed = true;
                            tareWristPosition(WRIST_HOME_RESET_POSITION);
                            setWantedState(WantedState.IDLE);
                            return SystemState.IDLING;
                        }
                    } else {
                        wristHomeTimestamp = Double.NaN;
                    }
                }

                return SystemState.HOMING;
            }
            default -> {
                return SystemState.IDLING;
            }
        }
    }

    private void applyStates() {
        switch (systemState) {
            case STOWING ->
                moveToPosition(WRIST_STOWED_SETPOINT);

            case DEPLOYING ->
                moveToPosition(WRIST_DEPLOYED_SETPOINT);

            case HOMING ->
                wristIO.setOpenLoop(WRIST_HOMING_VOLTAGE);

            default -> wristIO.setOpenLoop(Volts.of(0));
        }
    }

    public void periodic() {
        this.systemState = handleStateTransitions();
        applyStates();
        this.previousWantedState = wantedState;

        Logger.recordOutput("Intake/Wrist/WantedState", wantedState);
        Logger.recordOutput("Intake/Wrist/SystemState", systemState);

        wristIO.updateInputs(wristIOInputs);
        Logger.processInputs("Intake/Wrist", wristIOInputs);
        wristIO.syncControlConstants();
    }

    public boolean hasHomeCompleted() {
        return isWristHomed;
    }

    public boolean hasWristDeployed() {
        return isAtAngle(WRIST_DEPLOYED_SETPOINT);
    }

    private void moveToPosition(Angle angle) {
        wristIO.setClosedLoop(new Rotation2d(angle));
    }

    private void tareWristPosition(Angle position) {
        wristIO.resetPosition(position);
    }

    private boolean isAtAngle(Angle angle) {
        return angle.isNear(
            wristIOInputs.position.getMeasure(),
            WRIST_SETPOINT_TOLERANCE
        );
    }
}
