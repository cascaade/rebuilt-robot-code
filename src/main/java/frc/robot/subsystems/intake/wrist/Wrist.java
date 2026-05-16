package frc.robot.subsystems.intake.wrist;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lombok.Setter;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.intake.IntakeConstants.WristConstants.*;

public class Wrist extends SubsystemBase {
    private enum WantedState {
        IDLE,
        STOW,
        DEPLOY,
        HOME
    }

    private enum SystemState {
        IDLING,
        STOWING,
        STOWED,
        DEPLOYING,
        DEPLOYED,
        HOMING
    }

    @Setter
    private WantedState wantedState = WantedState.IDLE;
    private WantedState previousWantedState = WantedState.IDLE;
    private SystemState systemState = SystemState.IDLING;

    private final WristIO wristIO;
    private final WristIOInputsAutoLogged wristIOInputs;

    private double wristHomeTimestamp = Double.NaN;
    private boolean isWristHomed = false;

    public Wrist(WristIO wristIO) {
        this.wristIO = wristIO;
        this.wristIOInputs = new WristIOInputsAutoLogged();
    }

    private SystemState handleStateTransitions() {
        if (!isWristHomed && wantedState != WantedState.HOME) {
            wristHomeTimestamp = Double.NaN;
            return SystemState.IDLING;
        }

        switch (wantedState) {
            case STOW -> {
                if (isAtAngle(WRIST_STOWED_SETPOINT))
                    return SystemState.STOWED;

                return SystemState.STOWING;
            }
            case DEPLOY -> {
                if (isAtAngle(WRIST_DEPLOYED_SETPOINT))
                    return SystemState.DEPLOYED;

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
                        } else if (Seconds.of(Timer.getFPGATimestamp() - wristHomeTimestamp).gte(WRIST_ZERO_VELOCITY_TIME_PERIOD)) {
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
            case STOWING, STOWED ->
                moveToPosition(WRIST_STOWED_SETPOINT);

            case DEPLOYING, DEPLOYED ->
                moveToPosition(WRIST_DEPLOYED_SETPOINT);

            case HOMING ->
                wristIO.setOpenLoop(WRIST_HOMING_VOLTAGE);

            default -> wristIO.setOpenLoop(Volts.of(0));
        }
    }

    @Override
    public void periodic() {
        this.systemState = handleStateTransitions();
        applyStates();
        this.previousWantedState = wantedState;

        wristIO.updateInputs(wristIOInputs);
    }

    private void moveToPosition(Angle angle) {
        wristIO.setSetpoint(new Rotation2d(angle));
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
