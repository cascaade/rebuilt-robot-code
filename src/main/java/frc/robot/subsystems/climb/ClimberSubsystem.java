package frc.robot.subsystems.climb;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.climb.ClimbConstants.ClimbPose;
import lombok.Setter;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import java.util.function.BooleanSupplier;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.climb.ClimbConstants.*;

public class ClimberSubsystem extends SubsystemBase {
    private enum WantedState {
        IDLE,
        HOME,
        STOW,
        MOVE_TO_POSITION,
        WINCH_READY,
        MANUAL_CLIMB,
        LOCKED
    }

    private enum SystemState {
        IDLING,
        HOMING,
        STOWED,
        MOVING_TO_POSITION,
        READYING,
        WINCH_READY,
        MANUAL_CLIMBING,
        LOCKED
    }

    @Setter
    private WantedState wantedState = WantedState.IDLE;
    private WantedState previousWantedState = WantedState.IDLE;
    private SystemState systemState = SystemState.IDLING;

    private double climberHomeTimestamp = Double.NaN;
    private boolean isClimberHomed = false;

    private double setpointPosition = 0;

    public final ClimbIO climbIO;
    public final ClimbIOInputsAutoLogged inputs = new ClimbIOInputsAutoLogged();

    LoggedNetworkNumber volts = new LoggedNetworkNumber("Climber/Volts", 0);

    public ClimberSubsystem(ClimbIO climbIO) {
        this.climbIO = climbIO;
    }

    public Command runExtend() {
        return run(() -> {
            climbIO.setClosedLoop(ClimbPose.EXTENDED.getSetpoint());
        });
    }

    public Command runRetract() {
        return run(() -> {
            climbIO.setClosedLoop(ClimbPose.RETRACTED.getSetpoint());
        });
    }

    public Command runClimb(
        BooleanSupplier posSupplier,
        BooleanSupplier negSupplier
    ) {
        return run(() -> {
             if (posSupplier.getAsBoolean() && negSupplier.getAsBoolean()) {
                 climbIO.setOpenLoop(Volts.of(0));
                 Logger.recordOutput("Climber/SetpointVolts", 0);
             } else if (posSupplier.getAsBoolean()) {
                 climbIO.setOpenLoop(Volts.of(volts.get()));
                 Logger.recordOutput("Climber/SetpointVolts", volts.get());
             } else if (negSupplier.getAsBoolean()) {
                 climbIO.setOpenLoop(Volts.of(-volts.get()));
                 Logger.recordOutput("Climber/SetpointVolts", -volts.get());
             } else {
                 climbIO.setOpenLoop(Volts.of(0));
                 Logger.recordOutput("Climber/SetpointVolts", 0);
             }
        });
    }

    private SystemState handleStateTransitions() {
        if (!isClimberHomed && wantedState != WantedState.HOME) {
            return SystemState.IDLING;
        }

        if (previousWantedState == WantedState.HOME && wantedState != WantedState.HOME) {
            climberHomeTimestamp = Double.NaN;
        }

        switch (wantedState) {
            case HOME -> {
                if (previousWantedState != WantedState.HOME) {
                    isClimberHomed = false;
                }

                if (DriverStation.isEnabled()) {
                    if (Math.abs(inputs.velocityRotPerSec) <= CLIMBER_ZERO_VELOCITY_THRESHOLD) {
                        if (Double.isNaN(climberHomeTimestamp)) {
                            climberHomeTimestamp = Timer.getFPGATimestamp();
                            return SystemState.HOMING;
                        } else if (Timer.getFPGATimestamp() - climberHomeTimestamp >= CLIMBER_ZERO_VELOCITY_DURATION) {
                            climberHomeTimestamp = Double.NaN;
                            isClimberHomed = true;
                            tareClimberPosition(CLIMBER_HOME_RESET_POSITION);
                            setWantedState(WantedState.IDLE);
                            return SystemState.IDLING;
                        } else {
                            return SystemState.HOMING;
                        }
                    } else {
                        climberHomeTimestamp = Double.NaN;
                        return SystemState.HOMING;
                    }
                }
            }
            case STOW -> {
                if (isAtPosition(CLIMBER_STOWED_SETPOINT)) {
                    return SystemState.STOWED;
                }

                setpointPosition = CLIMBER_STOWED_SETPOINT;
                setWantedState(WantedState.MOVE_TO_POSITION);
            }
            case MOVE_TO_POSITION -> {
                return SystemState.MOVING_TO_POSITION;
            }
            case WINCH_READY -> {
                if (isAtPosition(CLIMBER_READY_SETPOINT)) {
                    return SystemState.WINCH_READY;
                }

                setpointPosition = CLIMBER_READY_SETPOINT;
                setWantedState(WantedState.MOVE_TO_POSITION);

                /* todo: to complete this i need to sort out:
                    1. what state to be in? moving or readying
                    2. how to transition out of the state like homing does with the timer
                 */
            }
            case MANUAL_CLIMB -> {
                // todo: maybe use controller directly?
//                if (posSupplier.getAsBoolean() && negSupplier.getAsBoolean()) {
//                    climbIO.setOpenLoop(Volts.of(0));
//                } else if (posSupplier.getAsBoolean()) {
//                    climbIO.setOpenLoop(Volts.of(volts.get()));
//                } else if (negSupplier.getAsBoolean()) {
//                    climbIO.setOpenLoop(Volts.of(-volts.get()));
//                } else {
//                    climbIO.setOpenLoop(Volts.of(0));
//                }
            }
            case LOCKED -> {
                return SystemState.LOCKED;
            }
            default -> {
                return SystemState.IDLING;
            }
        }
        return SystemState.IDLING;
    }

    private void applyStates() {
        switch (systemState) {
            case MOVING_TO_POSITION -> {
                climbIO.setClosedLoop(Radians.of(setpointPosition));
            }
        }
    }

    @Override
    public void periodic() {
        climbIO.updateInputs(inputs);
        Logger.processInputs("Climber", inputs);

        this.systemState = handleStateTransitions();
        applyStates();
        this.previousWantedState = wantedState;
    }

    private void tareClimberPosition(double position) {

    }

    private boolean isAtPosition(double position) {
        return MathUtil.isNear(
            position,
            inputs.positionRadians,
            CLIMBER_SETPOINT_TOLERANCE
        );
    }
}
