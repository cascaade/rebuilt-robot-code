package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.rollers.Rollers;
import frc.robot.subsystems.intake.rollers.RollersIO;
import frc.robot.subsystems.intake.wrist.Wrist;
import frc.robot.subsystems.intake.wrist.WristIO;
import lombok.Setter;

import static edu.wpi.first.units.Units.Seconds;
import static frc.robot.subsystems.intake.IntakeConstants.WristConstants.WRIST_PULSE_DEPLOY_DURATION;
import static frc.robot.subsystems.intake.IntakeConstants.WristConstants.WRIST_PULSE_STOW_DURATION;

public class IntakeFSM extends SubsystemBase {
    private enum WantedState {
        IDLE,
        HOME,
        INTAKE,
        OUTTAKE,
        PULSE,
        STOW
    }

    private enum SystemState {
        IDLING,
        HOMING_WRIST,
        DEPLOYING,
        INTAKING,
        OUTTAKING,
        PULSING,
        STOWING
    }

    @Setter
    private WantedState wantedState = WantedState.IDLE;
    private WantedState previousWantedState = WantedState.IDLE;
    private SystemState systemState = SystemState.IDLING;

    private final Wrist wrist;
    private final Rollers rollers;

    public IntakeFSM(WristIO wristIO, RollersIO rollersIO) {
        wrist = new Wrist(wristIO);
        rollers = new Rollers(rollersIO);
    }

    private SystemState handleStateTransitions() {
        if (DriverStation.isDisabled()) {
            return SystemState.IDLING;
        }

        if (!hasHomeCompleted() && wantedState != WantedState.HOME) {
            return SystemState.IDLING;
        }

        switch (wantedState) {
            case HOME -> {
                if (!wrist.hasHomeCompleted()) {
                    return SystemState.HOMING_WRIST;
                }
                setWantedState(WantedState.IDLE);
                return SystemState.IDLING;
            }
            case INTAKE -> {
                if (!wrist.hasWristDeployed()) {
                    return SystemState.DEPLOYING;
                }

                return SystemState.INTAKING;
            }
            case OUTTAKE -> {
                if (!wrist.hasWristDeployed()) {
                    return SystemState.DEPLOYING;
                }

                return SystemState.OUTTAKING;
            }
            case PULSE -> {
                return SystemState.PULSING;
            }
            case STOW -> {
                return SystemState.STOWING;
            }
            default -> {
                return SystemState.IDLING;
            }
        }
    }

    private void applyStates() {
        switch (systemState) {
            case HOMING_WRIST -> {
                // potential issue but i dont think it will be one:
                //   we set the home state after home has completed causing eternal home loop
                wrist.setWantedState(Wrist.WantedState.HOME);
                rollers.setWantedState(Rollers.WantedState.IDLE);
            }
            case DEPLOYING -> {
                wrist.setWantedState(Wrist.WantedState.DEPLOY);
                rollers.setWantedState(Rollers.WantedState.IDLE);
            }
            case INTAKING -> {
                wrist.setWantedState(Wrist.WantedState.DEPLOY);
                rollers.setWantedState(Rollers.WantedState.INTAKE);
            }
            case OUTTAKING -> {
                wrist.setWantedState(Wrist.WantedState.DEPLOY);
                rollers.setWantedState(Rollers.WantedState.OUTTAKE);
            }
            case PULSING -> {
                rollers.setWantedState(Rollers.WantedState.IDLE);

                var completion = Timer.getFPGATimestamp() % WRIST_PULSE_DEPLOY_DURATION.plus(WRIST_PULSE_STOW_DURATION).in(Seconds);

                if (completion < WRIST_PULSE_STOW_DURATION.in(Seconds)) {
                    wrist.setWantedState(Wrist.WantedState.STOW);
                } else {
                    wrist.setWantedState(Wrist.WantedState.DEPLOY);
                }
            }
            case STOWING -> {
                rollers.setWantedState(Rollers.WantedState.IDLE);
                wrist.setWantedState(Wrist.WantedState.STOW);
            }
            default -> {
                rollers.setWantedState(Rollers.WantedState.IDLE);
                wrist.setWantedState(Wrist.WantedState.IDLE);
            }
        }
    }

    public boolean hasHomeCompleted() {
        return wrist.hasHomeCompleted();
    }

    @Override
    public void periodic() {
        this.systemState = handleStateTransitions();
        applyStates();
        this.previousWantedState = wantedState;

        wrist.periodic();
        rollers.periodic();
    }
}
