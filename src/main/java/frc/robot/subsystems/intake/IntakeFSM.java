package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.rollers.Rollers;
import frc.robot.subsystems.intake.rollers.RollersIO;
import frc.robot.subsystems.intake.wrist.Wrist;
import frc.robot.subsystems.intake.wrist.WristIO;
import lombok.Setter;

public class IntakeFSM extends SubsystemBase {
    private enum WantedState {
        IDLE,
        HOME,
        INTAKE,
        OUTTAKE,
        STOW
    }

    private enum SystemState {
        IDLING,
        HOMING_WRIST,
        DEPLOYING_FOR_INTAKE,
        INTAKING,
        OUTTAKING,
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
                    return SystemState.DEPLOYING_FOR_INTAKE;
                }

                return SystemState.INTAKING;
            }
            case OUTTAKE -> {
                if (!wrist.hasWristDeployed()) {
                    return SystemState.DEPLOYING_FOR_INTAKE;
                }

                return SystemState.OUTTAKING;
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
            case DEPLOYING_FOR_INTAKE -> {
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
