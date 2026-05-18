package frc.robot.subsystems.intake.rollers;

import lombok.Setter;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.intake.IntakeConstants.RollerConstants.ROLLERS_INTAKE_SPEED;
import static frc.robot.subsystems.intake.IntakeConstants.RollerConstants.ROLLERS_OUTTAKE_SPEED;

public class Rollers {
    public enum WantedState {
        IDLE,
        INTAKE,
        OUTTAKE
    }

    private enum SystemState {
        IDLING,
        INTAKING,
        OUTTAKING
        // todo: add jam detection
    }

    @Setter
    private WantedState wantedState = WantedState.IDLE;
    private WantedState previousWantedState = WantedState.IDLE;
    private SystemState systemState = SystemState.IDLING;

    private final RollersIO rollersIO;
    private final RollersIOInputsAutoLogged rollersIOInputs = new RollersIOInputsAutoLogged();

    public Rollers(RollersIO rollersIO) {
        this.rollersIO = rollersIO;
    }

    private SystemState handleStateTransitions() {
        switch (wantedState) {
            case INTAKE -> {
                return SystemState.INTAKING;
            }
            case OUTTAKE -> {
                return SystemState.OUTTAKING;
            }
            default -> {
                return SystemState.IDLING;
            }
        }
    }

    private void applyStates() {
        switch (systemState) {
            case INTAKING ->
                rollersIO.setClosedLoop(ROLLERS_INTAKE_SPEED);

            case OUTTAKING ->
                rollersIO.setClosedLoop(ROLLERS_OUTTAKE_SPEED);

            default ->
                rollersIO.setOpenLoop(Volts.of(0));
        }
    }

    public void periodic() {
        this.systemState = handleStateTransitions();
        applyStates();
        this.previousWantedState = wantedState;

        rollersIO.updateInputs(rollersIOInputs);
        Logger.processInputs("Intake/Rollers", rollersIOInputs);
        rollersIO.syncControlConstants();
    }
}
