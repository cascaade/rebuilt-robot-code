package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lombok.Setter;

public class Superstructure extends SubsystemBase {
    public enum WantedSuperState {
        DEFAULT,
        HOME
    }

    public enum CurrentSuperState {
        HOMING
    }

    @Setter
    private WantedSuperState wantedSuperState = WantedSuperState.HOME;
    private CurrentSuperState currentSuperState = CurrentSuperState.HOMING;

    public Superstructure() {

    }

    private CurrentSuperState handleStateTransitions() {
        switch (wantedSuperState) {
            default -> {
                return CurrentSuperState.HOMING;
            }
        }
    }

    private void applyStates() {
        switch (currentSuperState) {

        }
    }

    @Override
    public void periodic() {
        super.periodic();
    }
}
