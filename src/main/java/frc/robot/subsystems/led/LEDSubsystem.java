package frc.robot.subsystems.led;

public class LEDSubsystem {
    public enum WantedState {
        DISPLAY_OFF
    }

    public enum SystemState {
        DISPLAYING_OFF
    }

    private WantedState wantedState = WantedState.DISPLAY_OFF;
    private WantedState previousWantedState = WantedState.DISPLAY_OFF;
    private SystemState systemState = SystemState.DISPLAYING_OFF;

    private final LEDControllerIO controllerIO;
    private final LEDControllerIOInputsAutoLogged controllerIOInputs;

    public LEDSubsystem(LEDControllerIO controllerIO) {
        this.controllerIO = controllerIO;
        this.controllerIOInputs = new LEDControllerIOInputsAutoLogged();
    }

    public SystemState handleStateTransitions() {
        return switch (wantedState) {
            default -> SystemState.DISPLAYING_OFF;
        };
    }

    public void applyStates() {
        switch (systemState) {
            case DISPLAYING_OFF:
                controllerIO.clearAnimation();
                controllerIO.setLEDs(0, 0, 0);
        }
    }
}
