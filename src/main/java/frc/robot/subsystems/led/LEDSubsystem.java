package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class LEDSubsystem extends SubsystemBase {
    public enum WantedState {
        DISPLAY_OFF,
        BLINK
    }

    public enum SystemState {
        DISPLAYING_OFF,
        BLINK
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
            case DISPLAY_OFF -> SystemState.DISPLAYING_OFF;
            case BLINK -> SystemState.BLINK;
        };
    }

    public void applyStates() {
        switch (systemState) {
            case DISPLAYING_OFF:
                controllerIO.clearAnimation();
                controllerIO.setLEDs(0, 0, 0);
                break;
            case BLINK:
                controllerIO.clearAnimation();
                if ((int) Timer.getFPGATimestamp() % 2 == 0) {
                    controllerIO.setLEDs(255, 255, 255);
                } else {
                    controllerIO.setLEDs(0, 0, 0);
                }
                break;
            default:
                System.out.println("Couldn't find match for LEDSubsystem.SystemState");
        }
    }

    public void setWantedState(WantedState wantedState) {
        this.wantedState = wantedState;
    }

    @Override
    public void periodic() {
        this.systemState = handleStateTransitions();
        this.previousWantedState = wantedState;

        applyStates();

        setWantedState(WantedState.BLINK);
    }
}
