package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lombok.Setter;

import static frc.robot.subsystems.led.LEDAnimation.LEDAnimationType.*;
import static frc.robot.subsystems.led.LEDConstants.*;

public class LEDSubsystem extends SubsystemBase {
    public enum WantedState {
        DISPLAY_OFF,
        DISABLED,
        DISCONNECTED,
        AUTONOMOUS,
        ENABLED,
        BOOT
    }

    public enum SystemState {
        DISPLAYING_OFF,
        DISABLED,
        DISCONNECTED,
        AUTONOMOUS,
        ENABLED,
        BOOT
    }

    @Setter
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
            case DISABLED -> SystemState.DISABLED;
            case DISCONNECTED -> SystemState.DISCONNECTED;
            case AUTONOMOUS -> SystemState.AUTONOMOUS;
            case ENABLED -> SystemState.ENABLED;
            case BOOT -> SystemState.BOOT;
        };
    }

    public void applyStates() {
        switch (systemState) {
            case DISPLAYING_OFF:
                controllerIO.clearAnimation();
                controllerIO.setLEDs(0, 0, 0);
                break;
            case DISABLED:
                controllerIO.clearAnimation();
                controllerIO.setLEDs(255, 0, 0);
                break;
            case DISCONNECTED:
                controllerIO.setAnimation(new LEDAnimation(
                    BLINK,
                    0,
                    BUFFER_LENGTH,
                        1,
                    new int[] { 255, 0, 0 }
                ));
                break;
            case AUTONOMOUS:
                controllerIO.setAnimation(new LEDAnimation(
                    BLINK,
                    0,
                    BUFFER_LENGTH,
                    2,
                    new int[] { 255, 160, 0 }
                ));
                break;
            case ENABLED:
                controllerIO.setAnimation(new LEDAnimation(
                    BLINK,
                    0,
                    BUFFER_LENGTH,
                    2,
                    new int[] { 0, 255, 0 }
                ));
                break;
            case BOOT:
                controllerIO.setAnimation(new LEDAnimation(
                    ROTATION,
                    0,
                    BUFFER_LENGTH,
                    -4,
                    new int[] { 0, 0, 255 },
                    new int[] { 0, 0, 255 },
                    new int[] { 255, 225, 0 },
                    new int[] { 255, 225, 0 }
                ));
                break;
            default:
                System.out.println("Couldn't find match for LEDSubsystem.SystemState");
        }
    }

    @Override
    public void periodic() {
        this.systemState = handleStateTransitions();
        this.previousWantedState = wantedState;

        applyStates();

        // sample use cases
        if (Timer.getFPGATimestamp() < 10) {
            setWantedState(WantedState.BOOT);
        } else if (DriverStation.isDSAttached()) {
            if (DriverStation.isAutonomous())
                setWantedState(WantedState.AUTONOMOUS);
            else if (DriverStation.isEnabled())
                setWantedState(WantedState.ENABLED);
            else
                setWantedState(WantedState.DISABLED);
        } else {
            setWantedState(WantedState.DISCONNECTED);
        }

        // needed for sim only
        controllerIO.update();
    }
}
