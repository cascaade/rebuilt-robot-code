package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lombok.Setter;
import org.littletonrobotics.junction.Logger;

import java.util.EnumMap;

import static frc.robot.subsystems.led.LEDAnimation.LEDAnimationType.*;
import static frc.robot.subsystems.led.LEDConstants.*;

public class LEDSubsystem extends SubsystemBase {
    public enum WantedState {
        DISPLAY_OFF,
        DISABLED,
        DISCONNECTED,
        AUTONOMOUS,
        TELEOP,
        AIMING_HUB,
        PASSING,
        INTAKING,
        HOMING,
        SHOOT,
        BOOT
    }

    private enum SystemState {
        DISPLAYING_OFF,
        DISABLED,
        DISCONNECTED,
        AUTONOMOUS,
        TELEOP,
        AIMING_HUB,
        PASSING,
        INTAKING,
        HOMING,
        SHOOT,
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
        if (wantedState == WantedState.DISPLAY_OFF) return SystemState.DISPLAYING_OFF;
        return SystemState.valueOf(wantedState.name());
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
            case TELEOP:
                controllerIO.setAnimation(new LEDAnimation(
                    BLINK,
                    0,
                    BUFFER_LENGTH,
                    2,
                    new int[] { 0, 255, 0 }
                ));
                break;
            case AIMING_HUB:
                controllerIO.setAnimation(new LEDAnimation(
                    LARSON,
                    0,
                    BUFFER_LENGTH,
                    3,
                    new int[] { 0, 255, 0 }
                ));
                break;
            case INTAKING:
                controllerIO.setAnimation(new LEDAnimation(
                    LARSON,
                    0,
                    20,
                    12,
                    new int[] { 0, 0, 255 }
                ));
                break;
            case PASSING:
                controllerIO.setAnimation(new LEDAnimation(
                    FLOW,
                    0,
                    BUFFER_LENGTH,
                    4,
                    new int[] { 0, 0, 255 }
                ));
            case HOMING:
                controllerIO.setAnimation(new LEDAnimation(
                    FLOW,
                    0,
                    10,
                    10,
                    new int[] { 255, 255, 255 }
                ));
                break;
            case BOOT, SHOOT:
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
        applyStates();
        this.previousWantedState = wantedState;

        Logger.recordOutput("LEDs/WantedState", wantedState);
        Logger.recordOutput("LEDs/SystemState", systemState);

        // needed for sim only
        controllerIO.update();
    }
}
