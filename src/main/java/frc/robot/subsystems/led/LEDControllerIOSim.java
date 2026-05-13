package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.Timer;
import org.littletonrobotics.junction.Logger;

import static frc.robot.subsystems.led.LEDConstants.*;

public class LEDControllerIOSim implements LEDControllerIO {
    private final String[] buffer;
    private LEDAnimation animation;

    public LEDControllerIOSim() {
        this.buffer = new String[BUFFER_LENGTH];

        for (int i = 0; i < buffer.length; i++) {
            buffer[i] = "#000000";
        }
    }

    @Override
    public void clearAnimation() {
        animation = null;
    }

    @Override
    public void setAnimation(LEDAnimation animation) {
        if (
            Math.min(Math.max(animation.startIndex(), 0), BUFFER_LENGTH) == animation.startIndex()
                && Math.min(Math.max(animation.endIndex(), 0), BUFFER_LENGTH) == animation.endIndex()
                && animation.startIndex() < animation.endIndex()
        ) {
            this.animation = animation;
        } else {
            throw new RuntimeException("[LEDs] Tried to set unacceptable animation");
        }
    }

    @Override
    public void setLEDs(int r, int g, int b) {
        for (int i = 0; i < buffer.length; i++) {
            buffer[i] = String.format("#%02X%02X%02X", r, g, b);
            Logger.recordOutput("lights/split/" + i, buffer[i]);
        }
        Logger.recordOutput("lights/buffer", buffer);
    }

    @Override
    public void update() {
        if (animation == null) return;

        switch (animation.animationType()) {
            case BLINK:
                if ((int) (Timer.getFPGATimestamp() * animation.frameRate()) % 2 == 0) {
                    setLEDs(
                        animation.colors()[0][0],
                        animation.colors()[0][1],
                        animation.colors()[0][2]
                    );
                } else {
                    setLEDs(0, 0, 0);
                }
                break;
            default:
                setLEDs(0, 0, 0);
        }
    }
}
