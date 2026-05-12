package frc.robot.subsystems.led;

import org.littletonrobotics.junction.Logger;

import static frc.robot.subsystems.led.LEDConstants.*;

public class LEDControllerIOSim implements LEDControllerIO {
    private final String[] buffer;

    public LEDControllerIOSim() {
        this.buffer = new String[BUFFER_LENGTH];

        for (int i = 0; i < buffer.length; i++) {
            buffer[i] = "#000000";
        }
    }

    @Override
    public void clearAnimation() {

    }

    @Override
    public void setAnimation() {

    }

    @Override
    public void setLEDs(int r, int g, int b) {
        for (int i = 0; i < buffer.length; i++) {
            buffer[i] = String.format("#%02X%02X%02X", r, g, b);
            Logger.recordOutput("lights/split/" + i, buffer[i]);
        }
        Logger.recordOutput("lights/buffer", buffer);
    }
}
