package frc.robot.subsystems.led;

import edu.wpi.first.math.MathUtil;
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
        for (int i = 0; i < buffer.length; i++)
            buffer[i] = String.format("#%02X%02X%02X", r, g, b);
        updateNTBuffer();
    }

    private void updateNTBuffer() {
        for (int i = 0; i < buffer.length; i++)
            Logger.recordOutput("lights/split/" + i, buffer[i]);
        Logger.recordOutput("lights/buffer", buffer);
    }

    @Override
    public void update() {
        if (animation == null) return;

        switch (animation.animationType()) {
            case BLINK -> {
                if ((int) (Timer.getFPGATimestamp() * animation.frameRate()) % 2 == 0) {
                    for (int i = 0; i < buffer.length; i++)
                        buffer[i] = String.format(
                            "#%02X%02X%02X",
                            animation.colors()[0][0],
                            animation.colors()[0][1],
                            animation.colors()[0][2]
                        );
                } else {
                    for (int i = 0; i < buffer.length; i++)
                        buffer[i] = "#000000";
                }
            }
            case FLOW -> {
                int[][] colors = animation.colors();

                int[] baseColor;
                int[] fillColor;

                if (colors.length >= 2) {
                    baseColor = colors[0];
                    fillColor = colors[1];
                } else {
                    baseColor = new int[] {0, 0, 0};
                    fillColor = colors[0];
                }

                int animationBufferLength = animation.endIndex() - animation.startIndex();
                int cycleLength = animationBufferLength * 2;

                int frame = Math.floorMod(
                    (int) Math.floor(Timer.getFPGATimestamp() * animation.frameRate()),
                    cycleLength
                );

                int filled;
                boolean rev;

                if (frame < animationBufferLength) {
                    filled = frame + 1;
                    rev = false;
                } else {
                    filled = cycleLength - frame - 1;
                    rev = true;
                }

                String baseHex = String.format(
                    "#%02X%02X%02X",
                    baseColor[0],
                    baseColor[1],
                    baseColor[2]
                );

                String fillHex = String.format(
                    "#%02X%02X%02X",
                    fillColor[0],
                    fillColor[1],
                    fillColor[2]
                );

                for (int i = 0; i < animationBufferLength; i++) {
                    int j = rev ? animationBufferLength - i - 1 : i;
                    buffer[animation.startIndex() + j] = i < filled ? fillHex : baseHex;
                }
            }
            case LARSON -> { // todo i dont like how frame-y these animations are, they should be smoothed
                int animationBufferLength = animation.endIndex() - animation.startIndex();
                int cycleLength = Math.max(1, animationBufferLength * 2 - 2);

                int frame = Math.floorMod(
                    (int) Math.floor(Timer.getFPGATimestamp() * animation.frameRate() * 2),
                    cycleLength
                );

                int position;
                if (frame < animationBufferLength) {
                    position = frame;
                } else {
                    position = cycleLength - frame;
                }

                int trailLength = 4;

                int r = animation.colors()[0][0];
                int g = animation.colors()[0][1];
                int b = animation.colors()[0][2];

                for (int i = 0; i < animationBufferLength; i++) {
                    int distance = Math.abs(i - position);

                    if (distance > trailLength) {
                        buffer[animation.startIndex() + i] = "#000000";
                        continue;
                    }

                    double brightness = Math.pow(
                        Math.max(0.0, 1.0 - distance / (double)(trailLength + 1)),
                        1.8
                    );

                    int scaledR = (int) Math.round(r * brightness);
                    int scaledG = (int) Math.round(g * brightness);
                    int scaledB = (int) Math.round(b * brightness);

                    buffer[animation.startIndex() + i] = String.format(
                        "#%02X%02X%02X",
                        scaledR,
                        scaledG,
                        scaledB
                    );
                }
            }
            case ROTATION -> {
                int len = animation.colors().length;
                int offset = (int) MathUtil.inputModulus(Timer.getFPGATimestamp() * animation.frameRate(), 0, len);

                for (int i = 0; i < buffer.length; i++)
                    buffer[i] = String.format("#%02X%02X%02X",
                        animation.colors()[(i + offset) % len][0],
                        animation.colors()[(i + offset) % len][1],
                        animation.colors()[(i + offset) % len][2]
                    );

            }
            default -> {
                for (int i = 0; i < buffer.length; i++)
                    buffer[i] = "#000000";
            }
        }

        updateNTBuffer();
    }
}
