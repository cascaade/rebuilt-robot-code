package frc.robot.subsystems.led;

public record LEDAnimation(LEDAnimationType animationType, int startIndex, int endIndex, double frameRate, int[]... colors) {
    public enum LEDAnimationType {
        BLINK,
        FLOW,
        LARSON,
        ROTATION
    }
}
