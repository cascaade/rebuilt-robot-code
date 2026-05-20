package frc.robot.util;

import edu.wpi.first.math.MathUtil;

public class SwerveMathUtil {
    public record TranslationOutput(double x, double y) {}

    public static double calculateSpeedFactor(double rawSpeedFactorInput, double slowedMult) {
        return 1.0 - (1.0 - slowedMult) * rawSpeedFactorInput;
    }

    public static double adjustAxisInput(
        double controllerInput,
        double deadband,
        double minThreshold,
        double steepness
    ) {
        // see https://www.desmos.com/calculator/wj59z401tq

        return
            Math.abs(controllerInput) > deadband ?
                MathUtil.clamp(
                    Math.signum(controllerInput) * (
                        (1 - minThreshold) *
                            Math.pow(
                                (Math.abs(controllerInput) - deadband) / (1 - deadband),
                                steepness
                            ) +
                            minThreshold
                    ),
                    -1,
                    1
                )
                : 0;
    }

    public static TranslationOutput processTranslationInputs(
        double rawX,
        double rawY,
        double speedFactor
    ) {
        // Invert joystick Y and map to NWU (North-West-Up)
        double vx = -rawY;
        double vy = -rawX;

        double mag = Math.hypot(vx, vy);
        double dir = Math.atan2(vy, vx);

        // Apply shaping (assuming adjustAxisInput is accessible here)
        double deadband = 0.2;
        double minThreshold = 0.03;
        double steepness = 1.8;

        mag = adjustAxisInput(mag, deadband, minThreshold, steepness);
        mag *= speedFactor;

        return new TranslationOutput(mag * Math.cos(dir), mag * Math.sin(dir));
    }

    public static double processRotationInput(double rawOmega, double speedFactor) {
        double omega = -rawOmega; // Reverse rotation

        double deadband = 0.2;
        double minThreshold = 0.03;
        double steepness = 2.8;

        omega = adjustAxisInput(omega, deadband, minThreshold, steepness);
        return omega * speedFactor;
    }
}
