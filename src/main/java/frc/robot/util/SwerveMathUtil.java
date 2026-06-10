package frc.robot.util;

import edu.wpi.first.math.MathUtil;

/**
 * A class that bundles all methods relating to conversions and specialized math in the swerve drive
 */
public class SwerveMathUtil {
    /**
     * A record that contains information about a given translation
     * @param x the x-magnitude of the translation
     * @param y the y-magnitude of the translation
     */
    public record TranslationOutput(double x, double y) {}

    /**
     * Calculate the speed multiplier based on an input
     * @param rawSpeedFactorInput the requested (input) slowness (0-1), 1 being slowest
     * @param slowedMult the lowest the multiplier can be, limiting the output between this and 1
     * @return the gain to multiply the speeds by in swerve velocity calculations
     */
    public static double calculateSpeedFactor(double rawSpeedFactorInput, double slowedMult) {
        return 1.0 - (1.0 - slowedMult) * rawSpeedFactorInput;
    }

    /**
     * Adjust a controller input based on a curve
     * @param controllerInput the input double provided by the controller
     * @param deadband the range of input in either direction to ignore
     * @param minThreshold the minimum output if outside the deadband
     * @param steepness how steep the input curve is
     * @return the adjusted output based on the described curve
     */
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

    /**
     * Convert controller inputs into a robot velocity translation
     * @param rawX raw controller input x
     * @param rawY raw controller input y
     * @param speedFactor raw controller input speed factor
     * @return the robot velocity translation (0-1 on both scales)
     */
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

    /**
     * Convert controller inputs into a robot angular velocity
     * @param rawOmega raw controller input omega
     * @param speedFactor raw controller input speed factor
     * @return the angular velocity scale of the robot (0-1)
     */
    public static double processRotationInput(double rawOmega, double speedFactor) {
        double omega = -rawOmega; // Reverse rotation

        double deadband = 0.2;
        double minThreshold = 0.03;
        double steepness = 2.8;

        omega = adjustAxisInput(omega, deadband, minThreshold, steepness);
        return omega * speedFactor;
    }
}
