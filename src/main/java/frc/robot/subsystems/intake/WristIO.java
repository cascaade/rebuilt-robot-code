package frc.robot.subsystems.intake;

import edu.wpi.first.math.geometry.Rotation2d;

import org.littletonrobotics.junction.AutoLog;

public interface WristIO {
    @AutoLog
    public static class WristIOInputs {
        public Rotation2d position = new Rotation2d();
        public double velocityRadPerSec = 0;
        
        public double appliedVolts = 0;
        public double currentAmps = 0;
    }

    public default void periodic() {}

    public default void updateInputs(WristIOInputs inputs) {}

    public default void setSetpoint(double angle) {}

    public default void resetPosition() {}

    public default void setOpenLoop(double voltage) {}
}
