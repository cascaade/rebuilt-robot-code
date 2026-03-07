package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

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

    public default void setOpenLoop(double voltage) {}
}
