package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface RollerIO {
    @AutoLog
    public static class IntakeIOInputs {
        public double positionRad = 0;
        public double velocityRadPerSec = 0;
        
        public double appliedVolts = 0;
        public double currentAmps = 0;
    }

    public default void periodic() {}

    public default void updateInputs(IntakeIOInputs inputs) {}

    public default void setClosedLoop(double velocityRadPerSec) {}

    public default void setOpenLoop(double voltage) {}
}
