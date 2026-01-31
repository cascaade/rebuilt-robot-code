package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    public static class IntakeIOInputs {
        public double positionMeters = 0;
        public double velocityMetersPerSec = 0;
        
        public double appliedVolts = 0;
        public double currentAmps = 0;
    }

    public void updateInputs(IntakeIOInputs inputs);

    public void setClosedLoop(double voltage);

    public void setOpenLoop(double voltage);
}
