package frc.robot.subsystems.climb;

import org.littletonrobotics.junction.AutoLog;

public interface ClimbIO {
    @AutoLog
    public class ClimbIOInputs {
        public double positionRadians = 0;
        public double velocityRotPerSec = 0;

        public double appliedVolts = 0;
        public double currentAmps = 0;
    }

    public void updateInputs(ClimbIOInputs inputs);

    public void setClosedLoop(double radians);

    public void setOpenLoop(double voltage);
}
